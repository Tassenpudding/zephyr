/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr ADC driver for the NXP LPC 12-bit SAR ADC (the "GPADC" on the
 * LPC54xxx family). The hardware converts a whole conversion sequence on a
 * single trigger: the sequence's channel mask maps directly onto Zephyr's
 * adc_sequence.channels bitmask, so one software-triggered sequence-A
 * conversion samples every selected channel (lowest-channel-first). The
 * channel setup, conversion and result read-out are provided by the NXP
 * MCUXpresso SDK lpc_adc component (fsl_adc.c); this driver is the Zephyr glue
 * around it: power/clock/reset bring-up, calibration, and the adc_context
 * state machine.
 */

#define DT_DRV_COMPAT nxp_lpc_adc

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include <fsl_adc.h>
#include <fsl_clock.h>
#include <fsl_power.h>
#include <fsl_reset.h>

#ifdef CONFIG_ADC_MCUX_LPC_ADC_STREAM
#include <string.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_mcux_lpc.h>
#include <fsl_ctimer.h>
#include <fsl_inputmux.h>
#endif /* CONFIG_ADC_MCUX_LPC_ADC_STREAM */

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_mcux_lpc_adc);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

/* The GPADC has 12 single-ended channels (0..11). */
#define MCUX_LPC_ADC_CHANNEL_COUNT 12U

/*
 * Synchronous ADC clock divider: the ADC clock is the core clock divided by
 * (value + 1). Matches the legacy firmware (220 MHz / 3 ~= 73 MHz, below the
 * 80 MHz maximum). Offset calibration re-divides to <=30 MHz on its own.
 */
#define MCUX_LPC_ADC_SYNC_CLKDIV 2U

/*
 * The ADC clock gate (AHBCLKCTRL0.ADC0) must be enabled at least 20 us after
 * the ADC analog domain is powered on (UM10912 44.3). 25 us adds margin.
 */
#define MCUX_LPC_ADC_POWER_SETTLE_US 25U

#ifdef CONFIG_ADC_MCUX_LPC_ADC_STREAM
/*
 * Sequence-B streaming path. A CTIMER match triggers the whole sequence at the
 * configured rate; with per-conversion DMA requests each selected channel's
 * result is moved (lowest-channel-first) from the SeqB global data register
 * into a ping-pong buffer, and each full buffer is handed to RTIO.
 *
 * CTIMER1 match 3 is the fixed SoC hardware-trigger input for ADC0 SeqB
 * (trigger input 0x07, per UM10912 and the legacy firmware). The ADC0-SeqB DMA
 * request is routed to the DMA channel's trigger via INPUTMUX.
 */
#define MCUX_LPC_ADC_SEQB_TIMER     CTIMER1
#define MCUX_LPC_ADC_SEQB_MATCH     kCTIMER_Match_3
#define MCUX_LPC_ADC_SEQB_TRIGGER   0x07U
#define MCUX_LPC_ADC_STREAM_BUF_CNT 2U
#define MCUX_LPC_ADC_STREAM_SAMPLES CONFIG_ADC_MCUX_LPC_ADC_STREAM_BLOCK_SAMPLES

/* Packed header written at the top of each RTIO streaming buffer. */
struct mcux_lpc_adc_stream_header {
	uint64_t timestamp_ns;
	uint32_t sample_rate_hz;
	uint16_t vref_mv;
	uint16_t samples_per_channel;
	uint8_t num_channels;
	/* Channel ids in the order samples are interleaved (ascending). */
	uint8_t channel_ids[MCUX_LPC_ADC_CHANNEL_COUNT];
} __attribute__((__packed__));
#endif /* CONFIG_ADC_MCUX_LPC_ADC_STREAM */

struct mcux_lpc_adc_config {
	ADC_Type *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint8_t sample_time;
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pincfg;
#ifdef CONFIG_ADC_MCUX_LPC_ADC_STREAM
	const struct device *dma_dev;
	uint8_t dma_channel;
	uint32_t stream_sample_rate_hz;
	uint16_t vref_mv;
#endif /* CONFIG_ADC_MCUX_LPC_ADC_STREAM */
};

struct mcux_lpc_adc_data {
	const struct device *dev;
	struct adc_context ctx;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint32_t channels;
#ifdef CONFIG_ADC_MCUX_LPC_ADC_STREAM
	struct rtio_iodev_sqe *sqe;
	uint16_t stream_bufs[MCUX_LPC_ADC_STREAM_BUF_CNT][MCUX_LPC_ADC_STREAM_SAMPLES];
	struct dma_block_config stream_blk[MCUX_LPC_ADC_STREAM_BUF_CNT];
	uint8_t stream_channel_ids[MCUX_LPC_ADC_CHANNEL_COUNT];
	uint8_t stream_num_channels;
	uint16_t stream_samples_per_channel;
	uint32_t stream_block_bytes;
	uint8_t stream_buf_idx;
#endif /* CONFIG_ADC_MCUX_LPC_ADC_STREAM */
};

static int mcux_lpc_adc_channel_setup(const struct device *dev,
				      const struct adc_channel_cfg *channel_cfg)
{
	ARG_UNUSED(dev);

	if (channel_cfg->channel_id >= MCUX_LPC_ADC_CHANNEL_COUNT) {
		LOG_ERR("Invalid channel %u", channel_cfg->channel_id);
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Unsupported channel acquisition time");
		return -ENOTSUP;
	}

	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -ENOTSUP;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Unsupported channel gain %d", channel_cfg->gain);
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Unsupported channel reference");
		return -ENOTSUP;
	}

	return 0;
}

static int mcux_lpc_adc_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct mcux_lpc_adc_data *data = dev->data;

	if (sequence->resolution != 12U) {
		LOG_ERR("Unsupported resolution %u", sequence->resolution);
		return -ENOTSUP;
	}

	if (sequence->oversampling != 0U) {
		LOG_ERR("Oversampling is not supported");
		return -ENOTSUP;
	}

	if (sequence->channels >= BIT(MCUX_LPC_ADC_CHANNEL_COUNT)) {
		LOG_ERR("Selected channels 0x%08x exceed channel range", sequence->channels);
		return -EINVAL;
	}

	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int mcux_lpc_adc_read_async(const struct device *dev, const struct adc_sequence *sequence,
				   struct k_poll_signal *async)
{
	struct mcux_lpc_adc_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, async ? true : false, async);
	error = mcux_lpc_adc_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

static int mcux_lpc_adc_read(const struct device *dev, const struct adc_sequence *sequence)
{
	return mcux_lpc_adc_read_async(dev, sequence, NULL);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct mcux_lpc_adc_data *data = CONTAINER_OF(ctx, struct mcux_lpc_adc_data, ctx);
	const struct mcux_lpc_adc_config *config = data->dev->config;
	ADC_Type *base = config->base;
	adc_conv_seq_config_t seq = {
		.channelMask = ctx->sequence.channels,
		.triggerMask = 0U,
		.triggerPolarity = kADC_TriggerPolarityPositiveEdge,
		.enableSyncBypass = false,
		.enableSingleStep = false,
		.interruptMode = kADC_InterruptForEachSequence,
	};

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	/* Program the sequence while disabled, then launch it in software. */
	ADC_EnableConvSeqA(base, false);
	ADC_SetConvSeqAConfig(base, &seq);
	ADC_EnableConvSeqA(base, true);
	ADC_DoSoftwareTriggerConvSeqA(base);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct mcux_lpc_adc_data *data = CONTAINER_OF(ctx, struct mcux_lpc_adc_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void mcux_lpc_adc_isr(const struct device *dev)
{
	const struct mcux_lpc_adc_config *config = dev->config;
	struct mcux_lpc_adc_data *data = dev->data;
	ADC_Type *base = config->base;
	uint32_t channels = data->channels;
	adc_result_info_t info;
	uint32_t channel;

	if ((ADC_GetStatusFlags(base) & kADC_ConvSeqAInterruptFlag) == 0U) {
		return;
	}

	ADC_EnableConvSeqA(base, false);
	ADC_ClearStatusFlags(base, kADC_ConvSeqAInterruptFlag);

	/*
	 * The sequence-complete interrupt fires once the whole sequence is
	 * done, so every selected channel now holds a fresh result. Read them
	 * lowest-channel-first to match the ascending order the Zephyr ADC API
	 * expects in the sample buffer.
	 */
	while (channels != 0U) {
		channel = find_lsb_set(channels) - 1U;
		channels &= ~BIT(channel);

		if (ADC_GetChannelConversionResult(base, channel, &info)) {
			*data->buffer++ = (uint16_t)info.result;
		} else {
			/* Not expected after a sequence completes; flag it. */
			LOG_WRN("No result for ADC channel %u", channel);
			*data->buffer++ = 0U;
		}
	}

	adc_context_on_sampling_done(&data->ctx, dev);
}

static int mcux_lpc_adc_init(const struct device *dev)
{
	const struct mcux_lpc_adc_config *config = dev->config;
	struct mcux_lpc_adc_data *data = dev->data;
	ADC_Type *base = config->base;
	adc_config_t adc_config;
	uint32_t adc_freq;
	int ret;

	data->dev = dev;

	if (config->pincfg != NULL) {
		int err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);

		if (err < 0) {
			return err;
		}
	}

	/*
	 * Power up the ADC: PDEN_ADC0 together with the shared analog supply and
	 * reference domains it depends on (PDRUNCFG0 bits 9/19/23, see UM10912
	 * 7.5.84 and 44.3).
	 */
	POWER_DisablePD(kPDRUNCFG_PD_VD2_ANA);
	POWER_DisablePD(kPDRUNCFG_PD_VDDA);
	POWER_DisablePD(kPDRUNCFG_PD_VREFP);
	POWER_DisablePD(kPDRUNCFG_PD_ADC0);

	k_busy_wait(MCUX_LPC_ADC_POWER_SETTLE_US);

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret) {
		return ret;
	}

	RESET_PeripheralReset(kADC0_RST_SHIFT_RSTn);

	/*
	 * Synchronous ADC clock = core clock / (clockDividerNumber + 1), matching
	 * the known-good legacy firmware: 220 MHz / 3 ~= 73 MHz (below the 80 MHz
	 * max). This throughput (~4.5 M conversions/s) is what lets the SeqB
	 * streaming path sustain its per-channel sample rate.
	 */
	ADC_GetDefaultConfig(&adc_config);
	adc_config.clockMode = kADC_ClockSynchronousMode;
	adc_config.clockDividerNumber = MCUX_LPC_ADC_SYNC_CLKDIV;
	adc_config.resolution = kADC_Resolution12bit;
	adc_config.sampleTimeNumber = config->sample_time;
	ADC_Init(base, &adc_config);

	/*
	 * Recalibration is required after every reset before normal operation.
	 * ADC_DoOffsetCalibration divides down to <=30 MHz internally for the
	 * calibration cycle and restores CTRL afterwards.
	 */
	adc_freq = CLOCK_GetFreq(kCLOCK_CoreSysClk) / (MCUX_LPC_ADC_SYNC_CLKDIV + 1U);
	if (!ADC_DoOffsetCalibration(base, adc_freq)) {
		LOG_ERR("ADC calibration failed");
		return -EIO;
	}

	ADC_EnableInterrupts(base, kADC_ConvSeqAInterruptEnable);
	config->irq_config_func(dev);

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#ifdef CONFIG_ADC_MCUX_LPC_ADC_STREAM

/* Convert a raw 12-bit reading to the q31 value / shift the decoder API expects. */
static void mcux_lpc_adc_to_q31(q31_t *out, uint16_t raw12, uint16_t vref_mv, uint8_t shift)
{
	uint32_t scale = BIT(12);
	/* Sensitivity in uV per LSB. */
	uint32_t sensitivity = (uint32_t)vref_mv * (scale - 1U) / scale * 1000U / scale;

	*out = (q31_t)((int64_t)BIT(31 - shift) * (int64_t)sensitivity / 1000000 * (int64_t)raw12);
}

/* Per-block DMA completion: publish the just-filled ping-pong buffer to RTIO. */
static void mcux_lpc_adc_dma_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
				int status)
{
	const struct device *dev = user_data;
	const struct mcux_lpc_adc_config *config = dev->config;
	struct mcux_lpc_adc_data *data = dev->data;
	struct mcux_lpc_adc_stream_header *hdr;
	uint16_t *src;
	uint8_t *buf;
	uint32_t buf_len;
	uint32_t need;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		rtio_iodev_sqe_err(data->sqe, status);
		return;
	}

	src = data->stream_bufs[data->stream_buf_idx];
	data->stream_buf_idx ^= 1U;

	need = sizeof(struct mcux_lpc_adc_stream_header) + data->stream_block_bytes;
	if (rtio_sqe_rx_buf(data->sqe, need, need, &buf, &buf_len) != 0) {
		/*
		 * No RTIO buffer available: drop this block rather than stall the
		 * DMA. The stream keeps running; the consumer sees a gap.
		 */
		return;
	}

	hdr = (struct mcux_lpc_adc_stream_header *)buf;
	hdr->timestamp_ns = k_ticks_to_ns_floor64(k_uptime_ticks());
	hdr->sample_rate_hz = config->stream_sample_rate_hz;
	hdr->vref_mv = config->vref_mv;
	hdr->samples_per_channel = data->stream_samples_per_channel;
	hdr->num_channels = data->stream_num_channels;
	memcpy(hdr->channel_ids, data->stream_channel_ids, data->stream_num_channels);

	memcpy(buf + sizeof(*hdr), src, data->stream_block_bytes);

	rtio_iodev_sqe_ok(data->sqe, 0);
}

static void mcux_lpc_adc_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct mcux_lpc_adc_config *config = dev->config;
	struct mcux_lpc_adc_data *data = dev->data;
	const struct adc_read_config *read_cfg = iodev_sqe->sqe.iodev->data;
	ADC_Type *base = config->base;
	uint32_t channels = read_cfg->sequence->channels;
	adc_conv_seq_config_t seq = {
		.triggerMask = MCUX_LPC_ADC_SEQB_TRIGGER,
		.triggerPolarity = kADC_TriggerPolarityPositiveEdge,
		.enableSyncBypass = false,
		.enableSingleStep = false,
		.interruptMode = kADC_InterruptForEachConversion,
	};
	ctimer_config_t timer_cfg;
	ctimer_match_config_t match_cfg;
	struct dma_config dma_cfg = {0};
	uint8_t num = 0;

	data->sqe = iodev_sqe;

	/* Derive the streamed channel list (ascending) and the frame geometry. */
	for (uint8_t ch = 0; ch < MCUX_LPC_ADC_CHANNEL_COUNT; ch++) {
		if (channels & BIT(ch)) {
			data->stream_channel_ids[num++] = ch;
		}
	}
	if (num == 0U) {
		rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
		return;
	}
	data->stream_num_channels = num;
	data->stream_samples_per_channel = MCUX_LPC_ADC_STREAM_SAMPLES / num;
	data->stream_block_bytes =
		(uint32_t)data->stream_samples_per_channel * num * sizeof(uint16_t);
	data->stream_buf_idx = 0U;

	/* DMA: circular ping-pong, hardware-triggered by the ADC SeqB request. */
	for (uint8_t i = 0; i < MCUX_LPC_ADC_STREAM_BUF_CNT; i++) {
		data->stream_blk[i] = (struct dma_block_config){
			.source_address = (uint32_t)(uintptr_t)&base->SEQ_GDAT[1],
			.dest_address = (uint32_t)(uintptr_t)data->stream_bufs[i],
			.block_size = data->stream_block_bytes,
			.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
			.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
			.source_reload_en = 1U,
		};
	}
	data->stream_blk[0].next_block = &data->stream_blk[1];
	data->stream_blk[1].next_block = NULL;

	dma_cfg.dma_slot = LPC_DMA_HWTRIG_EN | LPC_DMA_TRIGPOL_HIGH_RISING;
	dma_cfg.channel_direction = MEMORY_TO_MEMORY;
	dma_cfg.source_data_size = sizeof(uint16_t);
	dma_cfg.dest_data_size = sizeof(uint16_t);
	dma_cfg.block_count = MCUX_LPC_ADC_STREAM_BUF_CNT;
	dma_cfg.head_block = &data->stream_blk[0];
	dma_cfg.complete_callback_en = 1U;
	dma_cfg.dma_callback = mcux_lpc_adc_dma_cb;
	dma_cfg.user_data = (void *)dev;

	if (dma_config(config->dma_dev, config->dma_channel, &dma_cfg) != 0) {
		rtio_iodev_sqe_err(iodev_sqe, -EIO);
		return;
	}

	/* Route the ADC0 SeqB request to the DMA channel's trigger input. */
	INPUTMUX_Init(INPUTMUX);
	INPUTMUX_AttachSignal(INPUTMUX, config->dma_channel, kINPUTMUX_Adc0SeqbIrqToDma);
	INPUTMUX_Deinit(INPUTMUX);

	if (dma_start(config->dma_dev, config->dma_channel) != 0) {
		rtio_iodev_sqe_err(iodev_sqe, -EIO);
		return;
	}

	/* Program SeqB and enable its DMA-feeding interrupt (no NVIC handler). */
	seq.channelMask = channels;
	ADC_EnableConvSeqB(base, false);
	ADC_SetConvSeqBConfig(base, &seq);
	ADC_EnableInterrupts(base, kADC_ConvSeqBInterruptEnable);
	/*
	 * Make SeqB the higher-priority sequence: a SeqB trigger preempts an
	 * in-progress SeqA (on-demand) conversion so the streamed samples are
	 * never dropped. The preempted SeqA channel is re-sampled automatically
	 * when SeqA resumes, so on-demand reads stay correct, only delayed by a
	 * few conversions (UM10912 SEQA_CTRL.LOWPRIO).
	 */
	ADC_SetConvSeqBHighPriority(base);
	ADC_EnableConvSeqB(base, true);

	/*
	 * CTIMER1 match 3 toggles at twice the sample rate, producing a square
	 * wave whose rising edges (= sample rate) trigger SeqB.
	 */
	CTIMER_GetDefaultConfig(&timer_cfg);
	CTIMER_Init(MCUX_LPC_ADC_SEQB_TIMER, &timer_cfg);
	match_cfg = (ctimer_match_config_t){
		.matchValue =
			CLOCK_GetFreq(kCLOCK_CoreSysClk) / (2U * config->stream_sample_rate_hz),
		.enableCounterReset = true,
		.enableCounterStop = false,
		.outControl = kCTIMER_Output_Toggle,
		.outPinInitState = false,
		.enableInterrupt = false,
	};
	CTIMER_SetupMatch(MCUX_LPC_ADC_SEQB_TIMER, MCUX_LPC_ADC_SEQB_MATCH, &match_cfg);
	CTIMER_StartTimer(MCUX_LPC_ADC_SEQB_TIMER);
}

/*
 * The RTIO ADC decoder API addresses channels by their 0-based position in the
 * streamed set (the order the consumer listed them / the ascending order stored
 * in the header), not by hardware channel id -- this matches the in-tree
 * consumers (samples/drivers/adc/adc_stream) and the other streaming drivers.
 * hdr->channel_ids maps each position back to its hardware channel for reference.
 */
static int mcux_lpc_adc_decoder_get_frame_count(const uint8_t *buffer, uint32_t channel,
						uint16_t *frame_count)
{
	const struct mcux_lpc_adc_stream_header *hdr =
		(const struct mcux_lpc_adc_stream_header *)buffer;

	if (channel >= hdr->num_channels) {
		return -ENOTSUP;
	}

	*frame_count = hdr->samples_per_channel;

	return 0;
}

static int mcux_lpc_adc_decoder_get_size_info(struct adc_dt_spec spec, uint32_t channel,
					      size_t *base_size, size_t *frame_size)
{
	ARG_UNUSED(spec);
	ARG_UNUSED(channel);

	*base_size = sizeof(struct adc_data);
	*frame_size = sizeof(struct adc_sample_data);

	return 0;
}

static int mcux_lpc_adc_decoder_decode(const uint8_t *buffer, uint32_t channel, uint32_t *fit,
				       uint16_t max_count, void *data_out)
{
	const struct mcux_lpc_adc_stream_header *hdr =
		(const struct mcux_lpc_adc_stream_header *)buffer;
	const uint16_t *samples =
		(const uint16_t *)(buffer + sizeof(struct mcux_lpc_adc_stream_header));
	struct adc_data *out = data_out;
	uint64_t period_ns = NSEC_PER_SEC / hdr->sample_rate_hz;
	uint8_t shift = 32U - __builtin_clz(hdr->vref_mv);
	uint8_t pos = (uint8_t)channel;
	uint16_t count = 0;

	if (channel >= hdr->num_channels) {
		return -ENOTSUP;
	}

	if (*fit >= hdr->samples_per_channel) {
		return 0;
	}

	memset(out, 0, sizeof(struct adc_data));
	out->header.base_timestamp_ns = hdr->timestamp_ns;
	out->shift = shift;

	while (count < max_count && *fit < hdr->samples_per_channel) {
		uint16_t raw = samples[(*fit) * hdr->num_channels + pos];

		out->readings[count].timestamp_delta = (uint32_t)(*fit * period_ns);
		mcux_lpc_adc_to_q31(&out->readings[count].value, (raw >> 4) & 0xFFF, hdr->vref_mv,
				    shift);
		(*fit)++;
		count++;
	}

	out->header.reading_count = count;

	return count;
}

ADC_DECODER_API_DT_DEFINE() = {
	.get_frame_count = mcux_lpc_adc_decoder_get_frame_count,
	.get_size_info = mcux_lpc_adc_decoder_get_size_info,
	.decode = mcux_lpc_adc_decoder_decode,
};

static int mcux_lpc_adc_get_decoder(const struct device *dev, const struct adc_decoder_api **api)
{
	ARG_UNUSED(dev);
	*api = &ADC_DECODER_NAME();

	return 0;
}

#endif /* CONFIG_ADC_MCUX_LPC_ADC_STREAM */

#define MCUX_LPC_ADC_INIT(n)                                                                       \
	static void mcux_lpc_adc_config_func_##n(const struct device *dev);                        \
                                                                                                   \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(n, pinctrl_0), (PINCTRL_DT_INST_DEFINE(n);))                                                                                 \
                                                                                                   \
	static DEVICE_API(adc, mcux_lpc_adc_driver_api_##n) = {                                    \
		.channel_setup = mcux_lpc_adc_channel_setup,                                       \
		.read = mcux_lpc_adc_read,                                                         \
		IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = mcux_lpc_adc_read_async,))                                                                         \
				IF_ENABLED(CONFIG_ADC_MCUX_LPC_ADC_STREAM,                                         \
			   (.submit = mcux_lpc_adc_submit,                                         \
			    .get_decoder = mcux_lpc_adc_get_decoder,)) .ref_internal =  \
						 DT_INST_PROP(n, vref_mv),                         \
	};                                                                                         \
                                                                                                   \
	static const struct mcux_lpc_adc_config mcux_lpc_adc_config_##n = {                        \
		.base = (ADC_Type *)DT_INST_REG_ADDR(n),                                           \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),              \
		.sample_time = DT_INST_PROP(n, sample_time),                                       \
		.irq_config_func = mcux_lpc_adc_config_func_##n,                                   \
		.pincfg = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pinctrl_0),                         \
				      (PINCTRL_DT_INST_DEV_CONFIG_GET(n)), (NULL)),                               \
			 IF_ENABLED(CONFIG_ADC_MCUX_LPC_ADC_STREAM,                                         \
			   (.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, seqb)),          \
			    .dma_channel = DT_INST_DMAS_CELL_BY_NAME(n, seqb, channel),            \
			    .stream_sample_rate_hz = DT_INST_PROP(n, nxp_stream_sample_rate_hz),   \
			    .vref_mv = DT_INST_PROP(n, vref_mv),)) };  \
                                                                                                   \
	static struct mcux_lpc_adc_data mcux_lpc_adc_data_##n = {                                  \
		ADC_CONTEXT_INIT_TIMER(mcux_lpc_adc_data_##n, ctx),                                \
		ADC_CONTEXT_INIT_LOCK(mcux_lpc_adc_data_##n, ctx),                                 \
		ADC_CONTEXT_INIT_SYNC(mcux_lpc_adc_data_##n, ctx),                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, mcux_lpc_adc_init, NULL, &mcux_lpc_adc_data_##n,                  \
			      &mcux_lpc_adc_config_##n, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,     \
			      &mcux_lpc_adc_driver_api_##n);                                       \
                                                                                                   \
	static void mcux_lpc_adc_config_func_##n(const struct device *dev)                         \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), mcux_lpc_adc_isr,           \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(MCUX_LPC_ADC_INIT)
