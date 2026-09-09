/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc_rit

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include <fsl_rit.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(counter_mcux_lpc_rit, CONFIG_COUNTER_LOG_LEVEL);

/*
 * The RIT counts up to a single 48 bit compare value and can clear itself on
 * the match. That one compare register is what produces the wrap, so it backs
 * the counter top value and the device has no alarm channels.
 */
struct counter_mcux_lpc_rit_config {
	struct counter_config_info info;
	RIT_Type *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(const struct device *dev);
};

struct counter_mcux_lpc_rit_data {
	counter_top_callback_t top_callback;
	void *top_user_data;
	uint32_t freq;
};

/*
 * The SDK has no call for this: RIT_ClearCounter() is a deprecated alias for
 * RIT_SetCountAutoClear() and does not touch the counter.
 */
static void counter_mcux_lpc_rit_reset(RIT_Type *base)
{
	base->COUNTER = 0U;
	base->COUNTER_H = 0U;
}

static int counter_mcux_lpc_rit_start(const struct device *dev)
{
	const struct counter_mcux_lpc_rit_config *config = dev->config;

	RIT_StartTimer(config->base);

	return 0;
}

static int counter_mcux_lpc_rit_stop(const struct device *dev)
{
	const struct counter_mcux_lpc_rit_config *config = dev->config;

	RIT_StopTimer(config->base);

	return 0;
}

static int counter_mcux_lpc_rit_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_mcux_lpc_rit_config *config = dev->config;

	*ticks = (uint32_t)RIT_GetCounterTimerCount(config->base);

	return 0;
}

static int counter_mcux_lpc_rit_set_top_value(const struct device *dev,
					      const struct counter_top_cfg *cfg)
{
	const struct counter_mcux_lpc_rit_config *config = dev->config;
	struct counter_mcux_lpc_rit_data *data = dev->data;
	bool was_running;
	uint32_t current;
	int ret = 0;

	if (cfg->ticks > config->info.max_top_value) {
		return -ENOTSUP;
	}

	data->top_callback = cfg->callback;
	data->top_user_data = cfg->user_data;

	/* RIT_SetTimerCompare() stops the timer and does not start it again. */
	was_running = (config->base->CTRL & RIT_CTRL_RITEN_MASK) != 0U;
	RIT_SetTimerCompare(config->base, cfg->ticks);

	if ((cfg->flags & COUNTER_TOP_CFG_DONT_RESET) == 0U) {
		counter_mcux_lpc_rit_reset(config->base);
	} else {
		current = (uint32_t)RIT_GetCounterTimerCount(config->base);
		if (current >= cfg->ticks) {
			/*
			 * Already past the new top. The match only tests for
			 * equality, so without a reset the next one is a full
			 * 48 bit wrap away.
			 */
			if ((cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) != 0U) {
				counter_mcux_lpc_rit_reset(config->base);
			}
			ret = -ETIME;
		}
	}

	RIT_ClearStatusFlags(config->base, (uint32_t)kRIT_TimerFlag);

	if (was_running) {
		RIT_StartTimer(config->base);
	}

	return ret;
}

static uint32_t counter_mcux_lpc_rit_get_top_value(const struct device *dev)
{
	const struct counter_mcux_lpc_rit_config *config = dev->config;

	return (uint32_t)RIT_GetCompareTimerCount(config->base);
}

static uint32_t counter_mcux_lpc_rit_get_pending_int(const struct device *dev)
{
	const struct counter_mcux_lpc_rit_config *config = dev->config;

	return (RIT_GetStatusFlags(config->base) & (uint32_t)kRIT_TimerFlag) != 0U ? 1U : 0U;
}

static uint32_t counter_mcux_lpc_rit_get_freq(const struct device *dev)
{
	const struct counter_mcux_lpc_rit_data *data = dev->data;

	return data->freq;
}

static void counter_mcux_lpc_rit_isr(const struct device *dev)
{
	const struct counter_mcux_lpc_rit_config *config = dev->config;
	struct counter_mcux_lpc_rit_data *data = dev->data;

	RIT_ClearStatusFlags(config->base, (uint32_t)kRIT_TimerFlag);

	if (data->top_callback != NULL) {
		data->top_callback(dev, data->top_user_data);
	}
}

static int counter_mcux_lpc_rit_init(const struct device *dev)
{
	const struct counter_mcux_lpc_rit_config *config = dev->config;
	struct counter_mcux_lpc_rit_data *data = dev->data;
	rit_config_t rit_config;
	int ret;

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_get_rate(config->clock_dev, config->clock_subsys, &data->freq);
	if (ret != 0) {
		return ret;
	}

	RIT_GetDefaultConfig(&rit_config);
	RIT_Init(config->base, &rit_config);

	/* Wrap at the top value rather than run on to the full 48 bit range. */
	RIT_SetCountAutoClear(config->base, true);
	RIT_SetMaskBit(config->base, 0);
	RIT_SetTimerCompare(config->base, config->info.max_top_value);

	/* RIT_Init() enables the timer; the counter API expects it stopped. */
	RIT_StopTimer(config->base);

	config->irq_config_func(dev);

	return 0;
}

static DEVICE_API(counter, counter_mcux_lpc_rit_api) = {
	.start = counter_mcux_lpc_rit_start,
	.stop = counter_mcux_lpc_rit_stop,
	.get_value = counter_mcux_lpc_rit_get_value,
	.set_top_value = counter_mcux_lpc_rit_set_top_value,
	.get_top_value = counter_mcux_lpc_rit_get_top_value,
	.get_pending_int = counter_mcux_lpc_rit_get_pending_int,
	.get_freq = counter_mcux_lpc_rit_get_freq,
};

#define COUNTER_MCUX_LPC_RIT_INIT(n)                                                               \
	static void counter_mcux_lpc_rit_irq_config_##n(const struct device *dev)                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),                     \
			    counter_mcux_lpc_rit_isr, DEVICE_DT_INST_GET(n), 0);           \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static struct counter_mcux_lpc_rit_data counter_mcux_lpc_rit_data_##n;                     \
                                                                                                   \
	static const struct counter_mcux_lpc_rit_config counter_mcux_lpc_rit_config_##n = {        \
		.info = {                                                                          \
			.max_top_value = UINT32_MAX,                                               \
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,                                     \
			.channels = 0,                                                             \
		},                                                                                 \
		.base = (RIT_Type *)DT_INST_REG_ADDR(n),                                           \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),              \
		.irq_config_func = counter_mcux_lpc_rit_irq_config_##n,                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, counter_mcux_lpc_rit_init, NULL, &counter_mcux_lpc_rit_data_##n,  \
			      &counter_mcux_lpc_rit_config_##n, POST_KERNEL,                       \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_mcux_lpc_rit_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_MCUX_LPC_RIT_INIT)
