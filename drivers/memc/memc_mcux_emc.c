/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc_emc

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/clock.h>
#include <soc.h>
#include <fsl_emc.h>

LOG_MODULE_REGISTER(memc_mcux_emc, CONFIG_MEMC_LOG_LEVEL);

/*
 * Per-SDRAM-chip descriptor built from a "nxp,lpc-emc-sdram" child node. It
 * carries both the HAL chip configuration and the raw datasheet timings (in ns
 * / EMC clock cycles), because the write-recovery timing depends on the EMC
 * clock frequency, which is only known at runtime.
 */
struct emc_sdram_desc {
	emc_dynamic_chip_config_t chip;
	uint32_t refresh_ns;
	uint32_t trp_ns;
	uint32_t tras_ns;
	uint32_t tsrex_ns;
	uint32_t tapr_ns;
	uint32_t twr_ns;
	uint32_t trc_ns;
	uint32_t trfc_ns;
	uint32_t txsr_ns;
	uint32_t trrd_ns;
	uint8_t tmrd_nclk;
};

struct memc_emc_config {
	EMC_Type *base;
	const struct pinctrl_dev_config *pincfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	emc_basic_config_t basic;
	const struct emc_sdram_desc *sdram;
	uint8_t sdram_count;
	uint8_t fbclk_delay;
};

#define EMC_SDRAM_DESC(node)                                                                       \
	{                                                                                          \
		.chip =                                                                            \
			{                                                                          \
				.chipIndex = DT_REG_ADDR(node),                                    \
				.dynamicDevice = kEMC_Sdram,                                       \
				.rAS_Nclk = DT_PROP(node, nxp_ras_latency),                        \
				.sdramModeReg = DT_PROP(node, nxp_sdram_mode_register),            \
				.sdramExtModeReg = 0,                                              \
				.devAddrMap = DT_PROP(node, nxp_address_mapping),                  \
			},                                                                         \
		.refresh_ns = DT_PROP(node, refresh_period_ns),                                    \
		.trp_ns = DT_PROP(node, t_rp_ns),                                                  \
		.tras_ns = DT_PROP(node, t_ras_ns),                                                \
		.tsrex_ns = DT_PROP(node, t_srex_ns),                                              \
		.tapr_ns = DT_PROP(node, t_apr_ns),                                                \
		.twr_ns = DT_PROP(node, t_wr_ns),                                                  \
		.trc_ns = DT_PROP(node, t_rc_ns),                                                  \
		.trfc_ns = DT_PROP(node, t_rfc_ns),                                                \
		.txsr_ns = DT_PROP(node, t_xsr_ns),                                                \
		.trrd_ns = DT_PROP(node, t_rrd_ns),                                                \
		.tmrd_nclk = DT_PROP(node, nxp_mode_register_delay),                               \
	},

static int memc_emc_init(const struct device *dev)
{
	const struct memc_emc_config *config = dev->config;
	const struct emc_sdram_desc *sdram = config->sdram;
	emc_dynamic_chip_config_t chips[CONFIG_MEMC_MCUX_EMC_MAX_SDRAM_CHIPS];
	emc_dynamic_timing_config_t timing = {0};
	emc_basic_config_t basic = config->basic;
	uint32_t emc_freq;
	int ret;

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret) {
		return ret;
	}

	/*
	 * EMC_Init programs EMCCLKDIV/EMCSYSCTRL, so the EMC clock frequency is
	 * only valid afterwards. The write-recovery timing is derived from it.
	 */
	EMC_Init(config->base, &basic);

	/*
	 * Align read-data capture with the SDRAM round-trip delay. The reset
	 * default is too small once the EMC runs fast, corrupting multi-beat
	 * reads; the board supplies the correct feedback-clock delay.
	 */
	if (config->fbclk_delay != 0) {
		SYSCON->EMCDLYCTRL = SYSCON_EMCDLYCTRL_FBCLK_DELAY(config->fbclk_delay);
	}

	ret = clock_control_get_rate(config->clock_dev, config->clock_subsys, &emc_freq);
	if (ret) {
		return ret;
	}

	for (uint8_t i = 0; i < config->sdram_count; i++) {
		chips[i] = sdram[i].chip;
	}

	/*
	 * The controller applies one shared (worst-case) timing set to every
	 * chip; the reference design uses a single SDRAM, so take the timing
	 * from the first chip.
	 */
	timing.readConfig = kEMC_Cmddelay;
	timing.refreshPeriod_Nanosec = sdram[0].refresh_ns;
	timing.tRp_Ns = sdram[0].trp_ns;
	timing.tRas_Ns = sdram[0].tras_ns;
	timing.tSrex_Ns = sdram[0].tsrex_ns;
	timing.tApr_Ns = sdram[0].tapr_ns;
	/* tWR is the datasheet value plus one EMC clock period. */
	timing.tWr_Ns = (NSEC_PER_SEC / emc_freq) + sdram[0].twr_ns;
	timing.tDal_Ns = timing.tWr_Ns + sdram[0].trp_ns;
	timing.tRc_Ns = sdram[0].trc_ns;
	timing.tRfc_Ns = sdram[0].trfc_ns;
	timing.tXsr_Ns = sdram[0].txsr_ns;
	timing.tRrd_Ns = sdram[0].trrd_ns;
	timing.tMrd_Nclk = sdram[0].tmrd_nclk;

	EMC_DynamicMemInit(config->base, &timing, chips, config->sdram_count);

	LOG_DBG("EMC up: %u chip(s), EMC clock %u Hz", config->sdram_count, emc_freq);

	return 0;
}

#define MEMC_EMC_SDRAM_COUNT(inst) DT_INST_CHILD_NUM_STATUS_OKAY(inst)

#define MEMC_EMC_INIT(inst)                                                                        \
	BUILD_ASSERT(MEMC_EMC_SDRAM_COUNT(inst) >= 1,                                              \
		     "nxp,lpc-emc requires at least one nxp,lpc-emc-sdram child");                 \
	BUILD_ASSERT(MEMC_EMC_SDRAM_COUNT(inst) <= CONFIG_MEMC_MCUX_EMC_MAX_SDRAM_CHIPS,           \
		     "more SDRAM chips than CONFIG_MEMC_MCUX_EMC_MAX_SDRAM_CHIPS");                \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static const struct emc_sdram_desc memc_emc_sdram_##inst[] = {                             \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, EMC_SDRAM_DESC)};                          \
	static const struct memc_emc_config memc_emc_config_##inst = {                             \
		.base = (EMC_Type *)DT_INST_REG_ADDR(inst),                                        \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, name),           \
		.basic =                                                                           \
			{                                                                          \
				.endian = DT_INST_PROP(inst, nxp_emc_big_endian)                   \
						  ? kEMC_BigEndian                                 \
						  : kEMC_LittleEndian,                             \
				.fbClkSrc = DT_INST_PROP(inst, nxp_emc_external_feedback_clock)    \
						    ? kEMC_EMCFbclkInput                           \
						    : kEMC_IntloopbackEmcclk,                      \
				.emcClkDiv = DT_INST_PROP(inst, nxp_emc_clock_div),                \
			},                                                                         \
		.sdram = memc_emc_sdram_##inst,                                                    \
		.sdram_count = ARRAY_SIZE(memc_emc_sdram_##inst),                                  \
		.fbclk_delay = DT_INST_PROP(inst, nxp_emc_feedback_clock_delay),                   \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, memc_emc_init, NULL, NULL, &memc_emc_config_##inst,            \
			      POST_KERNEL, CONFIG_MEMC_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MEMC_EMC_INIT)
