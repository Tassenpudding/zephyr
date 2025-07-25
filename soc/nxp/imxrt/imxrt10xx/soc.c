/*
 * Copyright  2017-2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/linker/sections.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/cache.h>
#include <fsl_clock.h>
#ifdef CONFIG_NXP_IMXRT_BOOT_HEADER
#include <fsl_flexspi_nor_boot.h>
#endif
#include <zephyr/dt-bindings/clock/imx_ccm.h>
#include <fsl_iomuxc.h>
#if CONFIG_USB_DC_NXP_EHCI
#include "usb_phy.h"
#include "usb.h"
#endif

#include <zephyr/drivers/misc/flexram/nxp_flexram.h>

#include <cmsis_core.h>

#define CCM_NODE	DT_INST(0, nxp_imx_ccm)

#define BUILD_ASSERT_PODF_IN_RANGE(podf, a, b)				\
	BUILD_ASSERT(DT_PROP(DT_CHILD(CCM_NODE, podf), clock_div) >= (a) && \
		     DT_PROP(DT_CHILD(CCM_NODE, podf), clock_div) <= (b), \
		     #podf " is out of supported range (" #a ", " #b ")")

#if CONFIG_USB_DC_NXP_EHCI
/* USB PHY configuration */
#define BOARD_USB_PHY_D_CAL (0x0CU)
#define BOARD_USB_PHY_TXCAL45DP (0x06U)
#define BOARD_USB_PHY_TXCAL45DM (0x06U)
#endif


#if CONFIG_USB_DC_NXP_EHCI
	usb_phy_config_struct_t usbPhyConfig = {
		BOARD_USB_PHY_D_CAL, BOARD_USB_PHY_TXCAL45DP, BOARD_USB_PHY_TXCAL45DM,
	};
#endif

#ifdef CONFIG_NXP_IMXRT_BOOT_HEADER
const __imx_boot_data_section BOOT_DATA_T boot_data = {
#ifdef CONFIG_XIP
	.start = CONFIG_FLASH_BASE_ADDRESS,
	.size = (uint32_t)&_flash_used,
#else
	.start = CONFIG_SRAM_BASE_ADDRESS,
	.size = (uint32_t)&_image_ram_size,
#endif
	.plugin = PLUGIN_FLAG,
	.placeholder = 0xFFFFFFFF,
};

const __imx_boot_ivt_section ivt image_vector_table = {
	.hdr = IVT_HEADER,
	.entry = (uint32_t) _vector_start,
	.reserved1 = IVT_RSVD,
#ifdef CONFIG_DEVICE_CONFIGURATION_DATA
	.dcd = (uint32_t) dcd_data,
#else
	.dcd = (uint32_t) NULL,
#endif
	.boot_data = (uint32_t) &boot_data,
	.self = (uint32_t) &image_vector_table,
	.csf = (uint32_t)CSF_ADDRESS,
	.reserved2 = IVT_RSVD,
};
#endif

/**
 * @brief Initialize the system clock
 */
__weak void clock_init(void)
{
	/* Boot ROM did initialize the XTAL, here we only sets external XTAL
	 * OSC freq
	 */
	CLOCK_SetXtalFreq(DT_PROP(DT_CLOCKS_CTLR_BY_NAME(CCM_NODE, xtal),
				  clock_frequency));
	CLOCK_SetRtcXtalFreq(DT_PROP(DT_CLOCKS_CTLR_BY_NAME(CCM_NODE, rtc_xtal),
				     clock_frequency));

	/* Set PERIPH_CLK2 MUX to OSC */
	CLOCK_SetMux(kCLOCK_PeriphClk2Mux, 0x1);

	/* Set PERIPH_CLK MUX to PERIPH_CLK2 */
	CLOCK_SetMux(kCLOCK_PeriphMux, 0x1);

	if (IS_ENABLED(CONFIG_ADJUST_DCDC)) {
		/* Setting the VDD_SOC value */
		DCDC->REG3 = (DCDC->REG3 & (~DCDC_REG3_TRG_MASK)) |
				DCDC_REG3_TRG(CONFIG_DCDC_VALUE);
		/* Waiting for DCDC_STS_DC_OK bit is asserted */
		while (DCDC_REG0_STS_DC_OK_MASK != (DCDC_REG0_STS_DC_OK_MASK & DCDC->REG0)) {
			;
		}
	}

#ifdef CONFIG_INIT_ARM_PLL
	/* ARM PLL configuration for RUN mode */
	static const clock_arm_pll_config_t armPllConfig = {
		.loopDivider = 100U
	};
	CLOCK_InitArmPll(&armPllConfig); /* Configure ARM PLL to 1200M */
#endif

	static const clock_enet_pll_config_t ethPllConfig = {
		.enableClkOutput25M = IS_ENABLED(CONFIG_PTP_CLOCK_NXP_ENET),
		.enableClkOutput = DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(enet)),
		.loopDivider = DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(enet)),
#if DT_NODE_EXISTS(DT_NODELABEL(enet2))
		/* some platform don't have enet 2 and sdk doesn't have these fields for it */
		.enableClkOutput1 = DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(enet2)),
		.loopDivider1 = DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(enet2)),
#endif
#if IS_ENABLED(CONFIG_INIT_PLL6_500M)
		/* this field only exists on some platforms, so ifdef is needed */
		.enableClkOutput500M = true,
#endif
	};

	if (IS_ENABLED(CONFIG_INIT_ENET_PLL)) {
		CLOCK_InitEnetPll(&ethPllConfig);
	}

#ifdef CONFIG_INIT_VIDEO_PLL
	static const clock_video_pll_config_t videoPllConfig = {
		.loopDivider = 31,
		.postDivider = 8,
		.numerator = 0,
		.denominator = 0,
	};
	CLOCK_InitVideoPll(&videoPllConfig);
#endif

	const clock_sys_pll_config_t sysPllConfig = {
		.loopDivider = (DT_PROP(DT_CHILD(CCM_NODE, sys_pll), loop_div) - 20) / 2,
		.numerator = DT_PROP(DT_CHILD(CCM_NODE, sys_pll), numerator),
		.denominator = DT_PROP(DT_CHILD(CCM_NODE, sys_pll), denominator),
		.src = DT_PROP(DT_CHILD(CCM_NODE, sys_pll), src),
	};

	if (IS_ENABLED(CONFIG_INIT_SYS_PLL)) {
		CLOCK_InitSysPll(&sysPllConfig);
	}

#if DT_NODE_EXISTS(DT_CHILD(CCM_NODE, arm_podf))
	/* Set ARM PODF */
	BUILD_ASSERT_PODF_IN_RANGE(arm_podf, 1, 8);
	CLOCK_SetDiv(kCLOCK_ArmDiv, DT_PROP(DT_CHILD(CCM_NODE, arm_podf), clock_div) - 1);
#endif
	/* Set AHB PODF */
	BUILD_ASSERT_PODF_IN_RANGE(ahb_podf, 1, 8);
	CLOCK_SetDiv(kCLOCK_AhbDiv, DT_PROP(DT_CHILD(CCM_NODE, ahb_podf), clock_div) - 1);
	/* Set IPG PODF */
	BUILD_ASSERT_PODF_IN_RANGE(ipg_podf, 1, 4);
	CLOCK_SetDiv(kCLOCK_IpgDiv, DT_PROP(DT_CHILD(CCM_NODE, ipg_podf), clock_div) - 1);

#ifdef CONFIG_SOC_MIMXRT1042
	/* Set PRE_PERIPH_CLK to SYS_PLL */
	CLOCK_SetMux(kCLOCK_PrePeriphMux, 0x0);
#else
	/* Set PRE_PERIPH_CLK to PLL1, 1200M */
	CLOCK_SetMux(kCLOCK_PrePeriphMux, 0x3);
#endif

	/* Set PERIPH_CLK MUX to PRE_PERIPH_CLK */
	CLOCK_SetMux(kCLOCK_PeriphMux, 0x0);

#ifdef CONFIG_UART_MCUX_LPUART
	/* Configure UART divider to default */
	CLOCK_SetMux(kCLOCK_UartMux, 0); /* Set UART source to PLL3 80M */
	CLOCK_SetDiv(kCLOCK_UartDiv, 0); /* Set UART divider to 1 */
#endif

#ifdef CONFIG_I2C_MCUX_LPI2C
	CLOCK_SetMux(kCLOCK_Lpi2cMux, 0); /* Set I2C source as USB1 PLL 480M */
	CLOCK_SetDiv(kCLOCK_Lpi2cDiv, 5); /* Set I2C divider to 6 */
#endif

#ifdef CONFIG_SPI_NXP_LPSPI
	/* Configure input clock to be able to reach the datasheet specified band rate. */
	CLOCK_SetMux(kCLOCK_LpspiMux, 1); /* Set SPI source to USB1 PFD0 */
	CLOCK_SetDiv(kCLOCK_LpspiDiv, 0); /* Set SPI divider to 1 */
#endif

#ifdef CONFIG_MCUX_FLEXIO
	/* Configure input clock to be able to reach the datasheet specified baud rate.
	 * FLEXIO can reach to 120MHz. Select USB pll(480M) as source and divide by 2.
	 * pre divider by default is 1 which means divide by 2.
	 */
	CLOCK_SetMux(kCLOCK_Flexio1Mux, 3);
	CLOCK_SetDiv(kCLOCK_Flexio1Div, 1);

	CLOCK_SetMux(kCLOCK_Flexio2Mux, 3);
	CLOCK_SetDiv(kCLOCK_Flexio2Div, 1);
#endif

#ifdef CONFIG_DISPLAY_MCUX_ELCDIF
	/* MUX selects video PLL, which is initialized to 93MHz */
	CLOCK_SetMux(kCLOCK_LcdifPreMux, 2);
	/* Divide output by 2 */
	CLOCK_SetDiv(kCLOCK_LcdifDiv, 1);
	/* Set final div based on LCDIF clock-frequency */
	CLOCK_SetDiv(kCLOCK_LcdifPreDiv,
		((CLOCK_GetPllFreq(kCLOCK_PllVideo) / 2) /
		DT_PROP(DT_CHILD(DT_NODELABEL(lcdif), display_timings),
			clock_frequency)) - 1);
#endif


#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(enet)) && CONFIG_NET_L2_ETHERNET
#if CONFIG_ETH_NXP_ENET_RMII_EXT_CLK
	/* Enable clock input for ENET1 */
	IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, false);
#else
	/* Enable clock output for ENET1 */
	IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true);
#endif
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(enet2)) && CONFIG_NET_L2_ETHERNET
	/* Set ENET2 ref clock to be generated by External OSC,*/
	/* direction as output and frequency to 50MHz */
	IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET2TxClkOutputDir |
				kIOMUXC_GPR_ENET2RefClkMode, true);
#endif

#if ((DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(usb1)) && \
	(CONFIG_USB_DC_NXP_EHCI || CONFIG_UDC_NXP_EHCI)) ||\
	(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(usbh1)) && (CONFIG_UHC_NXP_EHCI)))
	CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usb480M,
		DT_PROP_BY_PHANDLE(DT_NODELABEL(usb1), clocks, clock_frequency));
	CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M,
		DT_PROP_BY_PHANDLE(DT_NODELABEL(usb1), clocks, clock_frequency));
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(usb1)) && CONFIG_USB_DC_NXP_EHCI
	USB_EhciPhyInit(kUSB_ControllerEhci0, CPU_XTAL_CLK_HZ, &usbPhyConfig);
#endif
#endif

#if ((DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(usb2)) && \
	(CONFIG_USB_DC_NXP_EHCI || CONFIG_UDC_NXP_EHCI)) ||\
	(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(usbh2)) && (CONFIG_UHC_NXP_EHCI)))
	CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usb480M,
		DT_PROP_BY_PHANDLE(DT_NODELABEL(usb2), clocks, clock_frequency));
	CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M,
		DT_PROP_BY_PHANDLE(DT_NODELABEL(usb2), clocks, clock_frequency));
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(usb1)) && CONFIG_USB_DC_NXP_EHCI
	USB_EhciPhyInit(kUSB_ControllerEhci1, CPU_XTAL_CLK_HZ, &usbPhyConfig);
#endif
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(usdhc1)) && CONFIG_IMX_USDHC
	/* Configure USDHC clock source and divider */
	CLOCK_InitSysPfd(kCLOCK_Pfd0, 24U);
	CLOCK_SetDiv(kCLOCK_Usdhc1Div, 1U);
	CLOCK_SetMux(kCLOCK_Usdhc1Mux, 1U);
	CLOCK_EnableClock(kCLOCK_Usdhc1);
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(usdhc2)) && CONFIG_IMX_USDHC
	/* Configure USDHC clock source and divider */
	CLOCK_InitSysPfd(kCLOCK_Pfd0, 24U);
	CLOCK_SetDiv(kCLOCK_Usdhc2Div, 1U);
	CLOCK_SetMux(kCLOCK_Usdhc2Mux, 1U);
	CLOCK_EnableClock(kCLOCK_Usdhc2);
#endif

#ifdef CONFIG_VIDEO_MCUX_CSI
	CLOCK_EnableClock(kCLOCK_Csi); /* Disable CSI clock gate */
	CLOCK_SetDiv(kCLOCK_CsiDiv, 0); /* Set CSI divider to 1 */
	CLOCK_SetMux(kCLOCK_CsiMux, 0); /* Set CSI source to OSC 24M */
#endif
#ifdef CONFIG_CAN_MCUX_FLEXCAN
	CLOCK_SetDiv(kCLOCK_CanDiv, 1); /* Set CAN_CLK_PODF. */
	CLOCK_SetMux(kCLOCK_CanMux, 2); /* Set Can clock source. */
#endif

#ifdef CONFIG_LOG_BACKEND_SWO
	/* Enable ARM trace clock to enable SWO output */
	CLOCK_EnableClock(kCLOCK_Trace);
	/* Divide root clock output by 3 */
	CLOCK_SetDiv(kCLOCK_TraceDiv, 3);
	/* Source clock from 528MHz system PLL */
	CLOCK_SetMux(kCLOCK_TraceMux, 0);
#endif

	/* Keep the system clock running so SYSTICK can wake up the system from
	 * wfi.
	 */
	CLOCK_SetMode(kCLOCK_ModeRun);

}

#if CONFIG_I2S_MCUX_SAI
void imxrt_audio_codec_pll_init(uint32_t clock_name, uint32_t clk_src,
					uint32_t clk_pre_div, uint32_t clk_src_div)
{
	switch (clock_name) {
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(sai1))
	case IMX_CCM_SAI1_CLK:
		CLOCK_SetMux(kCLOCK_Sai1Mux, clk_src);
		CLOCK_SetDiv(kCLOCK_Sai1PreDiv, clk_pre_div);
		CLOCK_SetDiv(kCLOCK_Sai1Div, clk_src_div);
		break;
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(sai2))
	case IMX_CCM_SAI2_CLK:
		CLOCK_SetMux(kCLOCK_Sai2Mux, clk_src);
		CLOCK_SetDiv(kCLOCK_Sai2PreDiv, clk_pre_div);
		CLOCK_SetDiv(kCLOCK_Sai2Div, clk_src_div);
		break;
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(sai3))
	case IMX_CCM_SAI3_CLK:
		CLOCK_SetMux(kCLOCK_Sai3Mux, clk_src);
		CLOCK_SetDiv(kCLOCK_Sai3PreDiv, clk_pre_div);
		CLOCK_SetDiv(kCLOCK_Sai3Div, clk_src_div);
		break;
#endif
	default:
		return;
	}
}
#endif

extern void rt10xx_power_init(void);
extern void imxrt_lpm_init(void);

/**
 *
 * @brief Perform basic hardware initialization
 *
 * Initialize the interrupt controller device drivers.
 * Also initialize the timer device driver, if required.
 */
void soc_early_init_hook(void)
{
	sys_cache_instr_enable();
	sys_cache_data_enable();

	/* Initialize system clock */
	clock_init();
#ifdef CONFIG_PM
#ifdef CONFIG_SOC_MIMXRT1064
	imxrt_lpm_init();
#endif
	rt10xx_power_init();
#endif
}

#ifdef CONFIG_SOC_RESET_HOOK
void soc_reset_hook(void)
{
	/* Call CMSIS SystemInit */
	SystemInit();

#if defined(FLEXRAM_RUNTIME_BANKS_USED)
	/* Configure flexram if not running from RAM */
	flexram_dt_partition();
#endif
}
#endif
