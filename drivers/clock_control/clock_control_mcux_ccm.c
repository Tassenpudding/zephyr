/*
 * Copyright 2017, 2024-2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_ccm
#include <errno.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/imx_ccm.h>
#include <fsl_clock.h>

#if defined(CONFIG_SOC_MIMX8QM6_ADSP) || defined(CONFIG_SOC_MIMX8QX6_ADSP)
#include <main/ipc.h>
#endif

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control);

#if defined(CONFIG_SOC_MIMX8QM6_ADSP) || defined(CONFIG_SOC_MIMX8QX6_ADSP)
#define AUD_PLL_DIV_CLK0_LPCG UINT_TO_POINTER(0x59D20000)
static sc_ipc_t ipc_handle;
#endif

#ifdef CONFIG_SPI_NXP_LPSPI
static const clock_name_t lpspi_clocks[] = {
	kCLOCK_Usb1PllPfd1Clk,
	kCLOCK_Usb1PllPfd0Clk,
	kCLOCK_SysPllClk,
	kCLOCK_SysPllPfd2Clk,
};
#endif
#ifdef CONFIG_UART_MCUX_IUART
static const clock_root_control_t uart_clk_root[] = {
	kCLOCK_RootUart1,
	kCLOCK_RootUart2,
	kCLOCK_RootUart3,
	kCLOCK_RootUart4,
};

static const clock_ip_name_t uart_clocks[] = {
	kCLOCK_Uart1,
	kCLOCK_Uart2,
	kCLOCK_Uart3,
	kCLOCK_Uart4,
};
#endif

#ifdef CONFIG_UART_MCUX_LPUART

#ifdef CONFIG_SOC_MIMX8QM6_ADSP
static const clock_ip_name_t lpuart_clocks[] = {
	kCLOCK_DMA_Lpuart0,
	kCLOCK_DMA_Lpuart1,
	kCLOCK_DMA_Lpuart2,
	kCLOCK_DMA_Lpuart3,
	kCLOCK_DMA_Lpuart4,
};

static const uint32_t lpuart_rate = MHZ(80);
#endif /* CONFIG_SOC_MIMX8QM6_ADSP */

#ifdef CONFIG_SOC_MIMX8QX6_ADSP
static const clock_ip_name_t lpuart_clocks[] = {
	kCLOCK_DMA_Lpuart0,
	kCLOCK_DMA_Lpuart1,
	kCLOCK_DMA_Lpuart2,
	kCLOCK_DMA_Lpuart3,
};

static const uint32_t lpuart_rate = MHZ(80);
#endif /* CONFIG_SOC_MIMX8QX6_ADSP */

#endif /* CONFIG_UART_MCUX_LPUART */

#ifdef CONFIG_DAI_NXP_SAI
#if defined(CONFIG_SOC_MIMX8QX6_ADSP) || defined(CONFIG_SOC_MIMX8QM6_ADSP)
static const clock_ip_name_t sai_clocks[] = {
	kCLOCK_AUDIO_Sai1,
	kCLOCK_AUDIO_Sai2,
	kCLOCK_AUDIO_Sai3,
};
#endif
#endif /* CONFIG_DAI_NXP_SAI */

#ifdef CONFIG_DAI_NXP_ESAI
#if defined(CONFIG_SOC_MIMX8QX6_ADSP) || defined(CONFIG_SOC_MIMX8QM6_ADSP)
static const clock_ip_name_t esai_clocks[] = {
	kCLOCK_AUDIO_Esai0,
	kCLOCK_AUDIO_Esai1,
};
#endif
#endif /* CONFIG_DAI_NXP_ESAI */

#if defined(CONFIG_I2C_NXP_II2C)
static const clock_ip_name_t i2c_clk_root[] = {
	kCLOCK_RootI2c1,
	kCLOCK_RootI2c2,
	kCLOCK_RootI2c3,
	kCLOCK_RootI2c4,
#ifdef CONFIG_SOC_MIMX8ML8
	kCLOCK_RootI2c5,
	kCLOCK_RootI2c6,
#endif
};
#endif

#if defined(CONFIG_CAN_MCUX_FLEXCAN) && defined(CONFIG_SOC_MIMX8ML8)
static const clock_ip_name_t flexcan_clk_root[] = {
	kCLOCK_RootFlexCan1,
	kCLOCK_RootFlexCan2,
};
#endif

static int mcux_ccm_on(const struct device *dev,
			      clock_control_subsys_t sub_system)
{
	uint32_t clock_name = (uintptr_t)sub_system;
	uint32_t instance = clock_name & IMX_CCM_INSTANCE_MASK;

	switch (clock_name) {
#ifdef CONFIG_UART_MCUX_IUART
	case IMX_CCM_UART1_CLK:
	case IMX_CCM_UART2_CLK:
	case IMX_CCM_UART3_CLK:
	case IMX_CCM_UART4_CLK:
		CLOCK_EnableClock(uart_clocks[instance]);
		return 0;
#endif

#if defined(CONFIG_UART_MCUX_LPUART) && defined(CONFIG_SOC_MIMX8QM6_ADSP)
	case IMX_CCM_LPUART1_CLK:
	case IMX_CCM_LPUART2_CLK:
	case IMX_CCM_LPUART3_CLK:
	case IMX_CCM_LPUART4_CLK:
	case IMX_CCM_LPUART5_CLK:
		CLOCK_EnableClock(lpuart_clocks[instance]);
		return 0;
#endif

#if defined(CONFIG_UART_MCUX_LPUART) && defined(CONFIG_SOC_MIMX8QX6_ADSP)
	case IMX_CCM_LPUART1_CLK:
	case IMX_CCM_LPUART2_CLK:
	case IMX_CCM_LPUART3_CLK:
	case IMX_CCM_LPUART4_CLK:
		CLOCK_EnableClock(lpuart_clocks[instance]);
		return 0;
#endif

#ifdef CONFIG_DAI_NXP_SAI
#if defined(CONFIG_SOC_MIMX8QM6_ADSP) || defined(CONFIG_SOC_MIMX8QX6_ADSP)
	case IMX_CCM_SAI1_CLK:
	case IMX_CCM_SAI2_CLK:
	case IMX_CCM_SAI3_CLK:
		CLOCK_EnableClock(sai_clocks[instance]);
		return 0;
#endif
#endif /* CONFIG_DAI_NXP_SAI */

#ifdef CONFIG_DAI_NXP_ESAI
#if defined(CONFIG_SOC_MIMX8QM6_ADSP) || defined(CONFIG_SOC_MIMX8QX6_ADSP)
	case IMX_CCM_ESAI0_CLK:
	case IMX_CCM_ESAI1_CLK:
		CLOCK_EnableClock(esai_clocks[instance]);
		return 0;
#endif
#endif /* CONFIG_DAI_NXP_ESAI */

#if defined(CONFIG_SOC_MIMX8QM6_ADSP) || defined(CONFIG_SOC_MIMX8QX6_ADSP)
	case IMX_CCM_AUD_PLL_DIV_CLK0:
		/* ungate PLL parent */
		sc_pm_clock_enable(ipc_handle, SC_R_AUDIO_PLL_0,
				   SC_PM_CLK_MISC0, true, false);

		/* ungate the clock itself */
		CLOCK_SetLpcgGate(AUD_PLL_DIV_CLK0_LPCG, true, false, 0xa);

		return 0;
#endif

#if defined(CONFIG_ETH_NXP_ENET)
#ifdef CONFIG_SOC_SERIES_IMX8M
#define ENET_CLOCK	kCLOCK_Enet1
#else
#define ENET_CLOCK	kCLOCK_Enet
#endif
	case IMX_CCM_ENET_CLK:
		CLOCK_EnableClock(ENET_CLOCK);
		return 0;
#endif
	default:
		(void)instance;
		return 0;
	}
}

static int mcux_ccm_off(const struct device *dev,
			       clock_control_subsys_t sub_system)
{
	uint32_t clock_name = (uintptr_t)sub_system;
	uint32_t instance = clock_name & IMX_CCM_INSTANCE_MASK;

	switch (clock_name) {
#ifdef CONFIG_UART_MCUX_IUART
	case IMX_CCM_UART1_CLK:
	case IMX_CCM_UART2_CLK:
	case IMX_CCM_UART3_CLK:
	case IMX_CCM_UART4_CLK:
		CLOCK_DisableClock(uart_clocks[instance]);
		return 0;
#endif

#ifdef CONFIG_DAI_NXP_SAI
#if defined(CONFIG_SOC_MIMX8QM6_ADSP) || defined(CONFIG_SOC_MIMX8QX6_ADSP)
	case IMX_CCM_SAI1_CLK:
	case IMX_CCM_SAI2_CLK:
	case IMX_CCM_SAI3_CLK:
		CLOCK_DisableClock(sai_clocks[instance]);
		return 0;
#endif
#endif /* CONFIG_DAI_NXP_SAI */

#ifdef CONFIG_DAI_NXP_ESAI
#if defined(CONFIG_SOC_MIMX8QM6_ADSP) || defined(CONFIG_SOC_MIMX8QX6_ADSP)
	case IMX_CCM_ESAI0_CLK:
	case IMX_CCM_ESAI1_CLK:
		CLOCK_DisableClock(esai_clocks[instance]);
		return 0;
#endif
#endif /* CONFIG_DAI_NXP_ESAI */

#if defined(CONFIG_SOC_MIMX8QM6_ADSP) || defined(CONFIG_SOC_MIMX8QX6_ADSP)
	case IMX_CCM_AUD_PLL_DIV_CLK0:
		/* gate the clock itself */
		CLOCK_SetLpcgGate(AUD_PLL_DIV_CLK0_LPCG, false, false, 0xa);

		/* gate PLL parent */
		sc_pm_clock_enable(ipc_handle, SC_R_AUDIO_PLL_0,
				   SC_PM_CLK_MISC0, false, false);

		return 0;
#endif
	default:
		(void)instance;
		return 0;
	}
}

static int mcux_ccm_get_subsys_rate(const struct device *dev,
				    clock_control_subsys_t sub_system,
				    uint32_t *rate)
{
	uint32_t clock_name = (uintptr_t)sub_system;

	switch (clock_name) {

#ifdef CONFIG_I2C_MCUX_LPI2C
	case IMX_CCM_LPI2C_CLK:
		if (CLOCK_GetMux(kCLOCK_Lpi2cMux) == 0) {
			*rate = CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 8
				/ (CLOCK_GetDiv(kCLOCK_Lpi2cDiv) + 1);
		} else {
			*rate = CLOCK_GetOscFreq()
				/ (CLOCK_GetDiv(kCLOCK_Lpi2cDiv) + 1);
		}

		break;
#endif

#ifdef CONFIG_SPI_NXP_LPSPI
	case IMX_CCM_LPSPI_CLK:
	{
		uint32_t lpspi_mux = CLOCK_GetMux(kCLOCK_LpspiMux);
		clock_name_t lpspi_clock = lpspi_clocks[lpspi_mux];

		*rate = CLOCK_GetFreq(lpspi_clock)
			/ (CLOCK_GetDiv(kCLOCK_LpspiDiv) + 1);
		break;
	}
#endif

#ifdef CONFIG_UART_MCUX_LPUART

#if defined(CONFIG_SOC_MIMX8QM6_ADSP)
	case IMX_CCM_LPUART1_CLK:
	case IMX_CCM_LPUART2_CLK:
	case IMX_CCM_LPUART3_CLK:
	case IMX_CCM_LPUART4_CLK:
	case IMX_CCM_LPUART5_CLK:
		uint32_t instance = clock_name & IMX_CCM_INSTANCE_MASK;

		CLOCK_SetIpFreq(lpuart_clocks[instance], lpuart_rate);
		*rate = CLOCK_GetIpFreq(lpuart_clocks[instance]);
		break;

#elif defined(CONFIG_SOC_MIMX8QX6_ADSP)
	case IMX_CCM_LPUART1_CLK:
	case IMX_CCM_LPUART2_CLK:
	case IMX_CCM_LPUART3_CLK:
	case IMX_CCM_LPUART4_CLK:
		uint32_t instance = clock_name & IMX_CCM_INSTANCE_MASK;

		CLOCK_SetIpFreq(lpuart_clocks[instance], lpuart_rate);
		*rate = CLOCK_GetIpFreq(lpuart_clocks[instance]);
		break;

#else
	case IMX_CCM_LPUART_CLK:
		if (CLOCK_GetMux(kCLOCK_UartMux) == 0) {
			*rate = CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6
				/ (CLOCK_GetDiv(kCLOCK_UartDiv) + 1);
		} else {
			*rate = CLOCK_GetOscFreq()
				/ (CLOCK_GetDiv(kCLOCK_UartDiv) + 1);
		}

		break;
#endif
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(usdhc1)) && CONFIG_IMX_USDHC
	case IMX_CCM_USDHC1_CLK:
		*rate = CLOCK_GetSysPfdFreq(kCLOCK_Pfd0) /
				(CLOCK_GetDiv(kCLOCK_Usdhc1Div) + 1U);
		break;
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(usdhc2)) && CONFIG_IMX_USDHC
	case IMX_CCM_USDHC2_CLK:
		*rate = CLOCK_GetSysPfdFreq(kCLOCK_Pfd0) /
				(CLOCK_GetDiv(kCLOCK_Usdhc2Div) + 1U);
		break;
#endif

#ifdef CONFIG_DMA_MCUX_EDMA
	case IMX_CCM_EDMA_CLK:
		*rate = CLOCK_GetIpgFreq();
		break;
#endif

#ifdef CONFIG_PWM_MCUX
	case IMX_CCM_PWM_CLK:
		*rate = CLOCK_GetIpgFreq();
		break;
#endif

#ifdef CONFIG_ETH_NXP_ENET
	case IMX_CCM_ENET_CLK:
#ifdef CONFIG_SOC_SERIES_IMX8M
		*rate = CLOCK_GetFreq(kCLOCK_EnetIpgClk);
#else
		*rate = CLOCK_GetIpgFreq();
#endif
#endif
		break;

#ifdef CONFIG_PTP_CLOCK_NXP_ENET
	case IMX_CCM_ENET_PLL:
#if defined(CONFIG_SOC_SERIES_IMXRT10XX)
		*rate = CLOCK_GetPllFreq(kCLOCK_PllEnet25M);
#else
		*rate = CLOCK_GetPllFreq(kCLOCK_PllEnet);
#endif
		break;
#endif

#ifdef CONFIG_UART_MCUX_IUART
	case IMX_CCM_UART1_CLK:
	case IMX_CCM_UART2_CLK:
	case IMX_CCM_UART3_CLK:
	case IMX_CCM_UART4_CLK:
	{
		uint32_t instance = clock_name & IMX_CCM_INSTANCE_MASK;
		clock_root_control_t clk_root = uart_clk_root[instance];
		uint32_t uart_mux = CLOCK_GetRootMux(clk_root);

		if (uart_mux == 0) {
			*rate = MHZ(24);
		} else if (uart_mux == 1) {
			*rate = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) /
				(CLOCK_GetRootPreDivider(clk_root)) /
				(CLOCK_GetRootPostDivider(clk_root)) /
				10;
		}

	} break;
#endif

#ifdef CONFIG_CAN_MCUX_FLEXCAN
#ifdef CONFIG_SOC_MIMX8ML8
	case IMX_CCM_CAN1_CLK:
	case IMX_CCM_CAN2_CLK:
	{
		uint32_t instance = clock_name & IMX_CCM_INSTANCE_MASK;
		uint32_t can_mux = CLOCK_GetRootMux(flexcan_clk_root[instance]);

		if (can_mux == 0) {
			*rate = MHZ(24);
		} else if (can_mux == 4) { /* SYSTEM_PLL1_CLK */
			*rate = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) /
				(CLOCK_GetRootPreDivider(flexcan_clk_root[instance])) /
				(CLOCK_GetRootPostDivider(flexcan_clk_root[instance]));
		}
	} break;
#else
	case IMX_CCM_CAN_CLK:
	{
		uint32_t can_mux = CLOCK_GetMux(kCLOCK_CanMux);

		if (can_mux == 0) {
			*rate = CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 8
				/ (CLOCK_GetDiv(kCLOCK_CanDiv) + 1);
		} else if  (can_mux == 1) {
			*rate = CLOCK_GetOscFreq()
				/ (CLOCK_GetDiv(kCLOCK_CanDiv) + 1);
		} else {
			*rate = CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6
				/ (CLOCK_GetDiv(kCLOCK_CanDiv) + 1);
		}
	} break;
#endif
#endif

#ifdef CONFIG_COUNTER_MCUX_GPT
	case IMX_CCM_GPT_CLK:
		*rate = CLOCK_GetFreq(kCLOCK_PerClk);
		break;
#ifdef CONFIG_SOC_SERIES_IMX8M
	case IMX_CCM_GPT_IPG_CLK:
	{
		uint32_t mux = CLOCK_GetRootMux(kCLOCK_RootGpt1);

		if (mux == 0) {
			*rate = OSC24M_CLK_FREQ;
		} else {
			*rate = 0;
		}
	} break;
#endif
#endif

#ifdef CONFIG_COUNTER_MCUX_QTMR
	case IMX_CCM_QTMR_CLK:
		*rate = CLOCK_GetIpgFreq();
		break;
#endif

#ifdef CONFIG_I2S_MCUX_SAI
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(sai1))
	case IMX_CCM_SAI1_CLK:
		*rate = CLOCK_GetFreq(kCLOCK_AudioPllClk)
				/ (CLOCK_GetDiv(kCLOCK_Sai1PreDiv) + 1)
				/ (CLOCK_GetDiv(kCLOCK_Sai1Div) + 1);
		break;
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(sai2))
	case IMX_CCM_SAI2_CLK:
		*rate = CLOCK_GetFreq(kCLOCK_AudioPllClk)
				/ (CLOCK_GetDiv(kCLOCK_Sai2PreDiv) + 1)
				/ (CLOCK_GetDiv(kCLOCK_Sai2Div) + 1);
		break;
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(sai3))
	case IMX_CCM_SAI3_CLK:
		*rate = CLOCK_GetFreq(kCLOCK_AudioPllClk)
				/ (CLOCK_GetDiv(kCLOCK_Sai3PreDiv) + 1)
				/ (CLOCK_GetDiv(kCLOCK_Sai3Div) + 1);
		break;
#endif
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(flexspi))
	case IMX_CCM_FLEXSPI_CLK:
		*rate = CLOCK_GetClockRootFreq(kCLOCK_FlexspiClkRoot);
		break;
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(flexspi2))
	case IMX_CCM_FLEXSPI2_CLK:
		*rate = CLOCK_GetClockRootFreq(kCLOCK_Flexspi2ClkRoot);
		break;
#endif
#ifdef CONFIG_COUNTER_NXP_PIT
	case IMX_CCM_PIT_CLK:
		*rate = CLOCK_GetFreq(kCLOCK_PerClk);
		break;
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(flexio1)) && CONFIG_MCUX_FLEXIO
	case IMX_CCM_FLEXIO1_CLK:
	{
		uint32_t flexio_mux = CLOCK_GetMux(kCLOCK_Flexio1Mux);
		uint32_t source_clk_freq = 0;

		if (flexio_mux == 0) {
			source_clk_freq = CLOCK_GetPllFreq(kCLOCK_PllAudio);
		} else if (flexio_mux == 1) {
			source_clk_freq = CLOCK_GetUsb1PfdFreq(kCLOCK_Pfd2);
	#ifdef PLL_VIDEO_OFFSET /* fsl_clock.h */
		} else if (flexio_mux == 2) {
			source_clk_freq = CLOCK_GetPllFreq(kCLOCK_PllVideo);
	#endif
		} else {
			source_clk_freq = CLOCK_GetPllFreq(kCLOCK_PllUsb1);
		}

		*rate = source_clk_freq / (CLOCK_GetDiv(kCLOCK_Flexio1PreDiv) + 1)
					/ (CLOCK_GetDiv(kCLOCK_Flexio1Div) + 1);
	} break;
#endif
#if (DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(flexio2)) \
		 || DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(flexio3))) && CONFIG_MCUX_FLEXIO
	case IMX_CCM_FLEXIO2_3_CLK:
	{
		uint32_t flexio_mux = CLOCK_GetMux(kCLOCK_Flexio2Mux);
		uint32_t source_clk_freq = 0;

		if (flexio_mux == 0) {
			source_clk_freq = CLOCK_GetPllFreq(kCLOCK_PllAudio);
		} else if (flexio_mux == 1) {
			source_clk_freq = CLOCK_GetUsb1PfdFreq(kCLOCK_Pfd2);
	#ifdef PLL_VIDEO_OFFSET /* fsl_clock.h */
		} else if (flexio_mux == 2) {
			source_clk_freq = CLOCK_GetPllFreq(kCLOCK_PllVideo);
	#endif
		} else {
			source_clk_freq = CLOCK_GetPllFreq(kCLOCK_PllUsb1);
		}

		*rate = source_clk_freq / (CLOCK_GetDiv(kCLOCK_Flexio2PreDiv) + 1)
					/ (CLOCK_GetDiv(kCLOCK_Flexio2Div) + 1);
	} break;
#endif

#ifdef CONFIG_SPI_MCUX_ECSPI
	case IMX_CCM_ECSPI1_CLK:
		*rate = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) /
			(CLOCK_GetRootPreDivider(kCLOCK_RootEcspi1)) /
			(CLOCK_GetRootPostDivider(kCLOCK_RootEcspi1));
		break;
	case IMX_CCM_ECSPI2_CLK:
		*rate = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) /
			(CLOCK_GetRootPreDivider(kCLOCK_RootEcspi2)) /
			(CLOCK_GetRootPostDivider(kCLOCK_RootEcspi2));
		break;
	case IMX_CCM_ECSPI3_CLK:
		*rate = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) /
			(CLOCK_GetRootPreDivider(kCLOCK_RootEcspi3)) /
			(CLOCK_GetRootPostDivider(kCLOCK_RootEcspi3));
		break;
#endif /* CONFIG_SPI_MCUX_ECSPI */

#if defined(CONFIG_I2C_NXP_II2C)
	case IMX_CCM_I2C1_CLK:
	case IMX_CCM_I2C2_CLK:
	case IMX_CCM_I2C3_CLK:
	case IMX_CCM_I2C4_CLK:
#ifdef CONFIG_SOC_MIMX8ML8
	case IMX_CCM_I2C5_CLK:
	case IMX_CCM_I2C6_CLK:
#endif
	{
		uint32_t instance = clock_name & IMX_CCM_INSTANCE_MASK;
		uint32_t i2c_mux = CLOCK_GetRootMux(i2c_clk_root[instance]);

		if (i2c_mux == 0) {
			*rate = MHZ(24);
		} else if (i2c_mux == 1) {
			*rate = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) /
				(CLOCK_GetRootPreDivider(i2c_clk_root[instance])) /
				(CLOCK_GetRootPostDivider(i2c_clk_root[instance])) /
				5; /* SYSTEM PLL1 DIV5 */
		}

	} break;
#endif
	}

	return 0;
}

/*
 * Since this function is used to reclock the FlexSPI when running in
 * XIP, it must be located in RAM when MEMC Flexspi driver is enabled.
 */
#ifdef CONFIG_MEMC_MCUX_FLEXSPI
#define CCM_SET_FUNC_ATTR __ramfunc
#else
#define CCM_SET_FUNC_ATTR
#endif

static int CCM_SET_FUNC_ATTR mcux_ccm_set_subsys_rate(const struct device *dev,
			clock_control_subsys_t subsys,
			clock_control_subsys_rate_t rate)
{
	uint32_t clock_name = (uintptr_t)subsys;
	uint32_t clock_rate = (uintptr_t)rate;

	switch (clock_name) {
	case IMX_CCM_FLEXSPI_CLK:
		__fallthrough;
	case IMX_CCM_FLEXSPI2_CLK:
#if defined(CONFIG_SOC_SERIES_IMXRT10XX) && defined(CONFIG_MEMC_MCUX_FLEXSPI)
		/* The SOC is using the FlexSPI for XIP. Therefore,
		 * the FlexSPI itself must be managed within the function,
		 * which is SOC specific.
		 */
		return flexspi_clock_set_freq(clock_name, clock_rate);
#endif
	default:
		/* Silence unused variable warning */
		ARG_UNUSED(clock_rate);
		return -ENOTSUP;
	}
}



static DEVICE_API(clock_control, mcux_ccm_driver_api) = {
	.on = mcux_ccm_on,
	.off = mcux_ccm_off,
	.get_rate = mcux_ccm_get_subsys_rate,
	.set_rate = mcux_ccm_set_subsys_rate,
};

static int mcux_ccm_init(const struct device *dev)
{
#if defined(CONFIG_SOC_MIMX8QM6_ADSP) || defined(CONFIG_SOC_MIMX8QX6_ADSP)
	int ret;

	ret = sc_ipc_open(&ipc_handle, DT_REG_ADDR(DT_NODELABEL(scu_mu)));
	if (ret != SC_ERR_NONE) {
		return -ENODEV;
	}

	CLOCK_Init(ipc_handle);
#endif
	return 0;
}

DEVICE_DT_INST_DEFINE(0, mcux_ccm_init, NULL, NULL, NULL,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &mcux_ccm_driver_api);
