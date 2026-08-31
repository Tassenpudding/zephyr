/*
 * Driver for Synopsys DesignWare MAC
 *
 * Copyright (c) 2021 BayLibre SAS
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NXP LPC546xx specific glue. The LPC546xx ENET is a DesignWare EMAC-QoS
 * instance, so it reuses the generic dwc_ether_qos core; only the clock gate,
 * block reset, RMII interface select and pin routing are platform specific.
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dwmac_plat, CONFIG_ETHERNET_LOG_LEVEL);

#define DT_DRV_COMPAT nxp_lpc546xx_ethernet

#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>

#include <fsl_device_registers.h>
#include <fsl_reset.h>

#include "eth_dwmac_priv.h"

/* The DMA bus master interface is 32-bit on this IP. */
#define DATA_BUS_WIDTH 32

#if DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, mii)
#define PHY_SEL_VALUE 0U
#elif DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, rmii)
#define PHY_SEL_VALUE 1U
#else
#error "Unsupported PHY connection type"
#endif

static const uint8_t nxp_oui[3] = DWMAC_NXP_OUI;

DWMAC_ASSERT_BUFFER_ALIGNMENT(DATA_BUS_WIDTH);

PINCTRL_DT_INST_DEFINE(0);
static const struct pinctrl_dev_config *eth0_pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0);

int dwmac_bus_init(const struct device *dev)
{
	const struct dwmac_config *cfg = dev->config;
	int ret;

	/* Enable the ENET AHB clock gate through the syscon clock controller. */
	ret = clock_control_on(cfg->clock, cfg->mac_clk);
	if (ret != 0) {
		LOG_ERR("Failed to enable ethernet clock");
		return ret;
	}

	/*
	 * Select the PHY interface. This is sampled only while the ENET block
	 * reset is asserted, so it has to be set before the reset is pulsed.
	 */
	SYSCON->ETHPHYSEL = (SYSCON->ETHPHYSEL & ~SYSCON_ETHPHYSEL_PHY_SEL_MASK) |
			    SYSCON_ETHPHYSEL_PHY_SEL(PHY_SEL_VALUE);

	/* Pulse the ENET block reset. */
	RESET_PeripheralReset(kETH_RST_SHIFT_RSTn);

	ret = pinctrl_apply_state(eth0_pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Could not configure ethernet pins");
		return ret;
	}

	return 0;
}

#define DESCRIPTOR_ALIGNMENT ((DATA_BUS_WIDTH) / (BITS_PER_BYTE))
#if defined(CONFIG_NOCACHE_MEMORY)
#define __desc_mem __nocache __aligned(DESCRIPTOR_ALIGNMENT)
#else
#define __desc_mem __aligned(DESCRIPTOR_ALIGNMENT)
#endif

/* Descriptor rings (uncached on cored parts; the Cortex-M4 has no data cache). */
static struct dwmac_dma_desc dwmac_tx_descs[NB_TX_DESCS] __desc_mem;
static struct dwmac_dma_desc dwmac_rx_descs[NB_RX_DESCS] __desc_mem;

int dwmac_platform_init(const struct device *dev)
{
	const struct net_eth_mac_config mac_cfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(0);
	struct dwmac_priv *p = dev->data;
	int ret;

	p->tx_descs = dwmac_tx_descs;
	p->rx_descs = dwmac_rx_descs;

	/* Basic configuration for this platform; the PHY updates speed and duplex. */
	DWMAC_REG_WRITE(MAC_CONF, MAC_CONF_PS | MAC_CONF_FES | MAC_CONF_DM);
	DWMAC_REG_WRITE(DMA_SYSBUS_MODE, DMA_SYSBUS_MODE_AAL | DMA_SYSBUS_MODE_FB);

	/* Set up the ENET IRQ (kept masked by the core until the iface is up). */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), dwmac_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	ret = dwmac_mac_addr_load(&mac_cfg, nxp_oui, p->mac_addr);
	if (ret != 0) {
		LOG_ERR("Failed to load a MAC address (%d)", ret);
		return ret;
	}

	return 0;
}

static const struct dwmac_config dwmac_config = {
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),
	.phy_dev = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(0, phy_handle)),
	.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.mac_clk = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, name),
};

static struct dwmac_priv dwmac_instance;

ETH_NET_DEVICE_DT_INST_DEFINE(0, dwmac_probe, NULL, &dwmac_instance, &dwmac_config,
			      CONFIG_ETH_INIT_PRIORITY, &dwmac_api, NET_ETH_MTU);
