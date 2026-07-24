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

#define LOG_MODULE_NAME dwmac_plat
#define LOG_LEVEL       CONFIG_ETHERNET_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define DT_DRV_COMPAT nxp_lpc546xx_ethernet

#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>

#include <fsl_device_registers.h>
#include <fsl_reset.h>

#include "eth_dwmac_priv.h"

/* The DMA bus master interface is 32-bit on this IP. */
#define DATA_BUS_WIDTH 32

/* MAC_RXQ_CTRL0: route RX queue 0 to the DCB/generic path (RXQ0EN = 0b10). */
#define MAC_RXQ_CTRL0_RXQ0EN_DCB (0x2U << 0)

/* MTL_TXQn_OPERATION_MODE fields. */
#define MTL_TXQ_OP_MODE_TSF      BIT(1)          /* transmit store-and-forward */
#define MTL_TXQ_OP_MODE_TXQEN_EN (0x2U << 2)     /* TX queue enabled */
#define MTL_TXQ_OP_MODE_TQS      GENMASK(24, 16) /* TX queue size (256-byte units - 1) */

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
	 * Select the RMII PHY interface. Per the reference manual this must be
	 * set before the ENET DMA is taken out of reset.
	 */
	SYSCON->ETHPHYSEL =
		(SYSCON->ETHPHYSEL & ~SYSCON_ETHPHYSEL_PHY_SEL_MASK) | SYSCON_ETHPHYSEL_PHY_SEL(1U);

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

/*
 * Program a stable, locally-administered unicast MAC address derived from the
 * die unique ID. The core does not manage the PHY over MDIO, so the address is
 * set here directly; a per-chip value keeps boards on the same segment distinct.
 */
static void lpc_eth_set_mac(uint8_t mac[6])
{
	uint8_t uid[8];
	ssize_t len = hwinfo_get_device_id(uid, sizeof(uid));

	mac[0] = 0x02; /* locally administered, unicast */
	mac[1] = 0x60;
	mac[2] = 0x37;

	if (len >= 3) {
		mac[3] = uid[len - 3];
		mac[4] = uid[len - 2];
		mac[5] = uid[len - 1];
	} else {
		mac[3] = 0x00;
		mac[4] = 0x00;
		mac[5] = 0x01;
	}
}

int dwmac_platform_init(const struct device *dev)
{
	struct dwmac_priv *p = dev->data;
	uint32_t tx_fifo, tqs;

	p->tx_descs = dwmac_tx_descs;
	p->rx_descs = dwmac_rx_descs;

	/* RMII, 100 Mbit/s, full duplex. */
	DWMAC_REG_WRITE(MAC_CONF, MAC_CONF_PS | MAC_CONF_FES | MAC_CONF_DM);
	DWMAC_REG_WRITE(DMA_SYSBUS_MODE, DMA_SYSBUS_MODE_AAL | DMA_SYSBUS_MODE_FB);

	/*
	 * Enable MAC receive queue 0 for generic (DCB) traffic. It is disabled
	 * out of reset on the LPC546xx, and the core does not set it, so without
	 * this the MAC drops every received frame before the DMA.
	 */
	DWMAC_REG_WRITE(MAC_RXQ_CTRL0, MAC_RXQ_CTRL0_RXQ0EN_DCB);

	/*
	 * Enable MTL transmit queue 0. Like the MAC RX queue it is disabled out
	 * of reset on the LPC546xx and the core does not configure it, so frames
	 * would sit in the MTL and never reach the MAC transmitter. Give it the
	 * whole TX FIFO and use store-and-forward.
	 */
	tx_fifo = 128U << FIELD_GET(MAC_HW_FEATURE1_TXFIFOSIZE, p->feature1);
	tqs = (tx_fifo / 256U) - 1U;
	DWMAC_REG_WRITE(MTL_TXQn_OPERATION_MODE(0), FIELD_PREP(MTL_TXQ_OP_MODE_TQS, tqs) |
							    MTL_TXQ_OP_MODE_TXQEN_EN |
							    MTL_TXQ_OP_MODE_TSF);

	/* Set up the ENET IRQ (kept masked by the core until the iface is up). */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), dwmac_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	lpc_eth_set_mac(p->mac_addr);

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
