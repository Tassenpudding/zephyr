/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr flash-API driver for a serial NOR flash attached to the NXP LPC
 * SPIFI controller. The flash command sequencing and SFDP-based geometry
 * discovery are provided by the NXP MCUXpresso SDK NOR-flash component
 * (fsl_spifi_nor_flash.c, Nor_Flash_* API); this driver is the Zephyr glue
 * around it: clock/reset/pinctrl bring-up, the SPIFI clock divider callback,
 * and the mapping of the flash API onto the Nor_Flash_* calls.
 */

#define DT_DRV_COMPAT nxp_lpc_spifi_nor

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <fsl_spifi.h>
#include <fsl_clock.h>
#include <fsl_reset.h>
#include "fsl_nor_flash.h"
#include "fsl_spifi_nor_flash.h"

LOG_MODULE_REGISTER(flash_mcux_spifi, CONFIG_FLASH_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 1,
	     "Only one SPIFI NOR flash instance is supported");

/* The NOR component always programs a whole page; a bounce buffer of this size
 * lets the driver accept arbitrary (unaligned, sub-page) writes. SPIFI serial
 * NOR page size is 256 bytes; guarded at runtime against the SFDP value.
 */
#define SPIFI_NOR_MAX_PAGE_SIZE 256U

/* SCK frequency used while probing SFDP, before the run frequency is known. */
#define SPIFI_NOR_SFDP_FREQ_HZ 12000000U

struct flash_mcux_spifi_config {
	SPIFI_Type *base;
	uintptr_t mem_base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pcfg;
	uint32_t max_frequency;
};

struct flash_mcux_spifi_data {
	struct k_sem lock;
	nor_handle_t nor;
	uint8_t page_buf[SPIFI_NOR_MAX_PAGE_SIZE];
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout layout;
#endif
};

/* The NOR component's clock callback carries no context and there is only ever
 * one instance, so the source rate and target run rate are held at file scope.
 */
static uint32_t spifi_src_hz;
static uint32_t spifi_run_hz;

static const struct flash_parameters flash_mcux_spifi_parameters = {
	.write_block_size = 1U,
	.erase_value = 0xff,
};

static uint32_t spifi_div_for(uint32_t target_hz)
{
	uint32_t div = DIV_ROUND_UP(spifi_src_hz, target_hz);

	return (div == 0U) ? 1U : div;
}

/* Invoked by Nor_Flash_Init: first with kSpifiNorClockInit_Sdfp (slow, for the
 * SFDP read), then with kSpifiNorClockInit_Max (the run frequency).
 */
static void flash_mcux_spifi_clock_config(spifi_nor_clock_init_t param)
{
	uint32_t target =
		(param == kSpifiNorClockInit_Sdfp) ? SPIFI_NOR_SFDP_FREQ_HZ : spifi_run_hz;

	CLOCK_SetClkDiv(kCLOCK_DivSpifiClk, spifi_div_for(target), false);
}

static int flash_mcux_spifi_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	const struct flash_mcux_spifi_config *cfg = dev->config;
	struct flash_mcux_spifi_data *dd = dev->data;
	int ret = 0;

	if (len == 0U) {
		return 0;
	}
	if ((offset < 0) || ((uint32_t)offset + len > dd->nor.bytesInMemorySize)) {
		return -EINVAL;
	}

	k_sem_take(&dd->lock, K_FOREVER);
	/* The read path uses the SPIFI memory-mapped window, so the absolute
	 * mapped address is passed rather than the flash offset.
	 */
	if (Nor_Flash_Read(&dd->nor, cfg->mem_base + offset, data, len) != kStatus_Success) {
		ret = -EIO;
	}
	k_sem_give(&dd->lock);

	return ret;
}

static int flash_mcux_spifi_write(const struct device *dev, off_t offset, const void *data,
				  size_t len)
{
	struct flash_mcux_spifi_data *dd = dev->data;
	const uint8_t *src = data;
	uint32_t page = dd->nor.bytesInPageSize;
	int ret = 0;

	if (len == 0U) {
		return 0;
	}
	if ((offset < 0) || ((uint32_t)offset + len > dd->nor.bytesInMemorySize)) {
		return -EINVAL;
	}

	k_sem_take(&dd->lock, K_FOREVER);

	while (len > 0U) {
		uint32_t page_off = (uint32_t)offset & ~(page - 1U);
		uint32_t in_page = (uint32_t)offset - page_off;
		uint32_t chunk = MIN(page - in_page, len);

		/* The NOR component programs a full page from the buffer.
		 * Programming 0xff leaves NOR bits unchanged, so padding the
		 * untouched bytes preserves the rest of the page.
		 */
		memset(dd->page_buf, 0xff, page);
		memcpy(dd->page_buf + in_page, src, chunk);

		if (Nor_Flash_Page_Program(&dd->nor, page_off, dd->page_buf) != kStatus_Success) {
			ret = -EIO;
			break;
		}

		offset += chunk;
		src += chunk;
		len -= chunk;
	}

	k_sem_give(&dd->lock);

	return ret;
}

static int flash_mcux_spifi_erase(const struct device *dev, off_t offset, size_t size)
{
	struct flash_mcux_spifi_data *dd = dev->data;
	uint32_t sector = dd->nor.bytesInSectorSize;
	int ret = 0;

	if (size == 0U) {
		return 0;
	}
	if ((offset < 0) || ((uint32_t)offset + size > dd->nor.bytesInMemorySize)) {
		return -EINVAL;
	}
	if (((uint32_t)offset % sector != 0U) || (size % sector != 0U)) {
		return -EINVAL;
	}

	k_sem_take(&dd->lock, K_FOREVER);
	if (Nor_Flash_Erase(&dd->nor, (uint32_t)offset, size) != kStatus_Success) {
		ret = -EIO;
	}
	k_sem_give(&dd->lock);

	return ret;
}

static const struct flash_parameters *flash_mcux_spifi_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_mcux_spifi_parameters;
}

static int flash_mcux_spifi_get_size(const struct device *dev, uint64_t *size)
{
	struct flash_mcux_spifi_data *dd = dev->data;

	*size = dd->nor.bytesInMemorySize;

	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_mcux_spifi_pages_layout(const struct device *dev,
					  const struct flash_pages_layout **layout,
					  size_t *layout_size)
{
	struct flash_mcux_spifi_data *dd = dev->data;

	*layout = &dd->layout;
	*layout_size = 1U;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static int flash_mcux_spifi_init(const struct device *dev)
{
	const struct flash_mcux_spifi_config *cfg = dev->config;
	struct flash_mcux_spifi_data *dd = dev->data;
	int ret;

	/* The NOR component keeps a single static handle, so the config blocks
	 * only need to live for the duration of Nor_Flash_Init.
	 */
	spifi_mem_nor_config_t nor_mem_cfg = {
		.clockInit = flash_mcux_spifi_clock_config,
		.cmd_format = kSPIFI_CommandAllSerial,
		.quad_mode_setting = kSerialNorQuadMode_NotConfig,
	};
	nor_config_t nor_cfg = {
		.memControlConfig = &nor_mem_cfg,
		.driverBaseAddr = cfg->base,
	};

	k_sem_init(&dd->lock, 1, 1);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Source SPIFI from the main clock and sample the resulting rate (with a
	 * temporary /1) so the clock callback can derive the SFDP and run
	 * dividers for the requested target frequencies.
	 */
	CLOCK_AttachClk(kMAIN_CLK_to_SPIFI_CLK);
	CLOCK_SetClkDiv(kCLOCK_DivSpifiClk, 1U, false);
	spifi_src_hz = CLOCK_GetSpifiClkFreq();
	spifi_run_hz = cfg->max_frequency;

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret) {
		return ret;
	}
	RESET_PeripheralReset(kSPIFI_RST_SHIFT_RSTn);

	if (Nor_Flash_Init(&nor_cfg, &dd->nor) != kStatus_Success) {
		LOG_ERR("SPIFI NOR initialization failed");
		return -EIO;
	}

	if (dd->nor.bytesInPageSize > SPIFI_NOR_MAX_PAGE_SIZE) {
		LOG_ERR("flash page size %u exceeds driver maximum %u", dd->nor.bytesInPageSize,
			SPIFI_NOR_MAX_PAGE_SIZE);
		return -ENOTSUP;
	}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	dd->layout.pages_size = dd->nor.bytesInSectorSize;
	dd->layout.pages_count = dd->nor.bytesInMemorySize / dd->nor.bytesInSectorSize;
#endif

	LOG_INF("SPIFI NOR: %u KiB, page %u B, sector %u B, SPIFI src %u Hz",
		dd->nor.bytesInMemorySize / 1024U, dd->nor.bytesInPageSize,
		dd->nor.bytesInSectorSize, spifi_src_hz);

	return 0;
}

static DEVICE_API(flash, flash_mcux_spifi_api) = {
	.read = flash_mcux_spifi_read,
	.write = flash_mcux_spifi_write,
	.erase = flash_mcux_spifi_erase,
	.get_parameters = flash_mcux_spifi_get_parameters,
	.get_size = flash_mcux_spifi_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_mcux_spifi_pages_layout,
#endif
};

PINCTRL_DT_INST_DEFINE(0);

static const struct flash_mcux_spifi_config flash_mcux_spifi_config_0 = {
	.base = (SPIFI_Type *)DT_INST_REG_ADDR_BY_NAME(0, control),
	.mem_base = DT_INST_REG_ADDR_BY_NAME(0, memory),
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, name),
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
	.max_frequency = DT_INST_PROP(0, spi_max_frequency),
};

static struct flash_mcux_spifi_data flash_mcux_spifi_data_0;

DEVICE_DT_INST_DEFINE(0, flash_mcux_spifi_init, NULL, &flash_mcux_spifi_data_0,
		      &flash_mcux_spifi_config_0, POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		      &flash_mcux_spifi_api);
