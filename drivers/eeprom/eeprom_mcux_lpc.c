/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc_eeprom

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <fsl_eeprom.h>
#include <fsl_power.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eeprom_mcux_lpc, CONFIG_EEPROM_LOG_LEVEL);

/*
 * The controller registers and the storage array are two separate regions:
 * writes go through the controller, reads are ordinary loads from the array.
 * fsl_eeprom.c addresses the array through FSL_FEATURE_EEPROM_BASE_ADDRESS, so
 * the devicetree has to agree with it.
 */
BUILD_ASSERT(DT_INST_REG_ADDR_BY_NAME(0, memory) == FSL_FEATURE_EEPROM_BASE_ADDRESS,
	     "EEPROM memory reg does not match the SDK base address");
BUILD_ASSERT(DT_INST_PROP(0, size) <= FSL_FEATURE_EEPROM_SIZE,
	     "EEPROM size exceeds the array on this part");

struct eeprom_mcux_lpc_config {
	EEPROM_Type *base;
	const uint8_t *memory;
	size_t size;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	bool read_only;
};

struct eeprom_mcux_lpc_data {
	struct k_mutex lock;
};

#define EEPROM_MCUX_LPC_PAGE_SIZE (FSL_FEATURE_EEPROM_SIZE / FSL_FEATURE_EEPROM_PAGE_COUNT)

/* Program the words staged in the page buffer and wait for the cycle to end. */
static void eeprom_mcux_lpc_program(EEPROM_Type *base)
{
	EEPROM_ClearInterruptFlag(base, (uint32_t)kEEPROM_ProgramFinishInterruptEnable);
	base->CMD = FSL_FEATURE_EEPROM_PROGRAM_CMD;

	while ((EEPROM_GetInterruptStatus(base) &
		(uint32_t)kEEPROM_ProgramFinishInterruptEnable) == 0U) {
	}
}

static int eeprom_mcux_lpc_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	const struct eeprom_mcux_lpc_config *config = dev->config;
	struct eeprom_mcux_lpc_data *dev_data = dev->data;

	if (len == 0U) {
		return 0;
	}

	if ((offset < 0) || ((offset + len) > config->size)) {
		LOG_ERR("attempt to read past device boundary");
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);
	memcpy(data, config->memory + offset, len);
	k_mutex_unlock(&dev_data->lock);

	return 0;
}

static int eeprom_mcux_lpc_write(const struct device *dev, off_t offset, const void *data,
				 size_t len)
{
	const struct eeprom_mcux_lpc_config *config = dev->config;
	struct eeprom_mcux_lpc_data *dev_data = dev->data;
	const uint8_t *src = data;
	off_t pos = offset;

	if (config->read_only) {
		return -EACCES;
	}

	if (len == 0U) {
		return 0;
	}

	if ((offset < 0) || ((offset + len) > config->size)) {
		LOG_ERR("attempt to write past device boundary");
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	/*
	 * The array only takes word writes, so a byte-granular request becomes
	 * a read-modify-write of the words it touches. The hardware programs a
	 * page at a time, so the staged words are committed whenever the run
	 * crosses a page boundary and once at the end.
	 *
	 * EEPROM_Write() does the same thing but underflows its unsigned length
	 * when a write starts unaligned and is shorter than the rest of that
	 * word, so it is not used here.
	 */
	while (len > 0U) {
		off_t word = ROUND_DOWN(pos, sizeof(uint32_t));
		size_t skip = pos - word;
		size_t chunk = MIN(sizeof(uint32_t) - skip, len);
		uint32_t value;

		memcpy(&value, config->memory + word, sizeof(value));
		memcpy((uint8_t *)&value + skip, src, chunk);
		sys_write32(value, (mem_addr_t)(config->memory + word));

		src += chunk;
		pos += chunk;
		len -= chunk;

		if ((len == 0U) || ((pos % EEPROM_MCUX_LPC_PAGE_SIZE) == 0)) {
			eeprom_mcux_lpc_program(config->base);
		}
	}

	k_mutex_unlock(&dev_data->lock);

	return 0;
}

static size_t eeprom_mcux_lpc_size(const struct device *dev)
{
	const struct eeprom_mcux_lpc_config *config = dev->config;

	return config->size;
}

static int eeprom_mcux_lpc_init(const struct device *dev)
{
	const struct eeprom_mcux_lpc_config *config = dev->config;
	struct eeprom_mcux_lpc_data *data = dev->data;
	eeprom_config_t eeprom_config;
	uint32_t clock_rate;
	int ret;

	k_mutex_init(&data->lock);

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_get_rate(config->clock_dev, config->clock_subsys, &clock_rate);
	if (ret != 0) {
		return ret;
	}

	/*
	 * EEPROM_Init() only powers the array when the SDK is built with its own
	 * power control, which Zephyr does not enable. The array needs more than
	 * 100 us to settle after being powered.
	 */
	POWER_DisablePD(kPDRUNCFG_PD_EEPROM);
	k_busy_wait(100);

	EEPROM_GetDefaultConfig(&eeprom_config);
	/*
	 * The default programs the page after every word written to the array,
	 * which collides with the explicit program this driver issues once per
	 * page and faults the bus.
	 */
	eeprom_config.autoProgram = kEEPROM_AutoProgramDisable;
	EEPROM_Init(config->base, &eeprom_config, clock_rate);

	return 0;
}

static DEVICE_API(eeprom, eeprom_mcux_lpc_api) = {
	.read = eeprom_mcux_lpc_read,
	.write = eeprom_mcux_lpc_write,
	.size = eeprom_mcux_lpc_size,
};

static struct eeprom_mcux_lpc_data eeprom_mcux_lpc_data_0;

static const struct eeprom_mcux_lpc_config eeprom_mcux_lpc_config_0 = {
	.base = (EEPROM_Type *)DT_INST_REG_ADDR_BY_NAME(0, control),
	.memory = (const uint8_t *)DT_INST_REG_ADDR_BY_NAME(0, memory),
	.size = DT_INST_PROP(0, size),
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, name),
	.read_only = DT_INST_PROP(0, read_only),
};

DEVICE_DT_INST_DEFINE(0, eeprom_mcux_lpc_init, NULL, &eeprom_mcux_lpc_data_0,
		      &eeprom_mcux_lpc_config_0, POST_KERNEL, CONFIG_EEPROM_INIT_PRIORITY,
		      &eeprom_mcux_lpc_api);
