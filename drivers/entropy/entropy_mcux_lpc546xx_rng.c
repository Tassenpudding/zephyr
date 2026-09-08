/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc546xx_rng

#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>

#include <fsl_rng.h>

/*
 * The generator is only reachable through the boot ROM, which powers and clocks
 * it around every read, so the driver needs no init function.
 */

static int entropy_mcux_lpc546xx_rng_get_entropy(const struct device *dev, uint8_t *buffer,
						 uint16_t length)
{
	ARG_UNUSED(dev);

	if (RNG_GetRandomData(buffer, length) != kStatus_Success) {
		return -EINVAL;
	}

	return 0;
}

static DEVICE_API(entropy, entropy_mcux_lpc546xx_rng_api) = {
	.get_entropy = entropy_mcux_lpc546xx_rng_get_entropy,
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, PRE_KERNEL_1, CONFIG_ENTROPY_INIT_PRIORITY,
		      &entropy_mcux_lpc546xx_rng_api);
