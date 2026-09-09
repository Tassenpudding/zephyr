/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc_sha

#include <zephyr/crypto/crypto.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <fsl_sha.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(crypto_mcux_lpc_sha, CONFIG_CRYPTO_LOG_LEVEL);

#define CRYPTO_MCUX_LPC_SHA256_DIGEST_SIZE 32U

/*
 * The engine keeps the running digest in hardware rather than in the context
 * that fsl_sha.c hands around, so two interleaved hashes would corrupt each
 * other. Only one session is handed out at a time.
 */
struct crypto_mcux_lpc_sha_data {
	struct k_mutex lock;
	sha_ctx_t sha_ctx;
	bool in_use;
};

struct crypto_mcux_lpc_sha_config {
	SHA_Type *base;
};

static int crypto_mcux_lpc_sha_compute(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish)
{
	const struct device *dev = ctx->device;
	const struct crypto_mcux_lpc_sha_config *config = dev->config;
	struct crypto_mcux_lpc_sha_data *data = dev->data;
	size_t digest_size = CRYPTO_MCUX_LPC_SHA256_DIGEST_SIZE;
	status_t status;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	status = SHA_Update(config->base, &data->sha_ctx, pkt->in_buf, pkt->in_len);
	if (status != kStatus_Success) {
		LOG_ERR("SHA_Update failed: %d", (int)status);
		ret = -EIO;
		goto out;
	}

	if (!finish) {
		ctx->started = true;
		goto out;
	}

	status = SHA_Finish(config->base, &data->sha_ctx, pkt->out_buf, &digest_size);
	if (status != kStatus_Success) {
		LOG_ERR("SHA_Finish failed: %d", (int)status);
		ret = -EIO;
		goto out;
	}

	/* Finishing erases the context, so start a fresh one for reuse. */
	ctx->started = false;
	status = SHA_Init(config->base, &data->sha_ctx, kSHA_Sha256);
	if (status != kStatus_Success) {
		ret = -EIO;
	}

out:
	k_mutex_unlock(&data->lock);

	return ret;
}

static int crypto_mcux_lpc_sha_begin_session(const struct device *dev, struct hash_ctx *ctx,
					     enum hash_algo algo)
{
	const struct crypto_mcux_lpc_sha_config *config = dev->config;
	struct crypto_mcux_lpc_sha_data *data = dev->data;
	status_t status;
	int ret = 0;

	if (algo != CRYPTO_HASH_ALGO_SHA256) {
		LOG_ERR("unsupported hash algorithm: %d", algo);
		return -ENOTSUP;
	}

	if ((ctx->flags & ~(CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS)) != 0U) {
		LOG_ERR("unsupported session flags: 0x%x", ctx->flags);
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->in_use) {
		ret = -EBUSY;
		goto out;
	}

	status = SHA_Init(config->base, &data->sha_ctx, kSHA_Sha256);
	if (status != kStatus_Success) {
		LOG_ERR("SHA_Init failed: %d", (int)status);
		ret = -EIO;
		goto out;
	}

	data->in_use = true;
	ctx->drv_sessn_state = data;
	ctx->hash_hndlr = crypto_mcux_lpc_sha_compute;
	ctx->started = false;

out:
	k_mutex_unlock(&data->lock);

	return ret;
}

static int crypto_mcux_lpc_sha_free_session(const struct device *dev, struct hash_ctx *ctx)
{
	struct crypto_mcux_lpc_sha_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->in_use = false;
	ctx->started = false;
	k_mutex_unlock(&data->lock);

	return 0;
}

static int crypto_mcux_lpc_sha_query_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
}

static int crypto_mcux_lpc_sha_init(const struct device *dev)
{
	const struct crypto_mcux_lpc_sha_config *config = dev->config;
	struct crypto_mcux_lpc_sha_data *data = dev->data;

	k_mutex_init(&data->lock);
	SHA_ClkInit(config->base);

	return 0;
}

static DEVICE_API(crypto, crypto_mcux_lpc_sha_api) = {
	.hash_begin_session = crypto_mcux_lpc_sha_begin_session,
	.hash_free_session = crypto_mcux_lpc_sha_free_session,
	.query_hw_caps = crypto_mcux_lpc_sha_query_caps,
};

static struct crypto_mcux_lpc_sha_data crypto_mcux_lpc_sha_data_0;

static const struct crypto_mcux_lpc_sha_config crypto_mcux_lpc_sha_config_0 = {
	.base = (SHA_Type *)DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, crypto_mcux_lpc_sha_init, NULL, &crypto_mcux_lpc_sha_data_0,
		      &crypto_mcux_lpc_sha_config_0, POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,
		      &crypto_mcux_lpc_sha_api);
