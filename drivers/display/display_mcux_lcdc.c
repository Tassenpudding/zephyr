/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc_lcdc

#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/display/panel.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/linker/devicetree_regions.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <soc.h>
#include <fsl_lcdc.h>
#include <fsl_clock.h>

LOG_MODULE_REGISTER(display_mcux_lcdc, CONFIG_DISPLAY_LOG_LEVEL);

static const uint32_t supported_fmts = PIXEL_FORMAT_RGB_565 | PIXEL_FORMAT_RGB_888;

struct mcux_lcdc_config {
	LCD_Type *base;
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pincfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	struct gpio_dt_spec backlight_gpio;
	lcdc_config_t lcdc_config;
	uint8_t *fb[2];
	size_t fb_size;
	uint16_t panel_width;
	uint16_t panel_height;
	enum display_pixel_format pixel_format;
};

struct mcux_lcdc_data {
	enum display_pixel_format pixel_format;
	size_t pixel_bytes;
	struct k_sem sem;
	volatile bool frame_pending;
	uint8_t back_idx;
	bool running;
};

static int mcux_lcdc_write(const struct device *dev, const uint16_t x, const uint16_t y,
			   const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct mcux_lcdc_config *config = dev->config;
	struct mcux_lcdc_data *data = dev->data;

	uint8_t *back = config->fb[data->back_idx];

	__ASSERT((data->pixel_bytes * desc->pitch * desc->height) <= desc->buf_size,
		 "Input buffer too small");

	LOG_DBG("W=%d, H=%d @%d,%d", desc->width, desc->height, x, y);

	/*
	 * A full-screen, tightly-packed framebuffer is required. LVGL supplies
	 * one when configured for full-refresh (see the shield defconfig). The
	 * frame is copied into the controller's own off-screen buffer, so the
	 * caller may reuse its buffer immediately and the on-screen frame is
	 * always a complete, stable snapshot. Partial updates are not supported.
	 */
	if ((x != 0) || (y != 0) || (desc->width != config->panel_width) ||
	    (desc->height != config->panel_height) || (desc->pitch != desc->width)) {
		LOG_ERR("Only full-screen framebuffer writes are supported");
		return -ENOTSUP;
	}

	/* Copy into the off-screen buffer (never the one currently scanned). */
	memcpy(back, buf, config->fb_size);

	/*
	 * Ensure the copy has reached memory before the LCDC DMA fetches it,
	 * then queue it. The controller latches UPBASE at the next vertical
	 * blank; wait for the always-on base-address-update interrupt to confirm
	 * the latch before swapping so the previous buffer stays intact while it
	 * is still on screen.
	 */
	barrier_dmem_fence_full();

	LCDC_SetPanelAddr(config->base, kLCDC_UpperPanel, (uint32_t)back);

	if (unlikely(!data->running)) {
		LCDC_Start(config->base);
		LCDC_PowerUp(config->base);
		data->running = true;
	}

	data->frame_pending = true;
	k_sem_take(&data->sem, K_FOREVER);

	/* Buffer is now on screen; the other one becomes the next back buffer. */
	data->back_idx ^= 1;

	return 0;
}

static int mcux_lcdc_blanking_off(const struct device *dev)
{
	const struct mcux_lcdc_config *config = dev->config;

	if (config->backlight_gpio.port) {
		return gpio_pin_set_dt(&config->backlight_gpio, 1);
	}

	return -ENOSYS;
}

static int mcux_lcdc_blanking_on(const struct device *dev)
{
	const struct mcux_lcdc_config *config = dev->config;

	if (config->backlight_gpio.port) {
		return gpio_pin_set_dt(&config->backlight_gpio, 0);
	}

	return -ENOSYS;
}

static int mcux_lcdc_set_pixel_format(const struct device *dev,
				      const enum display_pixel_format pixel_format)
{
	struct mcux_lcdc_data *data = dev->data;

	/* Format is fixed at init from the devicetree; only accept a no-op. */
	if (pixel_format != data->pixel_format) {
		LOG_ERR("Pixel format is fixed to the devicetree value");
		return -ENOTSUP;
	}

	return 0;
}

static int mcux_lcdc_set_orientation(const struct device *dev,
				     const enum display_orientation orientation)
{
	ARG_UNUSED(dev);

	if (orientation == DISPLAY_ORIENTATION_NORMAL) {
		return 0;
	}

	LOG_ERR("Changing display orientation not implemented");
	return -ENOTSUP;
}

static void mcux_lcdc_get_capabilities(const struct device *dev,
				       struct display_capabilities *capabilities)
{
	const struct mcux_lcdc_config *config = dev->config;
	struct mcux_lcdc_data *data = dev->data;

	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = config->panel_width;
	capabilities->y_resolution = config->panel_height;
	capabilities->supported_pixel_formats = supported_fmts;
	capabilities->current_pixel_format = data->pixel_format;
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static void mcux_lcdc_isr(const struct device *dev)
{
	const struct mcux_lcdc_config *config = dev->config;
	struct mcux_lcdc_data *data = dev->data;
	uint32_t status;

	status = LCDC_GetInterruptsPendingStatus(config->base);
	LCDC_ClearInterruptsStatus(config->base, status);

	/*
	 * The base-address-update flag asserts every frame; only act on it when a
	 * flush is actually waiting, so a stale flag can never release the writer
	 * early. This mirrors the reference driver's frame-pending handshake.
	 */
	if (data->frame_pending && (status & kLCDC_BaseAddrUpdateInterrupt)) {
		data->frame_pending = false;
		k_sem_give(&data->sem);
	}
}

static int mcux_lcdc_init(const struct device *dev)
{
	const struct mcux_lcdc_config *config = dev->config;
	struct mcux_lcdc_data *data = dev->data;
	uint32_t src_clock_hz;
	int ret;

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	if (config->backlight_gpio.port) {
		ret = gpio_pin_configure_dt(&config->backlight_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret) {
			return ret;
		}
	}

	/*
	 * Route the LCD clock root: attach the main clock and divide by one, so
	 * CLOCK_GetLcdClkFreq() reports the input the LCDC divides down to the
	 * panel pixel clock. Matches the reference design's DEMO_InitLcdClock().
	 */
	CLOCK_AttachClk(kMAIN_CLK_to_LCD_CLK);
	CLOCK_SetClkDiv(kCLOCK_DivLcdClk, 1, true);

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret) {
		return ret;
	}

	ret = clock_control_get_rate(config->clock_dev, config->clock_subsys, &src_clock_hz);
	if (ret) {
		return ret;
	}

	k_sem_init(&data->sem, 0, 1);

	config->irq_config_func(dev);

	if (LCDC_Init(config->base, &config->lcdc_config, src_clock_hz) != kStatus_Success) {
		LOG_ERR("LCDC init failed");
		return -EIO;
	}

	/*
	 * Enable the base-address-update interrupt once and leave it on. It fires
	 * at the start of every vertical back porch; mcux_lcdc_write() uses it via
	 * the frame_pending flag to synchronize buffer swaps to vertical blank.
	 */
	LCDC_EnableInterrupts(config->base, kLCDC_BaseAddrUpdateInterrupt);

	LOG_DBG("LCDC up: %ux%u, src clock %u Hz", config->panel_width, config->panel_height,
		src_clock_hz);

	return 0;
}

static DEVICE_API(display, mcux_lcdc_api) = {
	.blanking_on = mcux_lcdc_blanking_on,
	.blanking_off = mcux_lcdc_blanking_off,
	.write = mcux_lcdc_write,
	.get_capabilities = mcux_lcdc_get_capabilities,
	.set_pixel_format = mcux_lcdc_set_pixel_format,
	.set_orientation = mcux_lcdc_set_orientation,
};

#define LCDC_TIMING(id, cell) DT_PROP(DT_INST_CHILD(id, display_timings), cell)

/* Full-screen framebuffer size in bytes for this instance. */
#define LCDC_FB_SIZE(id)                                                                           \
	(DT_INST_PROP(id, width) * DT_INST_PROP(id, height) *                                      \
	 (DISPLAY_BITS_PER_PIXEL(DT_INST_PROP(id, pixel_format)) / BITS_PER_BYTE))

/* Linker section for the framebuffers, taken from the memory-region phandle. */
#define LCDC_FB_SECTION(id) LINKER_DT_NODE_REGION_NAME_TOKEN(DT_INST_PHANDLE(id, memory_region))

/* Map the devicetree panel pixel format to the LCDC bits-per-pixel setting. */
#define LCDC_BPP(id)                                                                               \
	((DT_INST_PROP(id, pixel_format) == PANEL_PIXEL_FORMAT_RGB_888)                            \
		 ? kLCDC_24BPP                                                                     \
		 : kLCDC_16BPP565)

/*
 * LCDC polarity flags are "invert" bits: a set bit makes the signal active-low
 * (HSYNC/VSYNC/DE) or drives data on the falling clock edge. The panel-timing
 * "*-active" properties use 1 for active-high / rising edge, so invert when 0.
 */
#define LCDC_POLARITY(id)                                                                          \
	((LCDC_TIMING(id, hsync_active) ? 0 : kLCDC_InvertHsyncPolarity) |                         \
	 (LCDC_TIMING(id, vsync_active) ? 0 : kLCDC_InvertVsyncPolarity) |                         \
	 (LCDC_TIMING(id, de_active) ? 0 : kLCDC_InvertDePolarity) |                               \
	 (LCDC_TIMING(id, pixelclk_active) ? 0 : kLCDC_InvertClkPolarity))

#define MCUX_LCDC_DEVICE_INIT(id)                                                                  \
	PINCTRL_DT_INST_DEFINE(id);                                                                 \
	static void mcux_lcdc_config_func_##id(const struct device *dev);                          \
	static uint8_t mcux_lcdc_fb_##id[2][LCDC_FB_SIZE(id)] __aligned(8)                          \
		Z_GENERIC_SECTION(LCDC_FB_SECTION(id));                                            \
	static const struct mcux_lcdc_config mcux_lcdc_config_##id = {                              \
		.base = (LCD_Type *)DT_INST_REG_ADDR(id),                                          \
		.irq_config_func = mcux_lcdc_config_func_##id,                                     \
		.fb = {mcux_lcdc_fb_##id[0], mcux_lcdc_fb_##id[1]},                                 \
		.fb_size = LCDC_FB_SIZE(id),                                                        \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(id),                                      \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(id)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(id, name),             \
		.backlight_gpio = GPIO_DT_SPEC_INST_GET_OR(id, backlight_gpios, {0}),              \
		.panel_width = DT_INST_PROP(id, width),                                            \
		.panel_height = DT_INST_PROP(id, height),                                          \
		.pixel_format = DT_INST_PROP(id, pixel_format),                                    \
		.lcdc_config = {                                                                   \
			.panelClock_Hz = LCDC_TIMING(id, clock_frequency),                         \
			.ppl = DT_INST_PROP(id, width),                                            \
			.hsw = LCDC_TIMING(id, hsync_len),                                         \
			.hfp = LCDC_TIMING(id, hfront_porch),                                      \
			.hbp = LCDC_TIMING(id, hback_porch),                                       \
			.lpp = DT_INST_PROP(id, height),                                           \
			.vsw = LCDC_TIMING(id, vsync_len),                                         \
			.vfp = LCDC_TIMING(id, vfront_porch),                                      \
			.vbp = LCDC_TIMING(id, vback_porch),                                       \
			.polarityFlags = LCDC_POLARITY(id),                                        \
			.upperPanelAddr = 0,                                                       \
			.lowerPanelAddr = 0,                                                       \
			.bpp = LCDC_BPP(id),                                                       \
			.dataFormat = kLCDC_LittleEndian,                                          \
			.swapRedBlue = DT_INST_PROP(id, nxp_swap_red_blue),                        \
			.display = kLCDC_DisplayTFT,                                               \
		},                                                                                 \
	};                                                                                         \
	static struct mcux_lcdc_data mcux_lcdc_data_##id = {                                        \
		.pixel_format = DT_INST_PROP(id, pixel_format),                                    \
		.pixel_bytes = DISPLAY_BITS_PER_PIXEL(DT_INST_PROP(id, pixel_format)) / BITS_PER_BYTE, \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(id, mcux_lcdc_init, NULL, &mcux_lcdc_data_##id,                       \
			      &mcux_lcdc_config_##id, POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,    \
			      &mcux_lcdc_api);                                                     \
	static void mcux_lcdc_config_func_##id(const struct device *dev)                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(id), DT_INST_IRQ(id, priority), mcux_lcdc_isr,             \
			    DEVICE_DT_INST_GET(id), 0);                                            \
		irq_enable(DT_INST_IRQN(id));                                                      \
	}

DT_INST_FOREACH_STATUS_OKAY(MCUX_LCDC_DEVICE_INIT)
