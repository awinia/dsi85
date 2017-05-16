/*
 * Copyright (C) 2017, Hella-Gutmann Solutions GmbH
 *
 * Partly based on panel-dsi85.c from:
 *
 * Copyright 2011 Texas Instruments, Inc.
 * Author: Archit Taneja <archit@ti.com>
 * based on d2l panel driver by Jerry Alexander <x0135174@ti.com>
 * 
 * Also partly based on the ptn3460 and adv7533 kernel drivers.
 *
 * This program iss free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details. 
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DEBUG

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_edid.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>

#include <video/mipi_display.h>

#include "dsi85_registers.h"

#define SN65DSI85_MODULE_NAME "bridge-sn65dsi85"

/* Config struct
 * (to be filled from DT)
 */
struct sn65dsi85_config {
	/* see mipi_dsi_device */
	unsigned int lanes; 
	enum mipi_dsi_pixel_format format;
	unsigned long mode_flags;
};

/*
 * Device state
 */
struct sn65dsi85_device {
	struct sn65dsi85_config *config;
	struct i2c_client *i2c;
	const struct i2c_device_id *i2c_id;
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct drm_panel *panel;
	struct device_node *dsi_host_node;
	struct mipi_dsi_device *dsi;
};

static inline struct sn65dsi85_device *bridge_to_sn65dsi85(struct drm_bridge *bridge)
{
	return container_of(bridge, struct sn65dsi85_device, bridge);
}
static inline struct sn65dsi85_device *connector_to_sn65dsi85(struct drm_connector *connector)
{
	return container_of(connector, struct sn65dsi85_device, connector);
}

static int dsi85_hardcoded_deinit(struct i2c_client* client)
{
	/* disable pll */
	i2c_smbus_write_byte_data(client, 0x0D, 0x00);
	return 0;
}

/*
 * Read configuration from metadata
 */
static struct sn65dsi85_config*
sn65dsi85_get_pdata(struct i2c_client *client)
{
	struct device_node *endpoint = NULL;
	struct sn65dsi85_config *pdata = NULL;
	
	endpoint = of_graph_get_next_endpoint(client->dev.of_node, NULL);
	if(!endpoint) {
		dev_err(&client->dev, "Cannot find OF endpoint\n");
		return NULL;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if(!pdata) {
		dev_err(&client->dev, "Failed to kzalloc the config object");
		goto done;
	}

	// TODO read number of lanes, bitformat etc.

done:
	of_node_put(endpoint);
	return pdata;	
}

static enum drm_connector_status sn65dsi85_connector_detect(struct drm_connector *connector,
		bool force)
{
	dev_dbg(connector->dev->dev, "%s entry", __func__);
	return connector_status_connected;
}

static void sn65dsi85_connector_destroy(struct drm_connector* connector)
{
	dev_dbg(connector->dev->dev, "%s entry", __func__);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs sn65dsi85_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = sn65dsi85_connector_detect,
	.destroy = sn65dsi85_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct drm_encoder *sn65dsi85_best_encoder(struct drm_connector* connector)
{
	struct sn65dsi85_device *self = connector_to_sn65dsi85(connector);
	/* The bridge's encoder is the best encoder */
	return self->bridge.encoder;
}

/* Get modes (connector_helper callback), delegated to Panel */
static int sn65dsi85_get_modes(struct drm_connector* connector)
{
	struct sn65dsi85_device *self;
	int num_modes = 0;
	dev_dbg(connector->dev->dev, "%s entry connector=%p", __func__, connector);

	if(!connector) {
		pr_err("Null connector in sn65dsi85_get_modes");
		return num_modes;
	}
	self = connector_to_sn65dsi85(connector);

	if (self->panel && self->panel->funcs && self->panel->funcs->get_modes) {
		num_modes = self->panel->funcs->get_modes(self->panel);
		if (num_modes > 0) {
			dev_dbg(connector->dev->dev, "%s got %d modes from panel, good enough", __func__, num_modes);
		}
		else {
			dev_warn(connector->dev->dev, "%s got 0 modes from panel", __func__);
		}
	}
	else {
		dev_warn(connector->dev->dev, "%s could not interrogate drm_panel for modes", __func__);
	}

	return num_modes;
}

static struct drm_connector_helper_funcs sn65dsi85_connector_helper_funcs = {
	.get_modes = sn65dsi85_get_modes,
	.best_encoder = sn65dsi85_best_encoder,
};

static void sn65dsi85_bridge_pre_enable(struct drm_bridge* bridge)
{
	struct sn65dsi85_device *self = bridge_to_sn65dsi85(bridge);
	int res;
	dev_dbg(bridge->dev->dev, "%s entry", __func__);

	res = drm_panel_prepare(self->panel);
	if (res) {
		DRM_ERROR("failed to prepare my panel: %d\n", res);
		return;
	}

}
static void sn65dsi85_bridge_enable(struct drm_bridge* bridge)
{
	struct sn65dsi85_device *self = bridge_to_sn65dsi85(bridge);
	int res;
	dev_dbg(bridge->dev->dev, "%s entry", __func__);

	dev_info(bridge->dev->dev, "%s: time for PLL-Enable", __func__);
	i2c_smbus_write_byte_data(self->i2c, 0x0D, 0x01);
	i2c_smbus_write_byte_data(self->i2c, 0x09, 0x01);			
	dev_info(bridge->dev->dev, "%s: PLL-Enable done", __func__);

	res = drm_panel_enable(self->panel);
	if (res) {
		DRM_ERROR("failed to enable my panel: %d\n", res);
		return;
	}
}
static void sn65dsi85_bridge_disable(struct drm_bridge* bridge)
{
	struct sn65dsi85_device *self = bridge_to_sn65dsi85(bridge);
	int res;
	dev_dbg(bridge->dev->dev, "%s entry", __func__);

	dev_info(bridge->dev->dev, "Time for hardcoded deinit");
	dsi85_hardcoded_deinit(self->i2c);
	dev_info(bridge->dev->dev, "Done hardcoded deinit");

	res = drm_panel_disable(self->panel);
	if (res) {
		DRM_ERROR("failed to disable my panel: %d\n", res);
		return;
	}
}
static void sn65dsi85_bridge_post_disable(struct drm_bridge* bridge)
{
	struct sn65dsi85_device *self = bridge_to_sn65dsi85(bridge);
	int res;
	dev_dbg(bridge->dev->dev, "%s entry", __func__);

	res = drm_panel_unprepare(self->panel);
	if (res) {
		DRM_ERROR("failed to unprepare my panel: %d\n", res);
		return;
	}
}

/*
 * Configure DSI link properties by creating a mipi_dsi_device
 */
int sn65dsi85_attach_dsi(struct sn65dsi85_device *self)
{
	struct device *dev = &self->i2c->dev;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	int ret = 0;
	const struct mipi_dsi_device_info info = { .type = "sn65dsi85_dsi_client",
						   .channel = 0,
						   .node = NULL,
						 };

	host = of_find_mipi_dsi_host_by_node(self->dsi_host_node);
	if (!host) {
		dev_err(dev, "%s failed to find dsi host by OF node %s\n", __func__,
			self->dsi_host_node? self->dsi_host_node->full_name : "null-OF-node");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		ret = PTR_ERR(dsi);
		dev_err(dev, "%s failed to create dsi device: %d\n", __func__, ret);
		goto err_dsi_device;
	}

	self->dsi = dsi;

	/* In the simple driver, this was part of the panel_desc_dsi instance: */
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO;
	/* TODO get these params from my DT instead */

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "%s failed to attach dsi to host: %d\n", __func__, ret);
		goto err_dsi_attach;
	}

	return 0;

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_device:
	return ret;
}

void sn65dsi85_detach_dsi(struct sn65dsi85_device *self)
{
	mipi_dsi_detach(self->dsi);
	mipi_dsi_device_unregister(self->dsi);
}

/*
 * Main Bridge config callback.
 *
 * Create a DRM Connector which answers queries for supported modes
 * (by delegating to the Panel, in our case).
 * Create a DSI Device which holds the DSI protocol details.
 */
static int sn65dsi85_bridge_attach(struct drm_bridge* bridge)
{
	struct sn65dsi85_device* self = bridge_to_sn65dsi85(bridge);
	int ret = 0;

	dev_dbg(bridge->dev->dev, "%s entry", __func__);

	if(!bridge->encoder) {
		DRM_ERROR("Bridge has no Encoder");
		return -ENODEV;
	}

	self->connector.polled = 0; // no hotplugging here
	ret = drm_connector_init(bridge->dev, &self->connector,
				 &sn65dsi85_connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
	if(ret) {
		DRM_ERROR("Could not drm_connector_init: %d", ret);
		return ret;
	}
	drm_connector_helper_add(&self->connector, &sn65dsi85_connector_helper_funcs);

	/* Tempting to do right now, but not a good idea:
	 *   drm_connector_register(&self->connector);
	 * Wants to create kobject nodes, but the connector has no kobject parent as of yet. 
	 * The kobject parent will be assigned later, 
	 * but drm_connector_init has already added our connector
	 * to its internal list, and there will be a call to drm_connector_register_all later.
	 */ 

	ret = drm_mode_connector_attach_encoder(&self->connector, self->bridge.encoder);
	if(ret) {
		DRM_ERROR("Could not drm_mode_connector_attach_encoder: %d", ret);
		return ret;
	}

	/* In order to configure the DSI parameters, create a mipi_dsi_device
	 */
	ret = sn65dsi85_attach_dsi(self);
	if(ret) {
		DRM_ERROR("Could not sn65dsi85_attach_dsi: %d", ret);
		return ret;
	}
	

	if (self->panel)
		drm_panel_attach(self->panel, &self->connector);

	/* but no hotplug IRQ. Other drivers do that here. */


	return ret;
}

static inline void write_dsi85(struct i2c_client *i2c, u8 reg, u8 value)
{
	int res;
	res = i2c_smbus_write_byte_data(i2c, reg, value);
	dev_dbg(&i2c->dev, "write reg 0x%X <- 0x%X (res=%d)", reg, value, res);
	if(res < 0) {
		dev_err(&i2c->dev, "could not write DSI85 register 0x%x, i2c error %d", reg, res);
	}
}

static void sn65dsi85_apply_mode(struct sn65dsi85_device *self,
				struct drm_display_mode *mode)
{
	struct i2c_client *i2c = self->i2c;
	u8 val;

	struct sn65dsi85_regs {
		u8 pll_en_stat;
		u8 lvds_clk_range;
		u8 hs_clk_src;
		u8 dsi_clk_divider;
		u8 refclk_multiplier;

		u8 left_right_pixels;
		u8 dsi_channel_mode;
		u8 cha_dsi_lanes;
		u8 chb_dsi_lanes;
		u8 sot_err_tol_dis;
		u8 cha_dsi_data_eq;
		u8 chb_dsi_data_eq;
		u8 cha_dsi_clk_eq;
		u8 chb_dsi_clk_eq;

		u8 cha_dsi_clk_rng;
		u8 chb_dsi_clk_rng;

		u8 de_neg_polarity;
		u8 hs_neg_polarity;
		u8 vs_neg_polarity;
		u8 lvds_link_cfg;
		u8 cha_24bpp_mode;
		u8 chb_24bpp_mode;
		u8 cha_24bpp_format1;
		u8 chb_24bpp_format1;

		u8 cha_lvds_vocm;
		u8 chb_lvds_vocm;
		u8 cha_lvds_vod_swing;
		u8 chb_lvds_vod_swing;

		u8 even_odd_swap;
		u8 cha_reverse_lvds;
		u8 chb_reverse_lvds;
		u8 cha_lvds_term;
		u8 chb_lvds_term;

		u16 cha_active_line_length;
		u16 chb_active_line_length;

		u16 cha_sync_delay;
		u16 chb_sync_delay;

		u16 cha_hsync_pulse_width;
		u16 chb_hsync_pulse_width;
		
		u16 cha_vsync_pulse_width;
		u16 chb_vsync_pulse_width;
		
		u8 cha_horizontal_back_porch;
		u8 chb_horizontal_back_porch;

		/* omitted: TPG-related */
	} regs = {
		.pll_en_stat = 0,
		.lvds_clk_range = 0,
		.hs_clk_src = 1, /* LVDS pixel clock derived from MIPI D-PHY channel A HS continuous clock */

		.dsi_clk_divider = 10,
		.refclk_multiplier = 0,

		.left_right_pixels = 0,
		.dsi_channel_mode = 1, /* single channel DSI receiver */
		.cha_dsi_lanes = 0,    /* four lanes A */
		.chb_dsi_lanes = 0x3,  /* one lane B (default) */
		.sot_err_tol_dis = 0,  /* tolerate single bit errors for SoT leader sequence */

		.cha_dsi_data_eq = 0,
		.chb_dsi_data_eq = 0,

		.cha_dsi_clk_eq = 0,
		.chb_dsi_clk_eq = 0,		
		.cha_dsi_clk_rng = 0x5D, /* Magic? TODO */
		.chb_dsi_clk_rng = 0,
		
		.de_neg_polarity = 0,
		.hs_neg_polarity = 1,
		.vs_neg_polarity = 1,
		.lvds_link_cfg = 0, /* enable channel A and channel B */
		.cha_24bpp_mode = 1, /* force 24bpp */
		.chb_24bpp_mode = 1, /* force 24bpp */
		.cha_24bpp_format1 = 0, /* use format2 */
		.chb_24bpp_format1 = 0, /* use format2 */

		.cha_lvds_vocm = 0,
		.chb_lvds_vocm = 0,
		.cha_lvds_vod_swing = 0,
		.chb_lvds_vod_swing = 0,

		.even_odd_swap = 0,
		.cha_reverse_lvds = 0,
		.chb_reverse_lvds = 0,
		.cha_lvds_term = 1, /* terminate */
		.chb_lvds_term = 1,

		.cha_active_line_length = mode->hdisplay,
		.chb_active_line_length = 0,
		
		.cha_sync_delay = 33, /* Magic? TODO */
		.chb_sync_delay = 0,
		
		.cha_hsync_pulse_width = mode->hsync_end - mode->hsync_start,
		.chb_hsync_pulse_width = 0,
		
		.cha_vsync_pulse_width = mode->vsync_end - mode->vsync_start,
		.chb_vsync_pulse_width = 0,

		.cha_horizontal_back_porch = mode->hsync_start - mode->hdisplay,
		.chb_horizontal_back_porch = 0,

	};

	/* Adjust those defaults */
	if (mode->clock <= 37500) {
		/* use 0 */
	}
	else if (mode->clock <= 62500) {
		regs.lvds_clk_range = 0x01;
	}
	else if (mode->clock <= 87500){
		regs.lvds_clk_range = 0x02;
	}
	else if (mode->clock <= 112500)	{
		regs.lvds_clk_range = 0x03;
	}
	else if (mode->clock <= 137500) {
		regs.lvds_clk_range = 0x04;
	}
	else {
		regs.lvds_clk_range = 0x05;
	}

	/* PFUSCH */
	/* These registers are still off from the hard-coded reference */
	regs.lvds_clk_range = 2;
	regs.cha_hsync_pulse_width = 44;
	regs.cha_horizontal_back_porch = 48;
	/* /PFUSCH */


	/* Soft reset and disable PLL */
	write_dsi85(i2c,  DSI85_SOFT_RESET, 1 );
	write_dsi85(i2c,  DSI85_PLL_EN, 0 );

	/* ------- CORE_PLL register ------- */
	write_dsi85(i2c,  DSI85_CORE_PLL, 
			     ((regs.pll_en_stat & 0x01) << 7)
			     | ((regs.lvds_clk_range & 0x07) << 1)
			     | ((regs.hs_clk_src & 0x01) << 0 ));

	/* ------- PLL_DIV register ------- */
	write_dsi85(i2c,  DSI85_PLL_DIV, 
			     (regs.dsi_clk_divider << 2) 
			     | (regs.refclk_multiplier & 0x3) );

	/* ------- DSI ------- */
	write_dsi85(i2c,  DSI85_DSI_CFG, 
			     ((regs.left_right_pixels & 0x01) << 7)
			     | ((regs.dsi_channel_mode & 0x03) << 5)
			     | ((regs.cha_dsi_lanes & 0x03) << 3)
			     | ((regs.chb_dsi_lanes & 0x03) << 1)
			     | ((regs.sot_err_tol_dis) << 0));
	write_dsi85(i2c,  DSI85_DSI_EQ,
			     ((regs.cha_dsi_data_eq & 0x03) << 6)
			     | ((regs.chb_dsi_data_eq & 0x03) << 4)
			     | ((regs.cha_dsi_clk_eq & 0x03) << 2)
			     | ((regs.chb_dsi_clk_eq & 0x03) << 0));
	write_dsi85(i2c,  DSI85_CHA_DSI_CLK_RNG,
			     regs.cha_dsi_clk_rng);
	write_dsi85(i2c,  DSI85_CHB_DSI_CLK_RNG,
			     regs.chb_dsi_clk_rng);
	
	/* ------- LVDS ------- */

	write_dsi85(i2c,  DSI85_LVDS_MODE,
			     ((regs.de_neg_polarity & 0x01) << 7)
			     | ((regs.hs_neg_polarity & 0x01) << 6)
			     | ((regs.vs_neg_polarity & 0x01) << 5)
			     | ((regs.lvds_link_cfg   & 0x01) << 4)
			     | ((regs.cha_24bpp_mode  & 0x01) << 3)
			     | ((regs.chb_24bpp_mode  & 0x01) << 2)
			     | ((regs.cha_24bpp_format1  & 0x01) << 1)
			     | ((regs.chb_24bpp_format1  & 0x01) << 0));
	write_dsi85(i2c,  DSI85_LVDS_SIGN,
			     ((regs.cha_lvds_vocm & 0x01) << 6)
			     | ((regs.chb_lvds_vocm & 0x01) << 4)
			     | ((regs.cha_lvds_vod_swing & 0x03) << 2)
			     | ((regs.chb_lvds_vod_swing & 0x03) << 0));
	write_dsi85(i2c,  DSI85_LVDS_TERM,
			     ((regs.even_odd_swap & 0x01) << 6)
			     | ((regs.cha_reverse_lvds & 0x01) << 5)
			     | ((regs.chb_reverse_lvds & 0x01) << 4)
			     | ((regs.cha_lvds_term & 0x01) << 1)
			     | ((regs.chb_lvds_term & 0x01) << 0));
	write_dsi85(i2c,  DSI85_LVDS_ADJUST, 0 ); /* TPG only */
	
	/* ------- Dimensions ------- */
	write_dsi85(i2c,  DSI85_CHA_LINE_LEN_LO, 
			     (u8)(regs.cha_active_line_length & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHA_LINE_LEN_HI, 
			     (u8)((regs.cha_active_line_length & 0x0f00u) >> 8));
	write_dsi85(i2c,  DSI85_CHB_LINE_LEN_LO, 
			     (u8)(regs.chb_active_line_length & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHB_LINE_LEN_HI, 
			     (u8)((regs.chb_active_line_length & 0x0f00u) >> 8));

	write_dsi85(i2c,  DSI85_CHA_VERT_LINES_LO, 0 ); /* TPG only */
	write_dsi85(i2c,  DSI85_CHA_VERT_LINES_HI, 0 );
	write_dsi85(i2c,  DSI85_CHB_VERT_LINES_LO, 0 );
	write_dsi85(i2c,  DSI85_CHB_VERT_LINES_HI, 0 );

	write_dsi85(i2c,  DSI85_CHA_SYNC_DELAY_LO, 
			     (u8)(regs.cha_sync_delay & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHA_SYNC_DELAY_HI, 
			     (u8)((regs.cha_sync_delay & 0x0f00u) >> 8));
	write_dsi85(i2c,  DSI85_CHB_SYNC_DELAY_LO, 
			     (u8)(regs.chb_sync_delay & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHB_SYNC_DELAY_HI, 
			     (u8)((regs.chb_sync_delay & 0x0f00u) >> 8));

	write_dsi85(i2c,  DSI85_CHA_HSYNC_WIDTH_LO, 
			     (u8)(regs.cha_hsync_pulse_width & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHA_HSYNC_WIDTH_HI, 
			     (u8)((regs.cha_hsync_pulse_width & 0x0f00u) >> 8));
	write_dsi85(i2c,  DSI85_CHB_HSYNC_WIDTH_LO, 
			     (u8)(regs.chb_hsync_pulse_width & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHB_HSYNC_WIDTH_HI, 
			     (u8)((regs.chb_hsync_pulse_width & 0x0f00u) >> 8));

	write_dsi85(i2c,  DSI85_CHA_VSYNC_WIDTH_LO, 
			     (u8)(regs.cha_vsync_pulse_width & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHA_VSYNC_WIDTH_HI, 
			     (u8)((regs.cha_vsync_pulse_width & 0x0f00u) >> 8));
	write_dsi85(i2c,  DSI85_CHB_VSYNC_WIDTH_LO, 
			     (u8)(regs.chb_vsync_pulse_width & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHB_VSYNC_WIDTH_HI, 
			     (u8)((regs.chb_vsync_pulse_width & 0x0f00u) >> 8));

	write_dsi85(i2c,  DSI85_CHA_HORZ_BACKPORCH, regs.cha_horizontal_back_porch );
	write_dsi85(i2c,  DSI85_CHB_HORZ_BACKPORCH, regs.chb_horizontal_back_porch );

	write_dsi85(i2c,  DSI85_CHA_VERT_BACKPORCH, 0 ); /* TPG only */
	write_dsi85(i2c,  DSI85_CHB_VERT_BACKPORCH, 0 ); /* TPG only */
	write_dsi85(i2c,  DSI85_CHA_HORZ_FRONTPORCH, 0 ); /* TPG only */
	write_dsi85(i2c,  DSI85_CHB_HORZ_FRONTPORCH, 0 ); /* TPG only */
	write_dsi85(i2c,  DSI85_CHA_HORZ_FRONTPORCH, 0 ); /* TPG only */
	write_dsi85(i2c,  DSI85_CHB_VERT_FRONTPORCH, 0 ); /* TPG only */
	
}


static void sn65dsi85_bridge_mode_set(struct drm_bridge *bridge,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	int i;
	int result;
	struct sn65dsi85_device *self = bridge_to_sn65dsi85(bridge);

	dev_dbg(bridge->dev->dev, "%s entry", __func__);
	sn65dsi85_apply_mode(self, mode);

#if 0	
	static const struct reg_assignment
		cfg_regs[] =
{
	{0x09, 0x01}, // soft reset
	{0x0D, 0x00}, // pll disable

	{0x09, 0x00}, // soft reset = 0
	{0x0A, 0x05}, // CORE_PLL = 0x05: HS_CLK_SRC=1 (from MIPI), LVDS_CLK_RANGE=62.5MHz..87.5MHz
	{0x0B, 0x28}, // PLL_DIV = 0x0B: DSI_CLK_DIVIDER=divide by 3, REFCLK_MULTIPLIER=mul by 4
	{0x0D, 0x00}, // PLL_EN = 0: disabled
	{0x10, 0x26}, // CSR: DSI_CFG = 0x26: Even-odd split; Single-channel DSI receiver(1); four lanes A; one lane B  ; no SoT bit errors tolerated
	{0x11, 0x00}, // DSI_EQ = 0: no Equalization
	{0x12, 0x5d}, // CHA_DSI_CLK_RNG = 0x5D: DSI channel A clock in range 465..470MHz
	{0x13, 0x00}, // CHB_DSI_CLK_RNG = 0x00: reserved channel B clock
	{0x18, 0x6c}, // LVDS_MODE = 0x6C: DE positive polarity; HS neg polarity; VS neg polarity; LVDS channels A and B enabled; LVDS ch A force 24bpp; LVDS ch B force 24bpp; LVDS ch A format 2; LVDS ch B format 2
	{0x19, 0x00}, // LVDS_SIGN = 0: LVDS ch A 1.2V; LVDS ch B 1.2V; CHA VOD SWING 0; CHB VOD SWING 0; 
	{0x1A, 0x03}, // LVDS_TERM = 0x03: LVDS ch A gets odd pixels; normal A pin order; normal B pin order; both termination enabled
	{0x1B, 0x00}, // CHAB_LVDS_CM_ADJUST = 0: use common mode voltage
	{0x20, 0x80}, // CHA_LINE_LEN_LO = 0x80: active horiz line 1920=0x780 pixels
	{0x21, 0x07}, // CHA_LINE_LEN_HI = 0x07
	{0x22, 0x00}, // CHB_LINE_LEN_LO = 0
	{0x23, 0x00}, // CHB_LINE_LEN_HI = 0
	{0x24, 0x00}, // CHA_VERT_LINES_LO = 0: TPG only
	{0x25, 0x00}, // CHA_VERT_LINES_HI = 0
	{0x26, 0x00}, // CHB_VERT_LINES_LO = 0
	{0x27, 0x00}, // CHB_VERT_LINES_HI = 0
	{0x28, 0x21}, // CHA_SYNC_DELAY_LOW = 0x21: delay pixel clock by 33 clocks
	{0x29, 0x00},
	{0x2A, 0x00}, 
	{0x2B, 0x00},
	{0x2C, 0x2c}, // CHA_HSYNC_PULSE_WIDTH_LOW: ch A hsync pulse width 0x2c=44 clocks
	{0x2D, 0x00},
	{0x2E, 0x00}, // CHB_HSYNC_PULSE_WIDTH_LOW: ch B hsync pulse width 0
	{0x2F, 0x00},
	{0x30, 0x0f}, // CHA_VSYNC_PULSE_WIDTH_LOW: ch A vsync pulse 15 lines
	{0x31, 0x00},
	{0x32, 0x00}, // CHB_VSYNC_PULSE_WIDTH_LOW: ch B vsync pulse 0 lines
	{0x33, 0x00},
	{0x34, 0x30}, // CHA_HORIZONTAL_BACK_PORCH: ch A 48 clocks backporch
	{0x35, 0x00}, // CHB_HORIZONTAL_BACK_PORCH: ch B 0 clocks backporch
	{0x36, 0x00}, // CHA_VERTICAL_BACK_PORCH: TPG only
	{0x37, 0x00}, // CHB_VERTICAL_BACK_PORCH: TPG only
	{0x38, 0x00}, // CHA_HORIZONTAL_FRONT_PORCH: TPG only
	{0x39, 0x00}, // CHB_HORIZONTAL_FRONT_PORCH: TPG only
	{0x3A, 0x00}, // CHA_VERTICAL_FRONT_PORCH: TPG only
	{0x3B, 0x00}, // CHB_VERTICAL_FRONT_PORCH: TPG only
	{0x3C, 0x00}, // CHA_TEST_PATTERN=0, CHB_TEST_PATTERN=0
	{0x3D, 0x00}, // RIGHT_CROP = 0 (left-right only)
	{0x3E, 0x00}, // LEFT_CROP = 0 (left-right only)
};

	dev_dbg(bridge->dev->dev, "%s Configuring the DSI85, except for PLL-Enable", __func__);

	for (i = 0; cfg_regs[i].reg != 0x00; i++)
	{
		result = i2c_smbus_write_byte_data(self->i2c, cfg_regs[i].reg, 
						   cfg_regs[i].val);
		if (result < 0)
		{
			dev_err(bridge->dev->dev, "%s i2c write error at step %d: %d", __func__, i, result);
			break;
		}
	}
#endif //0
	dev_dbg(bridge->dev->dev, "%s exit", __func__);
}

static const struct drm_bridge_funcs sn65dsi85_bridge_funcs = {
	.pre_enable   = sn65dsi85_bridge_pre_enable,
	.enable       = sn65dsi85_bridge_enable,
	.disable      = sn65dsi85_bridge_disable,
	.post_disable = sn65dsi85_bridge_post_disable,
	.attach       = sn65dsi85_bridge_attach,
	.mode_set     = sn65dsi85_bridge_mode_set,
};

/*
 * Probe an SN65DSI85 on an I2C bus.
 *
 * This installs a drm_bridge into the DRM system to do the actual mode-setting work.
 *
 * @c i2c_client pointer
 * @id i2c_device_id pointer
 */
static int sn65dsi85_probe(struct i2c_client* c, const struct i2c_device_id *id)
{
	struct sn65dsi85_config *pdata = NULL;
	struct sn65dsi85_device *ctx = NULL;
	struct device_node *endpoint, *panel_node, *host_node;
	int error_value = 0;

	dev_info(&c->dev, "sn65dsi85_probe: welcome!\n");

	pdata = sn65dsi85_get_pdata(c);
	if(!pdata) {
		dev_err(&c->dev, "Cannot read configuration. Probe failed.\n");
		return -EINVAL;
	}

	ctx = devm_kzalloc(&c->dev, sizeof(struct sn65dsi85_device), GFP_KERNEL);
	if(!ctx) {
		dev_err(&c->dev, "Cannot allocate device state\n");
		error_value = -ENOMEM;
		goto fail_free_pdata;
	}
	ctx->config = pdata;
	ctx->i2c = c;
	ctx->i2c_id = id;
	i2c_set_clientdata(c, ctx);

	/* Find the associated DSI host. port@0 is input */
	endpoint = of_graph_get_endpoint_by_regs(c->dev.of_node, 0, -1);
	if (endpoint) {
		dev_dbg(&c->dev, "%s my DSI-side endpoint: %s", __func__, endpoint->full_name);
		host_node = of_graph_get_remote_port_parent(endpoint);
		if (host_node) {
			dev_dbg(&c->dev, "%s my DSI host node: %s", __func__, host_node->full_name);
			ctx->dsi_host_node = host_node;
		}
		else {
			dev_err(&c->dev, "%s cannot find DSI host node", __func__);
		}
	}
	else {
		dev_err(&c->dev, "%s cannot find endpoint 0 (towards DSI)", __func__);
		error_value = -ENOENT;
		goto fail_free_pdata;
	}

	/* Find the associated panel. port@1 is output */
	endpoint = of_graph_get_endpoint_by_regs(c->dev.of_node, 1, -1);
	if (endpoint) {
		dev_dbg(&c->dev, "%s my panel-side endpoint: %s", __func__, endpoint->full_name);
		panel_node = of_graph_get_remote_port_parent(endpoint);
		if (panel_node) {
			ctx->panel = of_drm_find_panel(panel_node);
			if (!ctx->panel) {
				dev_warn(&c->dev, "Cannot find panel by panel node name=%s", 
					 panel_node->full_name);
				return -EPROBE_DEFER;
			}
			of_node_put(panel_node);
		}
	}
	else {
		dev_err(&c->dev, "%s cannot find endpoint 1 (towards panel)", __func__);
		error_value = -ENOENT;
		goto fail_free_pdata;
	}

	ctx->bridge.funcs = &sn65dsi85_bridge_funcs;
	ctx->bridge.of_node = c->dev.of_node;
	
	error_value = drm_bridge_add(&ctx->bridge);
	(void)error_value; // "Unconditionally returns zero"

        dev_info(&c->dev, "sn65dsi85_probe: done!\n");

	return 0;

fail_free_pdata:
	kfree(pdata);
        dev_info(&c->dev, "sn65dsi85_probe: err=%d :(\n", error_value);
	return error_value;
	
}

/*
 * Remove SN65DSI85
 * @c: pointer to the i2c_client
 */
static int sn65dsi85_remove(struct i2c_client *c)
{
	struct sn65dsi85_device *ctx = i2c_get_clientdata(c);
	dev_dbg(&c->dev, "%s entry", __func__);

	if(ctx->dsi) {
		sn65dsi85_detach_dsi(ctx);
	}

	drm_bridge_remove(&ctx->bridge);
	
	dev_dbg(&c->dev, "%s exit", __func__);
	return 0;
}

/*
 * Device-Tree declaration 
 */
static const struct of_device_id sn65dsi85_of_match[] = {
	{.compatible = "ti,dsi85" },
	{}
};
MODULE_DEVICE_TABLE(of, sn65dsi85_of_match);


/*
 * This is an I2C client driver module instead of a general module
 */
static const struct i2c_device_id sn65dsi85_i2c_id[] = {
	{ "ti,dsi85", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sn65dsi85_i2c_id);

static struct i2c_driver sn65dsi85_i2c_driver = {
	.driver = {
                .of_match_table = of_match_ptr(sn65dsi85_of_match),
		.owner = THIS_MODULE,
		.name = SN65DSI85_MODULE_NAME,
	},
	.probe = sn65dsi85_probe,
	.remove = sn65dsi85_remove,
	.id_table = sn65dsi85_i2c_id,
};

module_i2c_driver(sn65dsi85_i2c_driver);


MODULE_AUTHOR("Konrad Anton <konrad.anton@awinia.de>");
MODULE_DESCRIPTION("DRM Driver for SN65DSI85 MIPI-DSI-to-LVDS converter");
MODULE_LICENSE("GPL");

/* EOF */
