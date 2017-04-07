#include "sn65dsi85-registers.h"

#define LVDS_CLK_FROM_DSI_CLK 1 // ?

/* Register addresses */
#define DSI85_SOFT_RESET		    0x09
#define DSI85_CORE_PLL			    0x0A
#define DSI85_PLL_DIV			    0x0B
#define DSI85_PLL_EN			    0x0D
#define DSI85_DSI_CFG			    0x10
#define DSI85_DSI_EQ			    0x11
#define DSI85_CHA_DSI_CLK_RNG       0x12
#define DSI85_CHB_DSI_CLK_RNG       0x13
#define DSI85_LVDS_MODE			    0x18
#define DSI85_LVDS_SIGN			    0x19
#define DSI85_LVDS_TERM			    0x1A
#define DSI85_CHA_LINE_LEN_LO		0x20
#define DSI85_CHA_LINE_LEN_HI		0x21
#define DSI85_CHB_LINE_LEN_LO		0x22
#define DSI85_CHB_LINE_LEN_HI		0x23
#define DSI85_CHA_VERT_LINES_LO		0x24
#define DSI85_CHA_VERT_LINES_HI		0x25
#define DSI85_CHB_VERT_LINES_LO		0x26
#define DSI85_CHB_VERT_LINES_HI		0x27
#define DSI85_CHA_SYNC_DELAY_LO		0x28
#define DSI85_CHA_SYNC_DELAY_HI		0x29
#define DSI85_CHB_SYNC_DELAY_LO		0x2A
#define DSI85_CHB_SYNC_DELAY_HI		0x2B
#define DSI85_CHA_HSYNC_WIDTH_LO	0x2C
#define DSI85_CHA_HSYNC_WIDTH_HI	0x2D
#define DSI85_CHB_HSYNC_WIDTH_LO	0x2E
#define DSI85_CHB_HSYNC_WIDTH_HI	0x2F
#define DSI85_CHA_VSYNC_WIDTH_LO	0x30
#define DSI85_CHA_VSYNC_WIDTH_HI	0x31
#define DSI85_CHB_VSYNC_WIDTH_LO	0x32
#define DSI85_CHB_VSYNC_WIDTH_HI	0x33
#define DSI85_CHA_HORZ_BACKPORCH	0x34
#define DSI85_CHB_HORZ_BACKPORCH	0x35
#define DSI85_CHA_VERT_BACKPORCH	0x36
#define DSI85_CHB_VERT_BACKPORCH	0x37
#define DSI85_CHA_HORZ_FRONTPORCH	0x38
#define DSI85_CHB_HORZ_FRONTPORCH	0x39
#define DSI85_CHA_VERT_FRONTPORCH	0x3A
#define DSI85_CHB_VERT_FRONTPORCH	0x3B

#if 0
/**
 * dsi85_config - Configure dsi85
 *
 * Initial configuration for dsi85 configuration registers
 */
static void dsi85_config(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO "Now Configuring\n");
	
	struct dsi85_data *dsi85_d = dev_get_drvdata(&dssdev->dev);
	struct i2c_client *dsi85_i2c_client = dsi85_d->dsi85_i2c_client;
    u8 val = 0;

	/* Soft reset and disable PLL */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_SOFT_RESET, 0x01);
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_EN, 0x00);

#if LVDS_CLK_FROM_DSI_CLK
    val = 0x1;
#endif

	/* user external clock reference with no muliplier */
    if (dssdev->panel.timings.pixel_clock <= 37500)
    {
        // Do nothing.
    }
    else if (dssdev->panel.timings.pixel_clock <= 62500)
    {
        val |= (0x01 << 1);
    }
    else if (dssdev->panel.timings.pixel_clock <= 87500)
    {
        val |= (0x02 << 1);
    }
    else if (dssdev->panel.timings.pixel_clock <= 112500)
    {
        val |= (0x03 << 1);
    }
    else if (dssdev->panel.timings.pixel_clock <= 137500)
    {
        val |= (0x04 << 1);
    }
    else
    {
        val |= (0x05 << 1);
    }
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CORE_PLL, val);
#if LVDS_CLK_FROM_DSI_CLK
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_DIV, 0x10);  // Divide DSI_CLK by 3.
#else
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_DIV, 0x00);  // Multiply REFCLK by 1.
#endif

	/* four DSI lanes with single channel*/
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_DSI_CFG, 0x20);
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_DSI_EQ, 0x00);

    /* set DSI clock range */
    i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_DSI_CLK_RNG, (dssdev->panel.timings.pixel_clock * 3 / 5000));
    i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_DSI_CLK_RNG, (dssdev->panel.timings.pixel_clock * 3 / 5000));

	/* set LVDS for single channel, 24 bit mode, HS/VS low, DE high */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_MODE, 0x7F);

	/* set LVDS 200 Ohm termination and max differential swing voltage */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_SIGN, 0x00);
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_TERM, 0x00);

	/* x resolution high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_LINE_LEN_LO, 		((dssdev->panel.timings.x_res) & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_LINE_LEN_HI, 		((dssdev->panel.timings.x_res) & 0xFF00)>>8);

	/* x resolution high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_LINE_LEN_LO, 		(dssdev->panel.timings.x_res & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_LINE_LEN_HI, 		(dssdev->panel.timings.x_res & 0xFF00)>>8);

	/* y resolution high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_LINES_LO, 		(dssdev->panel.timings.y_res & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_LINES_HI, 		(dssdev->panel.timings.y_res & 0xFF00)>>8);

	/* y resolution high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_LINES_LO, 		(dssdev->panel.timings.y_res & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_LINES_HI, 		(dssdev->panel.timings.y_res & 0xFF00)>>8);

	/* SYNC delay high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, 		DSI85_CHA_SYNC_DELAY_LO, 0x00);
	i2c_smbus_write_byte_data(dsi85_i2c_client, 		DSI85_CHA_SYNC_DELAY_HI, 0x02);

	/* SYNC delay high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, 		DSI85_CHB_SYNC_DELAY_LO, 0x00);
	i2c_smbus_write_byte_data(dsi85_i2c_client, 		DSI85_CHB_SYNC_DELAY_HI, 0x02);

	/* HSYNC width high/low for channel A */
/*	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_LO, 		(dssdev->panel.timings.hsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_HI, 		(dssdev->panel.timings.hsw & 0xFF00)>>8);	*/
    i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_LO, 		(lvds_timings.hsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_HI, 		(lvds_timings.hsw & 0xFF00)>>8);

	/* HSYNC width high/low for channel B */
/*	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_LO, 		(dssdev->panel.timings.hsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_HI, 		(dssdev->panel.timings.hsw & 0xFF00)>>8); */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_LO, 		(lvds_timings.hsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_HI, 		(lvds_timings.hsw & 0xFF00)>>8);

	/* VSYNC width high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VSYNC_WIDTH_LO, 		(lvds_timings.vsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VSYNC_WIDTH_HI, 		(lvds_timings.vsw & 0xFF00)>>8);

	/* VSYNC width high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VSYNC_WIDTH_LO, 		(lvds_timings.vsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VSYNC_WIDTH_HI, 		(lvds_timings.vsw & 0xFF00)>>8);

	/* Horizontal BackPorch for channel A */
//	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HORZ_BACKPORCH, //		(dssdev->panel.timings.hbp & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HORZ_BACKPORCH, 		(lvds_timings.hbp & 0x00FF));

	/* Horizontal BackPorch for channel B */
//	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HORZ_BACKPORCH, //  	(dssdev->panel.timings.hbp & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HORZ_BACKPORCH, 		(lvds_timings.hbp & 0x00FF));

	/* Vertical BackPorch for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_BACKPORCH, 		(lvds_timings.vbp & 0x00FF)); 

	/* Vertical BackPorch for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_BACKPORCH, 		(lvds_timings.vbp & 0x00FF));

	/* Horizontal FrontPorch for channel A */
//	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HORZ_FRONTPORCH, //		(dssdev->panel.timings.hfp & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HORZ_FRONTPORCH, 		(lvds_timings.hfp & 0x00FF));

	/* Horizontal FrontPorch for channel B */
//	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HORZ_FRONTPORCH, //		(dssdev->panel.timings.hfp & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HORZ_FRONTPORCH, 		(lvds_timings.hfp & 0x00FF));

	/* Vertical FrontPorch for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_FRONTPORCH, 		(lvds_timings.vbp & 0x00FF));

	/* Vertical FrontPorch for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_FRONTPORCH, 		(lvds_timings.vbp & 0x00FF));

	/* Soft reset and enable PLL */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_SOFT_RESET, 0x01);
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_EN, 0x01);
	return;
}
#endif // 0

void sn65dsi85_show_testpattern(struct i2c_client *dsi85_i2c_client, struct sn65dsi85_video_timings *timings,
		       struct dsi85_lvds_timings *lvds_timings)
{
	u8 val = 0;
	printk(KERN_INFO "Now Configuring Test Pattern\n");
	
    
	/* Soft reset and disable PLL */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_SOFT_RESET, 0x01);
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_EN, 0x00);

#if LVDS_CLK_FROM_DSI_CLK
    val = 0x1;
#endif

	/* user external clock reference with no muliplier */
    if (timings->pixel_clock <= 37500)
    {
        // Do nothing.
    }
    else if (timings->pixel_clock <= 62500)
    {
        val |= (0x01 << 1);
    }
    else if (timings->pixel_clock <= 87500)
    {
        val |= (0x02 << 1);
    }
    else if (timings->pixel_clock <= 112500)
    {
        val |= (0x03 << 1);
    }
    else if (timings->pixel_clock <= 137500)
    {
        val |= (0x04 << 1);
    }
    else
    {
        val |= (0x05 << 1);
    }
    
    //LADLDL
    printk(KERN_INFO "Pixel CLK value was %d\n",val);
    
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CORE_PLL, val);
#if LVDS_CLK_FROM_DSI_CLK
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_DIV, 0x10);  // Divide DSI_CLK by 3.
#else
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_DIV, 0x00);  // Multiply REFCLK by 1.
#endif


	//LADLDL PLL enable after address A and B configured
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_EN, 0x01);



	/* four DSI lanes with single channel*/
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_DSI_CFG, 0x20);
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_DSI_EQ, 0x00);

	/* set DSI clock range */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_DSI_CLK_RNG, (timings->pixel_clock * 3 / 5000));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_DSI_CLK_RNG, (timings->pixel_clock * 3 / 5000));

	/* set LVDS for single channel, 24 bit mode, HS/VS low, DE high */
	//i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_MODE, 0x7F);
	/*LADLD set LVDS for single channel, 24 bit mode, HS/VS low, DE high */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_MODE, 0x60);

	/* set LVDS 200 Ohm termination and max differential swing voltage */
	//LADLDL
	//i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_SIGN, 0x00);
	//i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_TERM, 0x00);

	/* x resolution high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_LINE_LEN_LO, 		((timings->x_res) & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_LINE_LEN_HI, 		((timings->x_res) & 0xFF00)>>8);

	/* x resolution high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_LINE_LEN_LO, 		(timings->x_res & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_LINE_LEN_HI, 		(timings->x_res & 0xFF00)>>8);

	/* y resolution high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_LINES_LO, 		(timings->y_res & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_LINES_HI, 		(timings->y_res & 0xFF00)>>8);

	/* y resolution high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_LINES_LO, 		(timings->y_res & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_LINES_HI, 		(timings->y_res & 0xFF00)>>8);

	/* SYNC delay high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, 		DSI85_CHA_SYNC_DELAY_LO, 0x00);
	i2c_smbus_write_byte_data(dsi85_i2c_client, 		DSI85_CHA_SYNC_DELAY_HI, 0x02);

	/* SYNC delay high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, 		DSI85_CHB_SYNC_DELAY_LO, 0x00);
	i2c_smbus_write_byte_data(dsi85_i2c_client, 		DSI85_CHB_SYNC_DELAY_HI, 0x02);

	/* HSYNC width high/low for channel A */
/*	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_LO, 		(timings->hsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_HI, 		(timings->hsw & 0xFF00)>>8);	*/
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_LO, 		(lvds_timings->hsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_HI, 		(lvds_timings->hsw & 0xFF00)>>8);

	/* HSYNC width high/low for channel B */
/*	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_LO, 		(timings->hsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_HI, 		(timings->hsw & 0xFF00)>>8); */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_LO, 		(lvds_timings->hsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_HI, 		(lvds_timings->hsw & 0xFF00)>>8);

	/* VSYNC width high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VSYNC_WIDTH_LO, 		(lvds_timings->vsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VSYNC_WIDTH_HI, 		(lvds_timings->vsw & 0xFF00)>>8);

	/* VSYNC width high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VSYNC_WIDTH_LO, 		(lvds_timings->vsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VSYNC_WIDTH_HI, 		(lvds_timings->vsw & 0xFF00)>>8);

	/* Horizontal BackPorch for channel A */
//	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HORZ_BACKPORCH, //		(timings->hbp & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HORZ_BACKPORCH, 		(lvds_timings->hbp & 0x00FF));

	/* Horizontal BackPorch for channel B */
//	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HORZ_BACKPORCH, //  	(timings->hbp & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HORZ_BACKPORCH, 		(lvds_timings->hbp & 0x00FF));

	/* Vertical BackPorch for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_BACKPORCH, 		(lvds_timings->vbp & 0x00FF)); 

	/* Vertical BackPorch for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_BACKPORCH, 		(lvds_timings->vbp & 0x00FF));

	/* Horizontal FrontPorch for channel A */
//	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HORZ_FRONTPORCH, //		(timings->hfp & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HORZ_FRONTPORCH, 		(lvds_timings->hfp & 0x00FF));

	/* Horizontal FrontPorch for channel B */
//	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HORZ_FRONTPORCH, //		(timings->hfp & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HORZ_FRONTPORCH, 		(lvds_timings->hfp & 0x00FF));

	/* Vertical FrontPorch for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_FRONTPORCH, 		(lvds_timings->vbp & 0x00FF));

	/* Vertical FrontPorch for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_FRONTPORCH, 		(lvds_timings->vbp & 0x00FF));

	//Test Pattern
	i2c_smbus_write_byte_data(dsi85_i2c_client, 0x3C, 0x11);

	//LADLDL
	/* Soft reset and enable PLL */
	//i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_SOFT_RESET, 0x01);
	//i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_EN, 0x01);
	return;
}

