#ifndef SN65DSI85_REGISTERS_H_INCLUDED
#define SN65DSI85_REGISTERS_H_INCLUDED

#include <linux/types.h>
#include <linux/i2c.h>

struct sn65dsi85_video_timings {
	/* Unit: pixels */
	u16 x_res;
	/* Unit: pixels */
	u16 y_res;
	/* Unit: Hz */
	u32 pixel_clock;
	/* Unit: pixel clocks */
	u16 hsw;	/* Horizontal synchronization pulse width */
	/* Unit: pixel clocks */
	u16 hfp;	/* Horizontal front porch */
	/* Unit: pixel clocks */
	u16 hbp;	/* Horizontal back porch */
	/* Unit: line clocks */
	u16 vsw;	/* Vertical synchronization pulse width */
	/* Unit: line clocks */
	u16 vfp;	/* Vertical front porch */
	/* Unit: line clocks */
	u16 vbp;	/* Vertical back porch */

};

struct dsi85_lvds_timings {
    u16 hfp;
    u16 hsw;
    u16 hbp;
    u16 vfp;
    u16 vsw;
    u16 vbp;
};


void sn65dsi85_show_testpattern(struct i2c_client *dsi85_i2c_client, struct sn65dsi85_video_timings *timings,
		    struct dsi85_lvds_timings *lvds_timings);


#endif
