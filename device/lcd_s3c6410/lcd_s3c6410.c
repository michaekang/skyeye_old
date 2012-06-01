/* Copyright (C) 2007-2008 The Android Open Source Project
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
*/
//#include "android/android.h"
//#include "android/utils/debug.h"
//#include "android/utils/duff.h"
#include <skyeye_types.h>
#include <skyeye_sched.h>
#include <skyeye_signal.h>
#include <skyeye_class.h>
#include <skyeye_interface.h>
#include <skyeye_obj.h>
#include <skyeye_mm.h>
#include <skyeye_attr.h>
#include <memory_space.h>
#include <skyeye_lcd_intf.h>
#include <skyeye_lcd_surface.h>
//#define DEBUG
#include <skyeye_log.h>

#include "lcd_s3c6410.h"
#include "regs-fb.h"
#include "regs-fb-v4.h"

/* These values *must* match the platform definitions found under
 * hardware/libhardware/include/hardware/hardware.h
 */
enum {
    HAL_PIXEL_FORMAT_RGBA_8888          = 1,
    HAL_PIXEL_FORMAT_RGBX_8888          = 2,
    HAL_PIXEL_FORMAT_RGB_888            = 3,
    HAL_PIXEL_FORMAT_RGB_565            = 4,
    HAL_PIXEL_FORMAT_BGRA_8888          = 5,
    HAL_PIXEL_FORMAT_RGBA_5551          = 6,
    HAL_PIXEL_FORMAT_RGBA_4444          = 7,
};

enum {
    FB_GET_WIDTH        = 0x00,
    FB_GET_HEIGHT       = 0x04,
    FB_INT_STATUS       = 0x08,
    FB_INT_ENABLE       = 0x0c,
    FB_SET_BASE         = 0x10,
    FB_SET_ROTATION     = 0x14,
    FB_SET_BLANK        = 0x18,
    FB_GET_PHYS_WIDTH   = 0x1c,
    FB_GET_PHYS_HEIGHT  = 0x20,
    FB_GET_FORMAT       = 0x24,

    FB_INT_VSYNC             = 1U << 0,
    FB_INT_BASE_UPDATE_DONE  = 1U << 1
};

/* Type used to record a mapping from display surface pixel format to
 * HAL pixel format */
typedef struct {
    int    pixel_format; /* HAL pixel format */
    uint8_t bits;
    uint8_t bytes;
    uint32_t rmask, gmask, bmask, amask;
} FbConfig;
static int s3c6410_fb_get_bytes_per_pixel(struct s3c6410_fb_device *s)
{
#if 0
    if (s->fb->pixel_format < 0) {
//        (void) s3c6410_fb_get_pixel_format(s);
    }
    return s->fb->bytes_per_pixel;
#endif
    return 0;
}

static int
pixels_to_mm(int  pixels, int dpi)
{
    /* dpi = dots / inch
    ** inch = dots / dpi
    ** mm / 25.4 = dots / dpi
    ** mm = (dots * 25.4)/dpi
    */
    return (int)(0.5 + 25.4 * pixels  / dpi);
}


#define  STATS  0

#if STATS
static int   stats_counter;
static long  stats_total;
static int   stats_full_updates;
static long  stats_total_full_updates;
#endif

/* This structure is used to hold the inputs for
 * compute_fb_update_rect_linear below.
 * This corresponds to the source framebuffer and destination
 * surface pixel buffers.
 */
typedef struct {
    int            width;
    int            height;
    int            bytes_per_pixel;
    const uint8_t* src_pixels;
    int            src_pitch;
    uint8_t*       dst_pixels;
    int            dst_pitch;
} FbUpdateState;
static exception_t s3c6410_fb_read(conf_object_t *opaque, generic_address_t offset, void* buf, size_t count)
{
	exception_t ret = No_exp;
	struct s3c6410_fb_device *dev = opaque->obj;
	fb_reg_t* regs = dev->regs;
	DBG("In %s, offset=0x%x\n", __FUNCTION__, offset);
	switch(offset) {
        case VIDCON0:
		*(uint32_t*)buf = regs->vidcon[0];
		return ret;

        case VIDCON1:
		*(uint32_t*)buf = regs->vidcon[1];
		return ret;

        case VIDCON2:
		*(uint32_t*)buf = regs->vidcon[2];
		return ret;
	case WINCON(0):
		*(uint32_t*)buf = regs->wincon[0];
		break;
	case WINCON(1):
		*(uint32_t*)buf = regs->wincon[0];
		break;
	case WINCON(2):
		*(uint32_t*)buf = regs->wincon[2];
		break;
	case WINCON(3):
		*(uint32_t*)buf = regs->wincon[3];
		break;
	case VIDOSD_BASE:
		*(uint32_t*)buf = regs->vidosd[0][0];
		break;
	case (VIDOSD_BASE + 4):
		*(uint32_t*)buf = regs->vidosd[0][1];
		break;
	case (VIDOSD_BASE + 8):
		*(uint32_t*)buf = regs->vidosd[0][2];
		DBG("In %s, windows size is %d\n", __FUNCTION__, data & 0xFFFFFF);
		break;
	case (VIDOSD_BASE + 0x10):
		*(uint32_t*)buf = regs->vidosd[1][0];
		break;
	case (VIDOSD_BASE + 0x14):
		*(uint32_t*)buf = regs->vidosd[1][1];
		break;
	case (VIDOSD_BASE + 0x18):
		*(uint32_t*)buf = regs->vidosd[1][2];
		break;
	case (VIDOSD_BASE + 0x1c):
		*(uint32_t*)buf = regs->vidosd1d;
		break;
	case (VIDOSD_BASE + 0x20):
		*(uint32_t*)buf = regs->vidosd[2][0];
		break;
	case (VIDOSD_BASE + 0x24):
		*(uint32_t*)buf = regs->vidosd[2][1];
		break;
	case (VIDOSD_BASE + 0x28):
		*(uint32_t*)buf = regs->vidosd[2][2];
		break;
	case (VIDOSD_BASE + 0x2c):
		*(uint32_t*)buf = regs->vidosd2d;
		break;
	case (VIDOSD_BASE + 0x30):
		*(uint32_t*)buf = regs->vidosd[3][0];
		break;
	case (VIDOSD_BASE + 0x34):
		*(uint32_t*)buf = regs->vidosd[3][1];
		break;
	case (VIDOSD_BASE + 0x38):
		*(uint32_t*)buf = regs->vidosd[3][2];
		break;
	case (VIDOSD_BASE + 0x40):
		*(uint32_t*)buf = regs->vidosd[4][0];
		break;
	case (VIDOSD_BASE + 0x44):
		*(uint32_t*)buf = regs->vidosd[4][1];
		break;
	case (VIDOSD_BASE + 0x48):
		*(uint32_t*)buf = regs->vidosd[4][2];
		break;
	case 0xa8:
		*(uint32_t*)buf = regs->vidw01add0b0;
		break;
	case 0xac:
		*(uint32_t*)buf = regs->vidw01add0b1;
		break;
	case 0xb0:
		*(uint32_t*)buf = regs->vidw02add0;
		break;
	case 0xb8:
		*(uint32_t*)buf = regs->vidw03add0;
		break;
	case 0xe8:
		*(uint32_t*)buf = regs->vidw03add1;
		break;
	case 0xe0:
		*(uint32_t*)buf = regs->vid02add1;
		break;
	case 0xd8:
		*(uint32_t*)buf = regs->vidw01add1b0;
		break;
	case 0xdc:
		*(uint32_t*)buf = regs->vidw01add1b1;
		break;
	case 0x104:
		*(uint32_t*)buf = regs->vidw01add2;
		break;
	case 0x108:
		*(uint32_t*)buf = regs->vidw02add2;
		break;
	case 0x10c:
		*(uint32_t*)buf = regs->vidw03add2;
		break;
	case 0x130:
		*(uint32_t*)buf = regs->vidintcon0;
		break;
	case 0x134:
		*(uint32_t*)buf = regs->vidintcon1;
		break;
	
	case 0x170:
		*(uint32_t*)buf = regs->dithmode;
		break;
	case 0x1a0:
		*(uint32_t*)buf = regs->wpalcon;
		break;
	default:
		printf("Can not read the register at 0x%x\n", offset);
		*(uint32_t*)buf = 0;
		//return 0;
	}
	return ret;
}

//static void s3c6410_fb_write(void *opaque, target_phys_addr_t offset,
 //                       uint32_t val)
static exception_t s3c6410_fb_write(conf_object_t *opaque, generic_address_t offset, uint32_t* buf, size_t count)
{
	struct s3c6410_fb_device *dev = opaque->obj;
	fb_reg_t* regs = dev->regs;
	lcd_surface_t* surface = dev->surface;

	uint32_t data = *(uint32_t*)buf;
	DBG("In %s, offset=0x%x, data=0x%x\n", __FUNCTION__, offset, data);
	lcd_control_intf* lcd_ctrl = dev->lcd_ctrl->u.ptr;
	if(lcd_ctrl == NULL){
		skyeye_log(Error_log, __FUNCTION__, "Need to set the lcd panel\n");
		return Invarg_exp;
	}
	switch(offset) {
	case VIDCON0:
		regs->vidcon[0] = data;
		break;

        case VIDCON1: 
		regs->vidcon[1] = data;
		break;
	case 0xC: /* unknown address on manual ,just stop complaining from kernel */
		break;
	case VIDTCON0:
		regs->vidtcon[0] = data;
		break;
	case VIDTCON1:
		regs->vidtcon[1] = data;
		break;
	case VIDTCON2:
		regs->vidtcon[2] = data;
		int vertical = ((data >> 11) & 0x7FF);
		int horizontal = data & 0x7FF;
		DBG("In %s, vertical = %d, hor = %d\n", __FUNCTION__, vertical, horizontal);
		surface->width = horizontal + 1;
		surface->height = vertical + 1;
		break;
	case WINCON(0):
		regs->wincon[0] = data;
		int bpp = (data & WINCON0_BPPMODE_MASK) ;
		DBG("In %s, bpp=%d\n", __FUNCTION__, bpp);
		if(bpp == WINCON0_BPPMODE_16BPP_565){
			surface->depth = 16;
		}
		else{
			fprintf(stderr, "Wrong bpp in %s\n", __FUNCTION__);
		}
		if(data & WINCONx_ENWIN){
			/* Enable the window */
			static int done = 0;
			if(!done){
				lcd_ctrl->lcd_open(lcd_ctrl->conf_obj, dev->surface);	
				done = 1;
			}
			
		}
		else{
		/* Disable the window */
		}

		break;
	case WINCON(1):
		regs->wincon[1] = data;
		break;
	case WINCON(2):
		regs->wincon[2] = data;
		break;
	case WINCON(3):
		regs->wincon[3] = data;
		break;
	case WINCON(4):
		regs->wincon[4] = data;
		break;

	case VIDOSD_BASE:
		regs->vidosd[0][0] = data;
		DBG("In %s,left_top_x=%d, left_top_y=%d\n", __FUNCTION__, ((data >> 11) & 0x7ff), data & 0x7FF);
		break;
	case (VIDOSD_BASE + 4):
		regs->vidosd[0][1] = data;
		DBG("In %s,right_bot_x=%d, right_bot_y=%d\n", __FUNCTION__, ((data >> 11) & 0x7ff), data & 0x7FF);
		break;
	case (VIDOSD_BASE + 8):
		regs->vidosd[0][2] = data;
		DBG("In %s, windows size is %d\n", __FUNCTION__, data & 0xFFFFFF);
		break;
	case (VIDOSD_BASE + 0x10):
		regs->vidosd[1][0] = data;
		break;
	case (VIDOSD_BASE + 0x14):
		regs->vidosd[1][1] = data;
		break;
	case (VIDOSD_BASE + 0x18):
		regs->vidosd[1][2] = data;
		break;
	case (VIDOSD_BASE + 0x1c):
		regs->vidosd1d = data;
                break;
	case (VIDOSD_BASE + 0x20):
		regs->vidosd[2][0] = data;
		break;
	case (VIDOSD_BASE + 0x24):
		regs->vidosd[2][1] = data;
		break;
	case (VIDOSD_BASE + 0x28):
		regs->vidosd[2][2] = data;
		break;
	case (VIDOSD_BASE + 0x2c):
		regs->vidosd2d = data;
		break;
	case (VIDOSD_BASE + 0x30):
		regs->vidosd[3][0] = data;
		break;
	case (VIDOSD_BASE + 0x34):
		regs->vidosd[3][1] = data;
		break;
	case (VIDOSD_BASE + 0x38):
		regs->vidosd[3][2] = data;
		break;
	case (VIDOSD_BASE + 0x40):
		regs->vidosd[4][0] = data;
		break;
	case (VIDOSD_BASE + 0x44):
		regs->vidosd[4][1] = data;
		break;
	case (VIDOSD_BASE + 0x48):
		regs->vidosd[4][2] = data;
		break;
	case 0xa0:
		DBG("In %s, windows0 buf start=0x%x", __FUNCTION__, data);
		regs->vidw00add0b0 = data;
		surface->lcd_addr_begin = data;
		surface->need_update = 1;
		lcd_ctrl->lcd_update(lcd_ctrl->conf_obj, surface);	
		break;
	case 0xa4:
		regs->vidw00add0b1 = data;
		break;
	case 0xa8:
		regs->vidw01add0b0 = data;
		break;
	case 0xac:
		regs->vidw01add0b1 = data;
		break;
	case 0xb0:
		regs->vidw02add0 = data;
		break;
	case 0xb8:
		regs->vidw03add0 = data;
		break;
	case 0xe8:
		regs->vidw03add1 = data;
		break;
	case 0xe0:
		regs->vid02add1 = data;
		break;

	case 0xd0:
		DBG("In %s, windows0 buf end=0x%x", __FUNCTION__, data);
		regs->vidw00add1b0 = data;
		surface->lcd_addr_end = data;
		break;
	case 0xd4:
		regs->vidw00add1b1 = data;
		break;
	case 0xd8:
		regs->vidw01add1b0 = data;
		break;
	case 0xdc:
		regs->vidw01add1b1 = data;
		break;

	case 0x100:
		DBG("In %s, windows 0 buffer size is 0x%x\n", __FUNCTION__, data);
		regs->vidw_buf_size[0] = data;
		break;
	case 0x104:
                regs->vidw01add2 = data;
                break;
        case 0x108:
		regs->vidw02add2 = data;
                break;
        case 0x10c:
		regs->vidw03add2 = data;
                break;
        case 0x130:
		regs->vidintcon0 = data;
		#if 0
		printf("In %s, vidintcon0=0x%x\n", __FUNCTION__, data);
		if(data & 0x1)
			printf("In %s, Video Interrupt Enable\n", __FUNCTION__);
		if(data & 0x1000) /* Frame video interrupt */
			printf("In %s, Frame Video Interrupt Enable\n", __FUNCTION__);
		#endif
                break;
        case 0x134:
		/* W1C */
		regs->vidintcon1 &= ~data;
		if(dev->master != NULL && dev->master->lower_signal != NULL)
			dev->master->lower_signal(dev->master->conf_obj, dev->line_no);
                break;
	case 0x170:
		regs->dithmode = data;
		break;
	case 0x180:
		regs->winmap[0] = data;
		break;
	case 0x1a0:
                *(uint32_t*)buf = regs->wpalcon;
                break;

	default:
		if(offset >= 0x140 && offset <= 0x15c){
			regs->wkeycon[((offset - 0x140) / 4)] = data;
			break;
		}

		printf("Can not write the register at 0x%x\n", offset);
		return Invarg_exp;
            //cpu_abort (cpu_single_env, "s3c6410_fb_write: Bad offset %x\n", offset);
	}
	return No_exp;
}
#if 0
static void timer_update(conf_object_t *dev){
	if(dev->matser->conf_obj != NULL && dev->lcd_ctrl->u.ptr != NULL)
		lcd_control_intf* lcd_ctrl = dev->lcd_ctrl->u.ptr;
		gtk_lcd_update(lcd_ctrl->conf_obj);
		if(regs->vidintcon0 & 0x1){
			if(regs->vidintcon0 & 0x1000)
				regs->vidintcon1 |= 0x2 /* trigger frame sync interrupt */ 
			if(regs->vidintcon0 & 0x2)
				regs->vidintcon1 |= 0x1 /* trigger FIFO empty interrupt */ 
			dev->master->raise_signal(dev->master->conf_obj, dev->line_no);
		}
	}
}
#endif
static int refresh_trigger(conf_object_t* opaque){
	struct s3c6410_fb_device *dev = opaque->obj;
	fb_reg_t* regs = dev->regs;
	if(regs->vidintcon0 & 0x1 == 0)
		return 0;
	/* check if we are still at interrupt handler */
	if(regs->vidintcon1 & 0x3 != 0)
		return 0;
	if(regs->vidintcon0 & 0x1000)
		regs->vidintcon1 |= 0x2; /* trigger frame sync interrupt */ 
	if(regs->vidintcon0 & 0x2)
		regs->vidintcon1 |= 0x1; /* trigger FIFO empty interrupt */ 

	if(dev->master == NULL || dev->master->raise_signal == NULL)
		return 0;
	/* send the signal to the vic */
	if(regs->vidintcon1 & 0x3)
		dev->master->raise_signal(dev->master->conf_obj, dev->line_no);
	
	return 0;
}
static conf_object_t* new_s3c6410_lcd(char* obj_name){
	s3c6410_fb_device* dev = skyeye_mm_zero(sizeof(s3c6410_fb_device));
	dev->obj = new_conf_object(obj_name, dev);
	fb_state_t* state =  skyeye_mm_zero(sizeof(fb_state_t));
	dev->state = state;
	fb_reg_t* regs = skyeye_mm_zero(sizeof(fb_reg_t));
	regs->vidintcon0 = 0x03f00000;
	dev->regs = regs;
	lcd_surface_t* surface = skyeye_mm_zero(sizeof(lcd_surface_t));
	dev->surface = surface;
	/* Register io function to the object */
	memory_space_intf* io_memory = skyeye_mm_zero(sizeof(memory_space_intf));
	io_memory->conf_obj = dev->obj;
	io_memory->read = s3c6410_fb_read;
	io_memory->write = s3c6410_fb_write;
	SKY_register_interface(io_memory, obj_name, MEMORY_SPACE_INTF_NAME);

	dev->lcd_ctrl = make_new_attr(Val_ptr);
	SKY_register_attr(dev->obj, "lcd_ctrl_0", dev->lcd_ctrl);

	//int timer_id;
	//create_thread_scheduler(5000, Periodic_sched, timer_update, dev->lcd_ctrl, &timer_id);

	general_signal_intf* lcd_signal = skyeye_mm_zero(sizeof(general_signal_intf));
	lcd_signal->conf_obj = NULL;
	lcd_signal->raise_signal = NULL;
	lcd_signal->lower_signal = NULL;
	dev->master = lcd_signal;
	dev->line_no = 30; /* Frame sync */
	SKY_register_interface(lcd_signal, obj_name, GENERAL_SIGNAL_INTF_NAME);

	/* Get notified when gtk refresh finished */
	simple_signal_intf* refresh_signal = skyeye_mm_zero(sizeof(simple_signal_intf));
	refresh_signal->conf_obj = dev->obj;
	refresh_signal->trigger = refresh_trigger;
	SKY_register_interface(refresh_signal, obj_name, SIMPLE_SIGNAL_INTF_NAME);

	return dev->obj;
}
void free_s3c6410_lcd(conf_object_t* dev){
	
}
void init_s3c6410_lcd(){
	static skyeye_class_t class_data = {
		.class_name = "s3c6410_lcd",
		.class_desc = "s3c6410 lcd",
		.new_instance = new_s3c6410_lcd,
		.free_instance = free_s3c6410_lcd,
		.get_attr = NULL,
		.set_attr = NULL
	};
	
	SKY_register_class(class_data.class_name, &class_data);
}
