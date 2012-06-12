/* Copyright (C) 
* xq2537@gmail.com
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
* 
*/
/**
* @file lcd_sdl.c
* @brief The implementation of system controller
* @author xq2537@gmail.com
* @version 78.77
*/
#include "android/android.h"
#include "android/utils/debug.h"
#include "android/utils/duff.h"
#include "console.h"

#include <skyeye_types.h>
#include <skyeye_sched.h>
#include <skyeye_signal.h>
#include <skyeye_class.h>
#include <skyeye_lcd_intf.h>
#include <skyeye_keypad_intf.h>
#include <skyeye_interface.h>
#include <skyeye_obj.h>
#include <skyeye_mm.h> 
#include <skyeye_ram.h> 
#include <memory_space.h>
#include <skyeye_device.h>
#define DEBUG
#include <skyeye_log.h>
#include <skyeye_android_intf.h>

#include "lcd_sdl.h"

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


/* Type used to record a mapping from display surface pixel format to
 * HAL pixel format */
typedef struct {
    int    pixel_format; /* HAL pixel format */
    uint8_t bits;
    uint8_t bytes;
    uint32_t rmask, gmask, bmask, amask;
} FbConfig;

/* Return the pixel format of the current framebuffer, based on
 * the current display surface's pixel format.
 *
 * Note that you should not call this function from the device initialization
 * function, because the display surface will change format before the kernel
 * start.
 */
static int lcd_sdl_get_pixel_format(struct lcd_sdl_device *s)
{
    if (s->fb->pixel_format >= 0) {
        return s->fb->pixel_format;
    }
    static const FbConfig fb_configs[] = {
        { HAL_PIXEL_FORMAT_RGB_565, 16, 2, 0xf800, 0x7e0, 0x1f, 0x0 },
        { HAL_PIXEL_FORMAT_RGBX_8888, 32, 4, 0xff0000, 0xff00, 0xff, 0x0 },
        { HAL_PIXEL_FORMAT_RGBA_8888, 32, 4, 0xff0000, 0xff00, 0xff, 0xff000000 },
        { -1, }
    };

    /* Determine HAL pixel format value based on s->fb->ds */
    struct PixelFormat* pf = &s->fb->ds->surface->pf;
    if (VERBOSE_CHECK(init)) {
        printf("%s:%d: display surface,pixel format:\n", __FUNCTION__, __LINE__);
        printf("  bits/pixel:  %d\n", pf->bits_per_pixel);
        printf("  bytes/pixel: %d\n", pf->bytes_per_pixel);
        printf("  depth:       %d\n", pf->depth);
        printf("  red:         bits=%d mask=0x%x shift=%d max=0x%x\n",
            pf->rbits, pf->rmask, pf->rshift, pf->rmax);
        printf("  green:       bits=%d mask=0x%x shift=%d max=0x%x\n",
            pf->gbits, pf->gmask, pf->gshift, pf->gmax);
        printf("  blue:        bits=%d mask=0x%x shift=%d max=0x%x\n",
            pf->bbits, pf->bmask, pf->bshift, pf->bmax);
        printf("  alpha:       bits=%d mask=0x%x shift=%d max=0x%x\n",
            pf->abits, pf->amask, pf->ashift, pf->amax);
    }

    s->fb->bytes_per_pixel = pf->bytes_per_pixel;
    int nn;
    for (nn = 0; fb_configs[nn].pixel_format >= 0; nn++) {
        const FbConfig* fbc = &fb_configs[nn];
        if (pf->bits_per_pixel == fbc->bits &&
            pf->bytes_per_pixel == fbc->bytes &&
            pf->rmask == fbc->rmask &&
            pf->gmask == fbc->gmask &&
            pf->bmask == fbc->bmask &&
            pf->amask == fbc->amask) {
            /* We found it */
            s->fb->pixel_format = fbc->pixel_format;
            return s->fb->pixel_format;
        }
    }
    fprintf(stderr, "%s:%d: Unsupported display pixel format (depth=%d, bytespp=%d, bitspp=%d)\n",
                __FUNCTION__, __LINE__,
                pf->depth,
                pf->bytes_per_pixel,
                pf->bits_per_pixel);
    exit(1);
    return -1;
}

static int lcd_sdl_get_bytes_per_pixel(struct lcd_sdl_device *s)
{
    if (s->fb->pixel_format < 0) {
        (void) lcd_sdl_get_pixel_format(s);
    }
    return s->fb->bytes_per_pixel;
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

/* This structure is used to hold the outputs for
 * compute_fb_update_rect_linear below.
 * This corresponds to the smalled bounding rectangle of the
 * latest framebuffer update.
 */
typedef struct {
    int xmin, ymin, xmax, ymax;
} FbUpdateRect;

/* Determine the smallest bounding rectangle of pixels which changed
 * between the source (framebuffer) and destination (surface) pixel
 * buffers.
 *
 * Return 0 if there was no change, otherwise, populate '*rect'
 * and return 1.
 *
 * If 'dirty_base' is not 0, it is a physical address that will be
 * used to speed-up the check using the VGA dirty bits. In practice
 * this is only used if your kernel driver does not implement.
 *
 * This function assumes that the framebuffers are in linear memory.
 * This may change later when we want to support larger framebuffers
 * that exceed the max DMA aperture size though.
 */
static int
compute_fb_update_rect_linear(FbUpdateState*  fbs,
                              uint32_t        dirty_base,
                              FbUpdateRect*   rect)
{
    int  yy;
    int  width = fbs->width;
    const uint8_t* src_line = fbs->src_pixels;
    uint8_t*       dst_line = fbs->dst_pixels;
    uint32_t       dirty_addr = dirty_base;
    rect->xmin = rect->ymin = INT_MAX;
    rect->xmax = rect->ymax = INT_MIN;
    for (yy = 0; yy < fbs->height; yy++) {
        int xx1, xx2;
        /* If dirty_addr is != 0, then use it as a physical address to
         * use the VGA dirty bits table to speed up the detection of
         * changed pixels.
         */

	//modified by xiaoqiao
#if 0
        if (dirty_addr != 0) {
            int  dirty = 0;
            int  len   = fbs->src_pitch;

            while (len > 0) {
                int  len2 = TARGET_PAGE_SIZE - (dirty_addr & (TARGET_PAGE_SIZE-1));

                if (len2 > len)
                    len2 = len;

                dirty |= cpu_physical_memory_get_dirty(dirty_addr, VGA_DIRTY_FLAG);
                dirty_addr  += len2;
                len         -= len2;
            }

            if (!dirty) { /* this line was not modified, skip to next one */
                goto NEXT_LINE;
            }
        }
#endif

        /* Then compute actual bounds of the changed pixels, while
         * copying them from 'src' to 'dst'. This depends on the pixel depth.
         */
        switch (fbs->bytes_per_pixel) {
        case 2:
        {
            const uint16_t* src = (const uint16_t*) src_line;
            uint16_t*       dst = (uint16_t*) dst_line;

            xx1 = 0;
            DUFF4(width, {
                if (src[xx1] != dst[xx1])
                    break;
                xx1++;
            });
            if (xx1 == width) {
                break;
            }
            xx2 = width-1;
            DUFF4(xx2-xx1, {
                if (src[xx2] != dst[xx2])
                    break;
                xx2--;
            });
#if HOST_WORDS_BIGENDIAN
            /* Convert the guest little-endian pixels into big-endian ones */
            int xx = xx1;
            DUFF4(xx2-xx1+1,{
                unsigned   spix = src[xx];
                dst[xx] = (uint16_t)((spix << 8) | (spix >> 8));
                xx++;
            });
#else
            memcpy( dst+xx1, src+xx1, (xx2-xx1+1)*2 );
#endif
            break;
        }

        case 3:
        {
            xx1 = 0;
            DUFF4(width, {
                int xx = xx1*3;
                if (src_line[xx+0] != dst_line[xx+0] ||
                    src_line[xx+1] != dst_line[xx+1] ||
                    src_line[xx+2] != dst_line[xx+2]) {
                    break;
                }
                xx1 ++;
            });
            if (xx1 == width) {
                break;
            }
            xx2 = width-1;
            DUFF4(xx2-xx1,{
                int xx = xx2*3;
                if (src_line[xx+0] != dst_line[xx+0] ||
                    src_line[xx+1] != dst_line[xx+1] ||
                    src_line[xx+2] != dst_line[xx+2]) {
                    break;
                }
                xx2--;
            });
            memcpy( dst_line+xx1*3, src_line+xx1*3, (xx2-xx1+1)*3 );
            break;
        }

        case 4:
        {
            const uint32_t* src = (const uint32_t*) src_line;
            uint32_t*       dst = (uint32_t*) dst_line;

            xx1 = 0;
            DUFF4(width, {
                if (src[xx1] != dst[xx1]) {
                    break;
                }
                xx1++;
            });
            if (xx1 == width) {
                break;
            }
            xx2 = width-1;
            DUFF4(xx2-xx1,{
                if (src[xx2] != dst[xx2]) {
                    break;
                }
                xx2--;
            });
#if HOST_WORDS_BIGENDIAN
            /* Convert the guest little-endian pixels into big-endian ones */
            int xx = xx1;
            DUFF4(xx2-xx1+1,{
                uint32_t   spix = src[xx];
                spix = (spix << 16) | (spix >> 16);
                spix = ((spix << 8) & 0xff00ff00) | ((spix >> 8) & 0x00ff00ff);
                dst[xx] = spix;
                xx++;
            })
#else
            memcpy( dst+xx1, src+xx1, (xx2-xx1+1)*4 );
#endif
            break;
        }
        default:
            return 0;
        }
        /* Update bounds if pixels on this line were modified */
        if (xx1 < width) {
            if (xx1 < rect->xmin) rect->xmin = xx1;
            if (xx2 > rect->xmax) rect->xmax = xx2;
            if (yy < rect->ymin) rect->ymin = yy;
            if (yy > rect->ymax) rect->ymax = yy;
        }
    NEXT_LINE:
        src_line += fbs->src_pitch;
        dst_line += fbs->dst_pitch;
    }

    if (rect->ymin > rect->ymax) { /* nothing changed */
        return 0;
    }

//modified by xiaoqiao
    /* Always clear the dirty VGA bits */
#if 0
    cpu_physical_memory_reset_dirty(dirty_base + rect->ymin * fbs->src_pitch,
                                    dirty_base + (rect->ymax+1)* fbs->src_pitch,
                                    VGA_DIRTY_FLAG);
#endif
    return 1;
}


static void lcd_sdl_update_display(conf_object_t *opaque)
{
//    struct lcd_sdl_device *s = (lcd_sdl_device*)(opaque->obj);
    conf_object_t* obj = get_conf_obj("lcd_sdl_0");
    struct lcd_sdl_device *s = (lcd_sdl_device *)obj->obj;

//    printf("in %s\n",__func__);
    uint32_t base;
    uint8_t*  dst_line;
    uint8_t*  src_line;
    int full_update = 0;
    int  width, height, pitch;

    base = s->fb->fb_base;
    if(base == 0)
        return;

#if 0
    if((s->fb->int_enable & FB_INT_VSYNC) && !(s->fb->int_status & FB_INT_VSYNC)) {
        s->fb->int_status |= FB_INT_VSYNC;
		s->master->raise_signal(s->signal_target, s->line_no);
        //goldfish_device_set_irq(&s->fb->dev, 0, 1);
    }

    if(s->fb->need_update) {
        full_update = 1;
        if(s->fb->need_int) {
            s->fb->int_status |= FB_INT_BASE_UPDATE_DONE;
            if(s->fb->int_enable & FB_INT_BASE_UPDATE_DONE)
				s->master->raise_signal(s->signal_target,s->line_no);
                //goldfish_device_set_irq(&s->fb->dev, 0, 1);
        }
        s->fb->need_int = 0;
        s->fb->need_update = 0;
    }
#endif
    if(s->fb->need_update) {
        full_update = 1;
        s->fb->need_int = 0;
        s->fb->need_update = 0;
        if(s->master != NULL && s->master->trigger != NULL)
	    s->master->trigger(s->master->conf_obj);
    }

//modified by xiaoqiao
//    src_line  = qemu_get_ram_ptr( base );
    src_line  = get_dma_addr(base);

    dst_line  = s->fb->ds->surface->data;
    pitch     = s->fb->ds->surface->linesize;
    width     = s->fb->ds->surface->width;
    height    = s->fb->ds->surface->height;

    FbUpdateState  fbs;
    FbUpdateRect   rect;

    fbs.width      = width;
    fbs.height     = height;
    fbs.dst_pixels = dst_line;
    fbs.dst_pitch  = pitch;
    fbs.bytes_per_pixel = lcd_sdl_get_bytes_per_pixel(s);

    fbs.src_pixels = src_line;
    fbs.src_pitch  = width*s->fb->ds->surface->pf.bytes_per_pixel;


#if STATS
    if (full_update)
        stats_full_updates += 1;
    if (++stats_counter == 120) {
        stats_total               += stats_counter;
        stats_total_full_updates  += stats_full_updates;

        printf( "full update stats:  peak %.2f %%  total %.2f %%\n",
                stats_full_updates*100.0/stats_counter,
                stats_total_full_updates*100.0/stats_total );

        stats_counter      = 0;
        stats_full_updates = 0;
    }
#endif /* STATS */

    if (s->fb->blank)
    {
        memset( dst_line, 0, height*pitch );
        rect.xmin = 0;
        rect.ymin = 0;
        rect.xmax = width-1;
        rect.ymax = height-1;
    }
    else
    {
        if (full_update) { /* don't use dirty-bits optimization */
            base = 0;
        }
        if (compute_fb_update_rect_linear(&fbs, base, &rect) == 0) {
            return;
        }
    }

    rect.xmax += 1;
    rect.ymax += 1;

#if 0
    printf("lcd_sdl_update_display (y:%d,h:%d,x=%d,w=%d)\n",
           rect.ymin, rect.ymax-rect.ymin, rect.xmin, rect.xmax-rect.xmin);
#endif

    dpy_update(s->fb->ds, rect.xmin, rect.ymin, rect.xmax-rect.xmin, rect.ymax-rect.ymin);

}

static void lcd_sdl_invalidate_display(conf_object_t * opaque)
{
    // is this called?
    struct lcd_sdl_device *dev = (lcd_sdl_device*)(opaque->obj);
    dev->fb->need_update = 1;
}

static int lcd_sdl_update(conf_object_t *opaque, lcd_surface_t* surface)
{
	struct lcd_sdl_device *dev = (lcd_sdl_device*)(opaque->obj);
        fb_state_t* s = dev->fb;
	s->fb_base = surface->lcd_addr_begin;
	s->need_update = 1;
	return 0; 
}

void lcd_sdl_close()
{
	return NULL;
}

static void events_put_keycode(void *x, int keycode)
{
	conf_object_t* obj = get_conf_obj("lcd_sdl_0");
	struct lcd_sdl_device *lcd_dev = (lcd_sdl_device *)obj->obj;

	lcd_keypad_t* lcd_keypad = SKY_get_interface(obj, LCD_KEYPAD_INTF_NAME);
	lcd_keypad->keypad_update_status(lcd_keypad->obj, keycode);
}

static void skPenEvent(int *buffer, int eventType, int stateType, int x, int y)
{
//      printf("\nSkyEye: skPenEvent():event type=%d\n(x=%d,y=%d)\n",down,x,y);
	buffer[0] = x;
	buffer[1] = y;
	buffer[2] = 0;		// dx
	buffer[3] = 0;		// dy
	buffer[4] = eventType;	// event from pen (DOWN,UP,CLICK,MOVE)
	buffer[5] = stateType;	// state of pen (DOWN,UP,ERROR)
	buffer[6] = 1;		// no of the event
	buffer[7] = 0;		// time of the event (ms) since ts_open
}

static void events_put_mouse(void *opaque, int dx, int dy, int dz, int buttons_state)
{
	conf_object_t* obj = get_conf_obj("lcd_sdl_0");
	struct lcd_sdl_device *lcd_dev = (lcd_sdl_device *)obj->obj;

	int *Pen_buffer;
	lcd_touchscreen_t* lcd_ts = SKY_get_interface(obj, LCD_TS_INTF_NAME);
        Pen_buffer = get_pen_buffer();
	switch(buttons_state)
	{
		case 0:
			skPenEvent(Pen_buffer, 0, 1, dx, dy);
			break;
		case 1:
			skPenEvent(Pen_buffer, 0, 0, dx, dy);
			break;
		case 2:
			if (Pen_buffer[5] == 0){
				skPenEvent(Pen_buffer, 1, 0, dx, dy);
			}
			break;
		default:
			break;
	}
	lcd_ts->touchscreen_update_status(lcd_ts->obj, Pen_buffer);
}

static int lcd_sdl_open(conf_object_t *opaque, lcd_surface_t* surface)
{
	struct lcd_sdl_device *dev = (lcd_sdl_device*)(opaque->obj);
        fb_state_t* fb = dev->fb;
	DBG("In %s, width=%d, height=%d, begin_addr=0x%x,end_addr=0x%x\n, ", __FUNCTION__, surface->width, surface->height, surface->lcd_addr_begin, surface->lcd_addr_end);
	printf("In %s, width=%d, height=%d, begin_addr=0x%x,end_addr=0x%x\n, ", __FUNCTION__, surface->width, surface->height, surface->lcd_addr_begin, surface->lcd_addr_end);
	if (dev == NULL || 
	    surface->width <= 0 || surface->height <= 0) return -1;

#if 0
	if ((fbmem = (guint32*)get_dma_addr(surface->lcd_addr_begin)) == NULL) {
		fprintf(stderr, "[GTK_LCD]: Can't find LCD DMA from address 0x%x\n", surface->lcd_addr_begin);
		return -1;
	}
	DBG("In %s, fb_mem=0x%x\n", __FUNCTION__, fbmem);

	lcd->width = surface->width;
	lcd->virtual_width = surface->width + surface->lcd_line_offset;
	lcd->height = surface->height;
	lcd->depth = surface->depth;
	lcd->update_rect.width = -1;
	lcd->update_rect.height = -1;
	lcd->update_all = TRUE;
#endif

	fb->fb_base = surface->lcd_addr_begin;
	return 0; 
}

void lcd_sdl_init()
{
	conf_object_t* obj = get_conf_obj("lcd_sdl_0");
        struct lcd_sdl_device *dev = (lcd_sdl_device *)obj->obj;
        fb_state_t* fb = dev->fb;

	android_interface_t* android_if = SKY_get_interface(obj, ANDROID_INTF_NAME);

        fb->ds = android_if->graphic_console_init(lcd_sdl_update_display,
                             lcd_sdl_invalidate_display,
                             NULL,
                             NULL,
                             fb);

	printf("in %s,mouse event address 0x%x\n",__func__,events_put_mouse);
	android_if->qemu_add_mouse_event_handler(events_put_mouse,dev, 1, skyeye_strdup("s3c6410-events"));
	android_if->qemu_add_kbd_event_handler(events_put_keycode, dev);

}

static conf_object_t* new_lcd_sdl(char* obj_name){

	if (obj_name == NULL) return NULL;
	lcd_sdl_device* dev = skyeye_mm_zero(sizeof(lcd_sdl_device));
	dev->width = 640;
	dev->height = 480;
	dev->depth = 16;

	dev->obj = new_conf_object(obj_name, dev);
	fb_state_t* fb =  skyeye_mm_zero(sizeof(fb_state_t));
//	fb->dev = dev;

	dev->fb = fb;

        fb->dpi = 165;  /* XXX: Find better way to get actual value ! */


	/* IMPORTANT: DO NOT COMPUTE s->pixel_format and s->bytes_per_pixel
	* here because the display surface is going to change later.
	*/
	fb->bytes_per_pixel = 0;
	fb->pixel_format    = -1;


	/* init lcd_sdl regs */
	int timer_id;
	//create_thread_scheduler(5000, Periodic_sched, timer_update, dev->obj, &timer_id);

	lcd_control_intf* lcd_ctrl = skyeye_mm_zero(sizeof(lcd_control_intf));
	lcd_ctrl->conf_obj = dev->obj;
	lcd_ctrl->lcd_open = lcd_sdl_open;
	lcd_ctrl->lcd_close = lcd_sdl_close;
	lcd_ctrl->lcd_update = lcd_sdl_update;
	lcd_ctrl->lcd_filter_read = NULL;
	lcd_ctrl->lcd_filter_write = NULL;
	lcd_ctrl->lcd_lookup_color = NULL;

	SKY_register_interface(lcd_ctrl, obj_name, LCD_CTRL_INTF_NAME);

	simple_signal_intf* refresh_signal = skyeye_mm_zero(sizeof(simple_signal_intf));
	refresh_signal->conf_obj = NULL;
	refresh_signal->trigger = NULL;
	SKY_register_interface(refresh_signal, obj_name, SIMPLE_SIGNAL_INTF_NAME);
	dev->master = refresh_signal;

//	qemu_add_kbd_event_handler(events_put_keycode, dev);
//	qemu_add_mouse_event_handler(events_put_mouse,dev, 1, "s3c6410-events");

	android_sdl_control_intf * sdl_control = skyeye_mm_zero(sizeof(android_sdl_control_intf));
	sdl_control->conf_obj = dev->obj;
	sdl_control->sdl_ctrl = lcd_sdl_init;
	SKY_register_interface(sdl_control, obj_name,ANDROID_SDL_CTRL_INTF_NAME);

	/* Register io function to the object */
	return dev->obj;
}
void free_lcd_sdl(conf_object_t* dev){
	
}

void init_lcd_sdl(){
	static skyeye_class_t class_data = {
		.class_name = "lcd_sdl",
		.class_desc = "lcd sdl",
		.new_instance = new_lcd_sdl,
		.free_instance = free_lcd_sdl,
		.get_attr = NULL,
		.set_attr = NULL
	};

	SKY_register_class(class_data.class_name, &class_data);
}
