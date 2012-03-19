/*
	goldfish.h - definitions of "goldfish" machine  for skyeye
	Copyright (C) 2004 Skyeye Develop Group
	for help please send mail to <skyeye-developer@lists.gro.clinux.org>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*/

#ifndef __GOLDFISH_FB_H___
#define __GOLDFISH_FB_H___

#include <skyeye_types.h>
#include <skyeye_lcd_intf.h>
#include <skyeye_obj.h>
#include <skyeye_signal.h>
#include <memory_space.h>
#include "android/skyeye/console.h"

struct goldfish_fb_device;
typedef struct fb_state {
	struct goldfish_fb_device* dev;
	DisplayState*  ds;
	int      pixel_format;
	int      bytes_per_pixel;
	uint32_t fb_base;
	uint32_t base_valid : 1;
	uint32_t need_update : 1;
	uint32_t need_int : 1;
	uint32_t set_rotation : 2;
	uint32_t blank : 1;
	uint32_t int_status;
	uint32_t int_enable;
	int      rotation;   /* 0, 1, 2 or 3 */
	int      dpi;

}fb_state_t;

#define FB0_IRQ 14
typedef struct goldfish_fb_device{
	conf_object_t* obj;
	int line_no;
	fb_state_t* fb;

	conf_object_t* signal_target;
	general_signal_intf* master;

	memory_space_intf* io_memory;
}goldfish_fb_device;

#endif /* __SKYEYE_MACH_GOLDFISH_H___ */
