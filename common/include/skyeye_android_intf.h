/* Copyright (C) 
* 2012 - XiaoQiao xq2537@gmail.com
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
* @file skyeye_android_intf.c
* @brief The interface for android controller
* @author XiaoQiao xq2537@gmail.com
* @version 0.1
 @date 2012-06-06
*/
#ifndef __SKYEYE_ANDROID_INTF_H__
#define __SKYEYE_ANDROID_INTF_H__
#include "android/globals.h"  /* for android_hw */
#include "android/skyeye/console.h"

typedef struct android_intf{
	conf_object_t* obj;
	AndroidHwConfig* (*get_android_hw) (void);
	QEMUPutMouseEntry* (*qemu_add_mouse_event_handler)(QEMUPutMouseEvent *func,
                                                void *opaque, int absolute,
                                                const char *name);
	void (*qemu_add_kbd_event_handler)(QEMUPutKBDEvent *func, void *opaque);
	DisplayState* (*graphic_console_init)(vga_hw_update_ptr update,
                                   vga_hw_invalidate_ptr invalidate,
                                   vga_hw_screen_dump_ptr screen_dump,
                                   vga_hw_text_update_ptr text_update,
                                   void *opaque);
	void (*start_android)(void);
}android_interface_t;
#define ANDROID_INTF_NAME "android_intf"

typedef struct android_sdl_ctrl{
	conf_object_t* conf_obj;
	void (* sdl_ctrl)();
}android_sdl_control_intf;
#define ANDROID_SDL_CTRL_INTF_NAME "android_sdl_ctrl"
 

#endif

