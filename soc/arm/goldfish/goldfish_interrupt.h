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

#ifndef __GOLDFISH_INTERRUPT_H__
#define __GOLDFISH_INTERRUPT_H__

#include <skyeye_types.h>
#include <skyeye_obj.h>
#include <skyeye_signal.h>
#include <memory_space.h>

typedef struct pic_state{
    uint32_t level;
    uint32_t pending_count;
    uint32_t irq_enabled;
    uint32_t fiq_enabled;
}pic_state_t;

typedef struct goldfish_pic_device {
	conf_object_t* obj;
	pic_state_t* state;
	memory_space_intf* io_memory;
	general_signal_intf* slave;
}goldfish_pic_device;


#endif /* __SKYEYE_MACH_GOLDFISH_H___ */





