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

#ifndef __GOLDFISH_TTY_H___
#define __GOLDFISH_TTY_H___

#include <skyeye_types.h>
#include <skyeye_obj.h>
#include <skyeye_signal.h>
#include <memory_space.h>

struct goldfish_tty_device;
typedef struct tty_state {
	struct goldfish_tty_device* dev;
	uint32_t ptr;
	uint32_t ptr_len;
	uint32_t ready;
	uint8_t data[128];
	uint32_t data_count;
}tty_state_t;

#define TTY0_IRQ 4
typedef struct goldfish_tty_device{
	conf_object_t* obj;
	int line_no;
	tty_state_t* tty;

	conf_object_t* signal_target;
	general_signal_intf* master;
	memory_space_intf* io_memory;
}goldfish_tty_device;

#endif /* __SKYEYE_MACH_GOLDFISH_H___ */
