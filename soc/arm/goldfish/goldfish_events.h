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

#ifndef __GOLDFISH_EVENTS_H___
#define __GOLDFISH_EVENTS_H___

#include <skyeye_types.h>
#include <skyeye_obj.h>
#include <skyeye_signal.h>
#include <memory_space.h>
#include "user-events.h"

#define MAX_EVENTS 256*4
struct goldfish_events_device;
typedef struct events_state {
	struct goldfish_events_device* dev;
    uint32_t base;
    int pending;
    int page;

    unsigned events[MAX_EVENTS];
    unsigned first;
    unsigned last;
    unsigned state;

    const char *name;

    struct {
        size_t   len;
        uint8_t *bits;
    } ev_bits[EV_MAX + 1];

    int32_t *abs_info;
    size_t abs_info_count;

}events_state_t;

typedef struct goldfish_events_device{
	conf_object_t* obj;
	int internal_irq;
	int extern_irq;
	events_state_t* events;

	conf_object_t* signal_target;
	general_signal_intf* master;
	memory_space_intf* io_memory;
}goldfish_events_device;

#endif /* __SKYEYE_MACH_GOLDFISH_H___ */
