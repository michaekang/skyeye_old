/* vi: set sw=4 ts=4: */
/*
 * goldfish_bus.h: This file is part of ____
 *
 * Copyright (C) 2012 Oubang Shen <shenoubang@gmail.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
+ * */

#ifndef __GOLDFISH_BUS_H__
#define __GOLDFISH_BUS_H__  1

#include <skyeye_types.h>
#include <skyeye_obj.h>
#include <skyeye_signal.h>
#include <memory_space.h>

struct goldfish_device;

typedef struct goldfish_device {
	conf_object_t* obj;
	struct goldfish_device *next;
	struct goldfish_device *prev;
	uint32_t reported_state;
	//void *cookie;
	const char *name;
	uint32_t id;
	uint32_t base; // filled in by goldfish_device_add if 0
	uint32_t size;
	uint32_t irq; // filled in by goldfish_device_add if 0
	uint32_t irq_count;
}goldfish_device_t;

typedef struct bus_state {
	goldfish_device_t dev;
	goldfish_device_t* current;
}bus_state_t;

typedef exception_t (*goldfish_add_device_no_io_t)(conf_object_t* goldfish_bus, conf_object_t *obj,
		uint32_t id, uint32_t base, uint32_t size, uint32_t irq, uint32_t irq_count);

typedef struct goldfish_add_device {
	conf_object_t* conf_obj;
	goldfish_add_device_no_io_t add_device;
}goldfish_add_device_t;

#define BUS0_IRQ 1
typedef struct goldfish_bus_device {
	conf_object_t* obj;
	bus_state_t* bus;
	memory_space_intf* io_memory;
	conf_object_t* signal_target;
	general_signal_intf* master;
	goldfish_device_t *first_device;
	goldfish_device_t *last_device;
	goldfish_add_device_t* add_device;
	uint32_t goldfish_free_base;
	uint32_t goldfish_free_irq;
	int line_no;
}goldfish_bus_device_t;

#endif /* __GOLDFISH_BUS_H__ */

