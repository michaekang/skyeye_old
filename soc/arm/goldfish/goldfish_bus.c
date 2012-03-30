/* vi: set sw=4 ts=4: */
/*
 * goldfish_bus.c: This file is part of ____
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
 * */
#include <skyeye_log.h>
#include <skyeye_mm.h>
#include <skyeye_obj.h>
#include <skyeye_class.h>
#include <skyeye_types.h>
#include <skyeye_signal.h>
#include "goldfish_bus.h"
#include <skyeye_arch.h>
#include <skyeye_config.h>
#include <skyeye_sched.h>
#include <skyeye_options.h>
#include <stdio.h>

#define PDEV_BUS_OP_DONE         0x00
#define PDEV_BUS_OP_REMOVE_DEV   0x04
#define PDEV_BUS_OP_ADD_DEV      0x08

#define PDEV_BUS_OP_INIT         0x00

#define PDEV_BUS_OP              0x00
#define PDEV_BUS_GET_NAME        0x04
#define PDEV_BUS_NAME_LEN        0x08
#define PDEV_BUS_ID              0x0c
#define PDEV_BUS_IO_BASE         0x10
#define PDEV_BUS_IO_SIZE         0x14
#define PDEV_BUS_IRQ             0x18
#define PDEV_BUS_IRQ_COUNT       0x1c

static exception_t goldfish_add_device_no_io(conf_object_t* bus_obj, conf_object_t *obj,
		uint32_t id, uint32_t base, uint32_t size, uint32_t irq, uint32_t irq_count) {

	goldfish_bus_device_t* goldfish_bus = (goldfish_bus_device_t*)bus_obj->obj;
	goldfish_device_t* dev = skyeye_mm(sizeof(goldfish_device_t));
	dev->obj = obj;
	dev->name = obj->objname;
	dev->id = id;
	dev->base = base;
	dev->size = size;
	dev->irq = irq;
	dev->irq_count = irq_count;

	if(dev->base == 0) {
		dev->base = goldfish_bus->goldfish_free_base;
		goldfish_bus->goldfish_free_base += dev->size;
        }
	if(dev->irq == 0 && dev->irq_count > 0) {
		dev->irq = goldfish_bus->goldfish_free_irq;
		goldfish_bus->goldfish_free_irq += dev->irq_count;
	}
	//printf("goldfish_add_device: %s, base %x %x, irq %d %d\n",
	//      dev->name, dev->base, dev->size, dev->irq, dev->irq_count);
	dev->next = NULL;
	if(goldfish_bus->last_device) {
		goldfish_bus->last_device->next = dev;
	}
	else {
		goldfish_bus->first_device = dev;
	}
	goldfish_bus->last_device = dev;

	return No_exp;
}

static exception_t goldfish_bus_read(conf_object_t* opaque, generic_address_t offset, void* buf, size_t count) {
	conf_object_t* obj = get_conf_obj("goldfish_device_bus0");
	goldfish_bus_device_t *dev = (goldfish_bus_device_t *)obj->obj;
	bus_state_t* s = dev->bus;

        switch (offset) {
        case PDEV_BUS_OP:
            if(s->current) {
                s->current->reported_state = 1;
                s->current = s->current->next;
            }
            else {
                s->current = dev->first_device;
            }
            while(s->current && s->current->reported_state == 1)
                s->current = s->current->next;
            if(s->current) {
                //return PDEV_BUS_OP_ADD_DEV;
		printf("add dev name %s\n",s->current->name);
		*(uint32_t *)buf = PDEV_BUS_OP_ADD_DEV;
	    }
            else {
		dev->master->lower_signal(dev->signal_target, dev->line_no);
                //goldfish_device_set_irq(&s->dev, 0, 0);
                //return PDEV_BUS_OP_DONE;
		*(uint32_t *)buf = PDEV_BUS_OP_DONE;
            }
	    break;

        case PDEV_BUS_NAME_LEN:
		if (s->current) {
			*(uint32_t *)buf = strlen(s->current->name);
		}
		else {
			*(uint32_t *)buf = 0;
		}
	        break;
        case PDEV_BUS_ID:
		if (s->current) {
			*(uint32_t *)buf = s->current->id;
		}
		else {
			*(uint32_t *)buf = 0;
		}
		break;
            //return s->current ? s->current->id : 0;
        case PDEV_BUS_IO_BASE:
			if (s->current) {
				*(uint32_t *)buf = s->current->base;
			}
			else {
				*(uint32_t *)buf = 0;
			}
			break;
            //return s->current ? s->current->base : 0;
        case PDEV_BUS_IO_SIZE:
			if (s->current) {
				*(uint32_t *)buf = s->current->size;
			}
			else {
				*(uint32_t *)buf = 0;
			}
			break;
            //return s->current ? s->current->size : 0;
        case PDEV_BUS_IRQ:
			if (s->current) {
				*(uint32_t *)buf = s->current->irq;
			}
			else {
				*(uint32_t *)buf = 0;
			}
			break;
            //return s->current ? s->current->irq : 0;
        case PDEV_BUS_IRQ_COUNT:
			if (s->current) {
				*(uint32_t *)buf = s->current->irq_count;
			}
			else {
				*(uint32_t *)buf = 0;
			}
			break;
	default:
			break;
	}
	return No_exp;
}

//static void goldfish_bus_op_init(bus_state_t *s)
static void goldfish_bus_op_init(conf_object_t* opaque) {
	conf_object_t* obj = get_conf_obj("goldfish_device_bus0");
	goldfish_bus_device_t *dev_bus = (goldfish_bus_device_t *)obj->obj;
	bus_state_t* s = dev_bus->bus;
	goldfish_device_t* dev = dev_bus->first_device;

	while(dev) {
		dev->reported_state = 0;
		dev = dev->next;
	}
	s->current = NULL;
	// FIXME: how  to do for irq
	if (dev_bus->first_device != NULL)
	{
		printf("bus sent interrupt\n");
		dev_bus->master->raise_signal(dev_bus->signal_target, dev_bus->line_no);
	}
	else
		dev_bus->master->lower_signal(dev_bus->signal_target, dev_bus->line_no);

	//goldfish_device_set_irq(&s->dev, 0, first_device != NULL);
}

static exception_t goldfish_bus_write(conf_object_t* opaque, generic_address_t offset, void* buf, size_t count) {
	conf_object_t* obj = get_conf_obj("goldfish_device_bus0");
	goldfish_bus_device_t *dev = (goldfish_bus_device_t *)obj->obj;
	bus_state_t* s = dev->bus;
	uint32_t value = *(uint32_t*)buf;
        switch(offset) {
	case PDEV_BUS_OP:
            switch(value) {
                case PDEV_BUS_OP_INIT:
		    printf("init bus op\n");
                    goldfish_bus_op_init(opaque);
                    break;
                default:
	            return Invarg_exp;
            };
            break;
        case PDEV_BUS_GET_NAME:
	    {
	    uint32_t index;
	    uint32_t fault;

            if(s->current) {
		    for (index = 0;index < strlen(s->current->name) - 1;index++)
		    {
			    skyeye_config_t* config = get_current_config();
			    generic_arch_t *arch_instance = get_arch_instance(config->arch->arch_name);
			    fault = arch_instance->mmu_write(8, value, (s->current->name)[index]);
			    if(fault)
				fprintf(stderr, "SKYEYE:read virtual address error!!!\n" );
			     value ++;

		    }
            }
	    }
            break;
        default:
		return Invarg_exp;
	}
	return No_exp;
}


static conf_object_t* new_goldfish_bus_device(char* obj_name){
	goldfish_bus_device_t* dev = (goldfish_bus_device_t*)skyeye_mm_zero(sizeof(goldfish_bus_device_t));
	if (dev == NULL) {
		fprintf(stderr, "MM failed in %s\n", __FUNCTION__);
		return NULL;
	}
	dev->obj = new_conf_object(obj_name, dev);
	dev->bus = skyeye_mm_zero(sizeof(bus_state_t));
	dev->io_memory = skyeye_mm_zero(sizeof(memory_space_intf));
	dev->add_device = skyeye_mm_zero(sizeof(goldfish_add_device_t));
	if ((dev->obj == NULL) || (dev->bus == NULL) || (dev->io_memory == NULL) || (dev->add_device == NULL)) {
		fprintf(stderr, "MM failed in %s\n", __FUNCTION__);
		return NULL;
	}
	dev->io_memory->read = goldfish_bus_read;
	dev->io_memory->write = goldfish_bus_write;
	dev->add_device->conf_obj = dev->obj;
	dev->add_device->add_device = goldfish_add_device_no_io;
	dev->goldfish_free_base = 0x10000;
	dev->goldfish_free_irq = 10;
	return dev->obj;
}

static void del_goldfish_bus_device(char* obj_name){
}

void init_goldfish_bus(){
    static skyeye_class_t goldfish_bus_class = {
        .class_name = "goldfish_bus",
        .class_desc = "goldfish bus",
        .new_instance = new_goldfish_bus_device,
        .free_instance = del_goldfish_bus_device,
        .get_attr = NULL,
        .set_attr = NULL
    };
    SKY_register_class(goldfish_bus_class.class_name, &goldfish_bus_class);
}

