/* Copyright (C) 
* 2011 - Michael.Kang blackfin.kang@gmail.com
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
* @file gpio_s3c6410.c
* @brief The implementation of system controller
* @author Michael.Kang blackfin.kang@gmail.com
* @version 78.77
* @date 2011-12-12
*/

#include <skyeye_types.h>
#include <skyeye_sched.h>
#include <skyeye_signal.h>
#include <skyeye_class.h>
#include <skyeye_interface.h>
#include <skyeye_obj.h>
#include <skyeye_mm.h> 
#include <memory_space.h>
#include <skyeye_device.h>
#define DEBUG
#include <skyeye_log.h>

#include "gpio_s3c6410.h"

static exception_t s3c6410_gpio_read(conf_object_t *opaque, generic_address_t offset, void* buf, size_t count)
{
	struct s3c6410_gpio_device *dev = opaque->obj;
	gpio_reg_t* regs = dev->regs;
	switch(offset) {
		case 0x20:
			*(uint32_t*)buf = regs->gpbcon; // 0x7E00_B000
			break;
		case 0xa0:
			*(uint32_t*)buf = regs->gpfcon;
			break;
		case 0xa4:
			*(uint32_t*)buf = regs->gpfdat;		
			break;
		case 0xa8:
			*(uint32_t*)buf = regs->gpfpud;
			break;

		case 0xe0:
			*(uint32_t*)buf = regs->gphcon0;
			break;

		case 0xe4:
			*(uint32_t*)buf = regs->gphcon1;
			break;

		case 0xe8:
			*(uint32_t*)buf = regs->gphdat;
			break;
		case 0x100:
			*(uint32_t*)buf = regs->gpicon;
			break;
		case 0x120:
			*(uint32_t*)buf = regs->gpjcon;
			break;
		case 0x800:
			*(uint32_t*)buf = regs->gpkcon0;
			break;
		case 0x804:
			*(uint32_t*)buf = regs->gpkcon1;
			break;
		case 0x808:
			*(uint32_t*)buf = regs->gpkdat;
			break;
		case 0x80c:
			*(uint32_t*)buf = regs->gpkpud;
			break;
		case 0x810:
			*(uint32_t*)buf = regs->gplcon1;
			break;
		case 0x814:
			*(uint32_t*)buf = regs->gplcon1;
			break;
		case 0x818:
			*(uint32_t*)buf = regs->gpldat;
			break;
		case 0x81c:
			*(uint32_t*)buf = regs->gplpud;
			break;
		case 0x830:
			*(uint32_t*)buf = regs->gpncon;
			break;
		case 0x834:
			*(uint32_t*)buf = regs->gpndat;
			break;
		case 0x900:
			*(uint32_t*)buf = regs->eint0con0;
			break;
		case 0x920:
			*(uint32_t*)buf = regs->eint0mask;
			break;
		case 0x924:
			*(uint32_t*)buf = regs->eint0pend;
			break;

		default:
			printf("Can not read the register at 0x%x in gpio\n", offset);
			return Invarg_exp;
	}
	return No_exp;
}

static exception_t s3c6410_gpio_write(conf_object_t *opaque, generic_address_t offset, uint32_t* buf, size_t count)
{
	struct s3c6410_gpio_device *dev = opaque->obj;
	gpio_reg_t* regs = dev->regs;
	uint32_t val = *(uint32_t*)buf;
	switch(offset) {
		case 0x20:
			regs->gpbcon = val; // 0x7E00_B000
			break;
		case 0xa0:
			regs->gpfcon = val;
			break;
		case 0xa4:
			regs->gpfdat = val;		
			break;
		case 0xa8:
			regs->gpfpud = val;
			break;
		case 0xe0:
			regs->gphcon0 = val;
			break;

		case 0xe4:
			regs->gphcon1 = val;
			break;

		case 0xe8:
			regs->gphdat = val;
			break;

		case 0x100:
			regs->gpicon = val;
			break;
		case 0x120:
			regs->gpjcon = val;
			break;
		case 0x800:
			regs->gpkcon0 = val;
			break;
		case 0x804:
			regs->gpkcon1 = val;
			break;
		case 0x808:
			regs->gpkdat = val;
			break;
		case 0x80c:
			regs->gpkpud = val;
			break;
		case 0x810:
			regs->gplcon1 = val;
			break;
		case 0x814:
			regs->gplcon1 = val;
			break;
		case 0x818:
			regs->gpldat = val;
			break;
		case 0x81c:
			regs->gplpud = val;
			break;

		case 0x830:
			regs->gpncon = val;
			break;
		case 0x834:
			regs->gpndat = val;
			break;
		case 0x900:
			regs->eint0con0 = val;
			break;
		case 0x920:
			regs->eint0mask = val;
			break;
		case 0x924:
			regs->eint0pend = val;
			break;
		default:
			printf("Can not write the register at 0x%x in gpio\n", offset);
			return Invarg_exp;
	}
	return No_exp;
}
static conf_object_t* new_s3c6410_gpio(char* obj_name){
	s3c6410_gpio_device* dev = skyeye_mm_zero(sizeof(s3c6410_gpio_device));
	gpio_reg_t* regs =  skyeye_mm_zero(sizeof(gpio_reg_t));
	dev->obj = new_conf_object(obj_name, dev);
	/* init gpio regs */
	regs->gpbcon = 0x40000;
	regs->eint0mask = 0x0FFFFFFF;
	regs->gpfpud = 0x55555555;

	regs->gpkcon0 = 0x22222222;
	regs->gpkcon1 = 0x22222222;
	regs->gpkpud = 0x55555555;

	regs->gplcon0 = 0x22222222;
	regs->gplcon1 = 0x02222222;
	regs->gplpud = 0x15555555;

	dev->regs = regs;

	/* Register io function to the object */
	memory_space_intf* io_memory = skyeye_mm_zero(sizeof(memory_space_intf));
	io_memory->conf_obj = dev->obj;
	io_memory->read = s3c6410_gpio_read;
	io_memory->write = s3c6410_gpio_write;
	SKY_register_interface(io_memory, obj_name, MEMORY_SPACE_INTF_NAME);	
	return dev->obj;
}
void free_s3c6410_gpio(conf_object_t* dev){
	
}

void init_s3c6410_gpio(){
	static skyeye_class_t class_data = {
		.class_name = "s3c6410_gpio",
		.class_desc = "s3c6410 gpio",
		.new_instance = new_s3c6410_gpio,
		.free_instance = free_s3c6410_gpio,
		.get_attr = NULL,
		.set_attr = NULL
	};
		
	SKY_register_class(class_data.class_name, &class_data);
}
