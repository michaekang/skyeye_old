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
* @file usbhost_s3c6410.c
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

#include "usbhost_s3c6410.h"

static exception_t s3c6410_usbhost_read(conf_object_t *opaque, generic_address_t offset, void* buf, size_t count)
{
	struct s3c6410_usbhost_device *dev = opaque->obj;
	usbhost_reg_t* regs = dev->regs;
	switch(offset) {
		case 0x0:
			*(uint32_t*)buf = regs->hc_revision;
			break;		
		case 0x4:
			*(uint32_t*)buf = regs->hc_control;
			break;
		case 0x8:
			*(uint32_t*)buf = regs->hc_commonstatus;
			break;
		case 0x34:
			*(uint32_t*)buf = regs->hc_rminterval;
			break;
		case 0x44:
			*(uint32_t*)buf = regs->hc_ls_threashold;
			break;
		case 0x48:
			*(uint32_t*)buf = regs->hc_rhdescriptor_a;
			break;
		case 0x54:
			*(uint32_t*)buf = regs->hc_rhportstatus1;
			break;

		default:
			printf("Can not read the register at 0x%x in usbhost\n", offset);
			return Invarg_exp;
	}
	return No_exp;
}

static exception_t s3c6410_usbhost_write(conf_object_t *opaque, generic_address_t offset, uint32_t* buf, size_t count)
{
	struct s3c6410_usbhost_device *dev = opaque->obj;
	usbhost_reg_t* regs = dev->regs;
	uint32_t val = *(uint32_t*)buf;
	switch(offset) {
		case 0x0:
			regs->hc_revision = val;
			break;		
		case 0x4:
			regs->hc_control = val;
			break;
		case 0x8:
			regs->hc_commonstatus = val;
			break;
		case 0x34:
			regs->hc_rminterval = val;
			break;
		case 0x44:
			regs->hc_ls_threashold = val;
			break;
		case 0x48:
			regs->hc_rhdescriptor_a = val;
			break;
		case 0x54:
			regs->hc_rhportstatus1 = val;
			break;
		default:
			printf("Can not write the register at 0x%x in usbhost\n", offset);
			return Invarg_exp;
	}
	return No_exp;
}
static conf_object_t* new_s3c6410_usbhost(char* obj_name){
	s3c6410_usbhost_device* dev = skyeye_mm_zero(sizeof(s3c6410_usbhost_device));
	usbhost_reg_t* regs =  skyeye_mm_zero(sizeof(usbhost_reg_t));
	dev->obj = new_conf_object(obj_name, dev);
	/* init usbhost regs */
	regs->hc_revision = 0x10;
	regs->hc_control = 0x0;
	regs->hc_commonstatus = 0x0;
	regs->hc_rminterval = 0x2edf;
	regs->hc_ls_threashold = 0x628;
	regs->hc_rhdescriptor_a = 0x02001202;
	regs->hc_rhportstatus1 = 0x100;
	dev->regs = regs;

	/* Register io function to the object */
	memory_space_intf* io_memory = skyeye_mm_zero(sizeof(memory_space_intf));
	io_memory->conf_obj = dev->obj;
	io_memory->read = s3c6410_usbhost_read;
	io_memory->write = s3c6410_usbhost_write;
	SKY_register_interface(io_memory, obj_name, MEMORY_SPACE_INTF_NAME);	
	return dev->obj;
}
void free_s3c6410_usbhost(conf_object_t* dev){
	
}

void init_s3c6410_usbhost(){
	static skyeye_class_t class_data = {
		.class_name = "s3c6410_usbhost",
		.class_desc = "s3c6410 usbhost",
		.new_instance = new_s3c6410_usbhost,
		.free_instance = free_s3c6410_usbhost,
		.get_attr = NULL,
		.set_attr = NULL
	};
		
	SKY_register_class(class_data.class_name, &class_data);
}
