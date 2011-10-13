/* Copyright (C) 2007-2008 The Android Open Source Project
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
*/
//#include "android/android.h"
//#include "android/utils/debug.h"
//#include "android/utils/duff.h"
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

#include "touchscreen_s3c6410.h"


static exception_t touchscreen_updata_status(conf_object_t* touchscreen)
{
	int *Pen_buffer = get_pen_buffer();
	s3c6410_touchscreen_device* dev = (s3c6410_touchscreen_device*)touchscreen->obj;
	touchscreen_reg_t* regs = dev->regs;
	s3c6410_touchscreen_status* status = (s3c6410_touchscreen_status*)dev->status;
#if 0
	status->x = Pen_buffer[0];
	status->y = Pen_buffer[1];
	status->event = Pen_buffer[4];
	status->stylus = Pen_buffer[5];
	// set touchscreen status in regs
	printf("x = %d, y = %d, envent = %d, stylus = %d\n",
		status->x, status->y, status->event, status->stylus);
#endif
	return No_exp;
}

static exception_t s3c6410_touchscreen_read(conf_object_t *opaque, generic_address_t offset, void* buf, size_t count)
{
	struct s3c6410_touchscreen_device *dev = opaque->obj;
	s3c6410_touchscreen_status* status = dev->status;
	touchscreen_reg_t* regs = dev->regs;
	DBG("yukewei ######################### In %s, offset=0x%x\n", __FUNCTION__, offset);
	switch(offset) {
		case 0x0:
			*(uint32_t*)buf = regs->adccon; // 0x7E00_B000
			break;
		case 0x4:
			*(uint32_t*)buf = regs->adctsc; // 0x7E00_B004
			break;
		case 0x8:
			*(uint32_t*)buf = regs->adcdly; // 0x7E00_B008
			break;
		case 0x0C:
			*(uint32_t*)buf = regs->adcdat0; // 0x7E00_B00C
			break;
		case 0x10:
			*(uint32_t*)buf = regs->adcdat1; // 0x7E00_B010
			break;
		case 0x14:
			*(uint32_t*)buf = regs->adcupdn; // 0x7E00_B014
			break;
		case 0x18:
			*(uint32_t*)buf = regs->adcclrint; // 0x7E00_B018
			break;
		case 0x20:
			*(uint32_t*)buf = regs->adcclrintpndnup; // 0x7E00_B020
			break;
		default:
			printf("Can not read the register at 0x%x in touchscreen\n", offset);
			return 0;
	}
	return No_exp;
}

static exception_t s3c6410_touchscreen_write(conf_object_t *opaque, generic_address_t offset, uint32_t* buf, size_t count)
{
	struct s3c6410_touchscreen_device *dev = opaque->obj;
	s3c6410_touchscreen_status* status = dev->status;
	touchscreen_reg_t* regs = dev->regs;
	DBG("yukewei ######################### In %s, offset=0x%x\n", __FUNCTION__, offset);
	uint32_t val = *(uint32_t*)buf;
	switch(offset) {
		case 0x0:
			regs->adccon = val; // 0x7E00_B000
			break;
		case 0x4:
			regs->adctsc = val; // 0x7E00_B004
			break;
		case 0x8:
			regs->adcdly = val; // 0x7E00_B008
			break;
		case 0x0C:
			regs->adcdat0 = val; // 0x7E00_B00C
			break;
		case 0x10:
			regs->adcdat1 = val; // 0x7E00_B010
			break;
		case 0x14:
			regs->adcupdn = val; // 0x7E00_B014
			break;
		case 0x18:
			regs->adcclrint = val; // 0x7E00_B018
			break;
		case 0x20:
			regs->adcclrintpndnup = val; // 0x7E00_B020
			break;
		default:
			printf("Can not write the register at 0x%x in touchscreen\n", offset);
			return 0;
	}
	return No_exp;
}
static conf_object_t* new_s3c6410_touchscreen(char* obj_name){
	s3c6410_touchscreen_device* dev = skyeye_mm_zero(sizeof(s3c6410_touchscreen_device));
	touchscreen_reg_t* regs =  skyeye_mm_zero(sizeof(touchscreen_reg_t));
	s3c6410_touchscreen_status* status = skyeye_mm_zero(sizeof(s3c6410_touchscreen_status));
	dev->obj = new_conf_object(obj_name, dev);
	/* init touchscreen regs */
	regs->adccon = 0x3fc4;
	regs->adctsc = 0x0058;
	regs->adcdly = 0x00ff;
	regs->adcupdn = 0x0000;
	dev->regs = regs;
	dev->status = status;

	int timer_id;
	create_thread_scheduler(10000, Periodic_sched, touchscreen_updata_status, dev->obj, &timer_id);
	/* Register io function to the object */
	memory_space_intf* io_memory = skyeye_mm_zero(sizeof(memory_space_intf));
	io_memory->conf_obj = dev->obj;
	io_memory->read = s3c6410_touchscreen_read;
	io_memory->write = s3c6410_touchscreen_write;
	SKY_register_interface(io_memory, obj_name, MEMORY_SPACE_INTF_NAME);	
	return dev->obj;
}
void free_s3c6410_touchscreen(conf_object_t* dev){
	
}

void init_s3c6410_touchscreen(){
	static skyeye_class_t class_data = {
		.class_name = "s3c6410_touchscreen",
		.class_desc = "s3c6410 touchscreen",
		.new_instance = new_s3c6410_touchscreen,
		.free_instance = free_s3c6410_touchscreen,
		.get_attr = NULL,
		.set_attr = NULL
	};
		
	SKY_register_class(class_data.class_name, &class_data);
}
