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
#define DEBUG
#include <skyeye_log.h>

#include "touchscreen_s3c6410.h"

static exception_t s3c6410_touchscreen_read(conf_object_t *opaque, generic_address_t offset, void* buf, size_t count)
{
	uint32_t ret;
	struct s3c6410_fb_device *s = opaque->obj;
	//DBG("yukewei ######################### In %s, offset=0x%x\n", __FUNCTION__, offset);
	printf("yukewei ######################### In %s, offset=0x%x\n", __FUNCTION__, offset);
	switch(offset) {
	default:
		printf("Can not read the register at 0x%x\n", offset);
		return 0;
	}
}

//static void s3c6410_fb_write(void *opaque, target_phys_addr_t offset,
 //                       uint32_t val)
static exception_t s3c6410_touchscreen_write(conf_object_t *opaque, generic_address_t offset, uint32_t* buf, size_t count)
{
	struct s3c6410_touchscreen_device *s = opaque->obj;
	//DBG("yukewei ####################### In %s, offset=0x%x\n", __FUNCTION__, offset);
	printf("yukewei ######################### In %s, offset=0x%x\n", __FUNCTION__, offset);
	uint32_t val = *(uint32_t*)buf;
	switch(offset) {
        default:
            break;
	}
}
static conf_object_t* new_s3c6410_touchscreen(char* obj_name){
	s3c6410_touchscreen_device* dev = skyeye_mm_zero(sizeof(s3c6410_touchscreen_device));
	dev->obj = new_conf_object(obj_name, dev);
	touchscreen_reg_t* regs =  skyeye_mm_zero(sizeof(touchscreen_reg_t));
	/* init touchscreen regs */
	regs->adccon = 0x3fc4;
	regs->adctsc = 0x0058;
	regs->adcdly = 0x00ff;
	regs->adcupdn = 0x0000;
	dev->regs = regs;
	/* Register io function to the object */
	memory_space_intf* io_memory = skyeye_mm_zero(sizeof(memory_space_intf));
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
