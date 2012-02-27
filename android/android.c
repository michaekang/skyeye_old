/* Copyright (C) 
* xq2537@gmail.com
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
* @file android.c
* @brief The implementation of system controller
* @author xq2537@gmail.com
* @version 78.77
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

#include "android_main.h"

static conf_object_t* new_android(char* obj_name){
	android_main();
	return NULL;
}

void free_android(conf_object_t* dev){
	
}

void init_android(){
	static skyeye_class_t class_data = {
		.class_name = "android",
		.class_desc = "android",
		.new_instance = new_android,
		.free_instance = free_android,
		.get_attr = NULL,
		.set_attr = NULL
	};

	SKY_register_class(class_data.class_name, &class_data);
}
