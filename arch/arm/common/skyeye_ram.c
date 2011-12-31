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
* @file skyeye_ram.c
* @brief The ram class
* @author Michael.Kang blackfin.kang@gmail.com
* @version 78.77
* @date 2011-12-21
*/

#include <skyeye_class.h>
#include <skyeye_interface.h>
#include <skyeye_obj.h>
#include <skyeye_mm.h>
#include <skyeye_attr.h>
#include <skyeye_ram.h>
#include <memory_space.h>

typedef struct ram_image{
	conf_object_t* obj;
	uint32 base_addr;
	uint32 size;
}ram_image_t;

/**
* @brief The generic memory read function
*
* @param size data width
* @param offset data offset
* @param value the return value of read from
*
* @return the flash used to indicate the success or failure
*/
static exception_t ram_read(conf_object_t *opaque, generic_address_t offset, uint32_t* buf, size_t count){
	ram_image_t* image = (ram_image_t*)opaque->obj;
	if(mem_read(count * 8, offset + image->base_addr, buf) != 0)
		return Invarg_exp;
	else{
		return No_exp;
	}
}

/**
* @brief the write function of the ram
*
* @param size the data width
* @param offset the data offset
* @param value the data value written to
*
* @return the flag
*/
//char mem_write(short size, int offset, uint32_t value){
static exception_t ram_write(conf_object_t *opaque, generic_address_t addr, uint32_t* buf, size_t count){
#if 0
	ram_image_t* image = (ram_image_t*)opaque->obj;
	uint32 value = *(uint32_t*)buf;
	if(mem_write(count*8, addr + image->base_addr, value) != 0)
		return Invarg_exp;
	else
		return No_exp;
#endif
	return No_exp;
}

static conf_object_t* new_ram_image(char* obj_name){
	ram_image_t* image = skyeye_mm_zero(sizeof(ram_image_t));
	image->obj = new_conf_object(obj_name, image);
	image->base_addr = 0x50000000;
	image->size = 0x20000000;
	/* Register io function to the object */
	memory_space_intf* ram_space = skyeye_mm_zero(sizeof(memory_space_intf));
	ram_space->conf_obj = image->obj;
	ram_space->read = ram_read;
	ram_space->write = ram_write;
	SKY_register_interface(ram_space, obj_name, MEMORY_SPACE_INTF_NAME);

	return image->obj;
}
void free_ram_image(conf_object_t* dev){
	
}

void init_ram_image(){
	static skyeye_class_t class_data = {
		.class_name = "ram_image",
		.class_desc = "ram image",
		.new_instance = new_ram_image,
		.free_instance = free_ram_image,
		.get_attr = NULL,
		.set_attr = NULL
	};
	
	SKY_register_class(class_data.class_name, &class_data);
}
