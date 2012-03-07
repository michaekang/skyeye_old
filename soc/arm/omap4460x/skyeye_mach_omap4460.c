/*
 * skyeye_mach_omap4460.c:
 *
 * Copyright (C) 2012 Oubang Shen <shenoubang@gmail.com>
 * Skyeye Develop Group, for help please send mail to
 * <skyeye-developer@lists.gro.clinux.org>
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
 */
#include <skyeye_class.h>
#include <skyeye.h>

static conf_object_t* new_omap4460_mach(char* obj_name){
	SKYEYE_LOG_IN_CLR(RED, "In %s, line = %d, new_omap4460_mach not realize\n", __func__, __LINE__);

	return NULL;
}

void free_omap4460_mach(conf_object_t* mach){
}

extern void init_omap4460_mach() {
	SKYEYE_DBG("In %s, line = %d, init_omap4460_mach start!!\n",
			__func__, __LINE__);
	static skyeye_class_t class_data = {
		.class_name = "omap4460_mach",
		.class_desc = "omap4460_machine",
		.new_instance = new_omap4460_mach,
		.free_instance = free_omap4460_mach,
		.get_attr = NULL,
		.set_attr = NULL
	};

	SKY_register_class(class_data.class_name, &class_data);
	SKYEYE_DBG("In %s, line = %d, register omap4460_mach ok!!\n",
			__func__, __LINE__);
}
