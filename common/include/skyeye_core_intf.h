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
* @file skyeye_core_intf.h
* @brief The running interface for a processor core
* @author Michael.Kang blackfin.kang@gmail.com
* @version 78.77
* @date 2011-12-20
*/

#ifndef __SKYEYE_CORE_INFT_H__
#define __SKYEYE_CORE_INFT_H__
typedef struct core_run{
	conf_object_t* conf_obj;
	void (*set_pc)(conf_object_t* opaque, generic_address_t addr);
	void (*step_once)(conf_object_t* opaque);
	uint32 (*get_regval_by_id)(conf_object_t* opaque, int id);
	exception_t (*set_regval_by_id)(conf_object_t* opaque, int id, uint32 value);
}core_run_intf;
#define CORE_RUN_INTF_NAME "core_run_intf"
typedef struct core_signal{
	conf_object_t* obj;
	exception_t (*signal)(conf_object_t* obj, interrupt_signal_t* signal);
}core_signal_intf;
#define CORE_SIGNAL_INTF_NAME "core_signal_intf"
#endif
