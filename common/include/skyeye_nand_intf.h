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
* @file skyeye_nand_intf.c
* @brief The interface for nand controller
* @author xiaoqiao xq2537@gmail.com 
* @version 0.1
* @date 2012-03-20
*/
#ifndef __SKYEYE_NAND_INTF_H__
#define __SKYEYE_NAND_INTF_H__
typedef struct goldfish_nand_ctrl{
	conf_object_t* conf_obj;
	void (* nand_ctrl)(char *);
}goldfish_nand_control_intf;
#define NAND_CTRL_INTF_NAME "goldfish_nand_ctrl"
 
#endif
