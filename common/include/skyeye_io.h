/* Copyright (C) 
* 2012 - Michael.Kang blackfin.kang@gmail.com
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
* @file skyeye_io.h
* @brief The io related interface
* @author Michael.Kang blackfin.kang@gmail.com
* @version 7849
* @date 2012-05-08
*/

#ifndef __SKYEYE_IO_H__
#define __SKYEYE_IO_H__
#include <skyeye_types.h>
typedef void (*io_cycle_func_t)(void *arg);
void register_io_cycle(io_cycle_func_t func,void* arg);
#endif
