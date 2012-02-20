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
* @file uart_16550.c
* @brief 16550 uart module
* @author Michael.Kang blackfin.kang@gmail.com
* @version 0.1
* @date 2011-07-13
*/
#ifndef __UART16550_H__
#define __UART16550_H__

#include <skyeye_types.h>
#include <skyeye_obj.h>
#include <memory_space.h>

typedef struct reg_16550{
	uint32 rbr;
	uint32 thr;
	uint32 ier;
	uint32 iir;
	uint32 fcr;
	uint32 lcr;
	uint32 lsr;
	uint32 msr;
	uint32 scr;
	uint32 dll;
	uint32 dlm;
	uint8 t_fifo[16];
	uint8 r_fifo[16];
} reg_16550_t;

typedef struct uart_16550{
	conf_object_t* obj;
	reg_16550_t* reg;
	memory_space_intf* io_memory;
	uint32 irq;
	char name[1024];
}uart_16550_t;

#endif
