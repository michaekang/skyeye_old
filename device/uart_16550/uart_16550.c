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

#include <stdlib.h>
#include <assert.h>

#include <bank_defs.h>
#include <skyeye_sched.h>
#include <skyeye_options.h>
#include <skyeye_config.h>
#include <skyeye_command.h>
#include <skyeye_uart_ops.h>
#include <skyeye_class.h>
#include <skyeye_mm.h>

#include "uart_16550.h"

#define DEBUG
#include <skyeye_log.h>

const static char* class_name = "uart_16550";

static exception_t  uart_16550_read(conf_object_t *obj,generic_address_t offset, void* buf, size_t count){
	uint32 data;
	uart_16550_t *uart = (uart_16550_t *)(obj->obj);
	reg_16550_t* reg = uart->reg;
	switch ((offset & 0xfff) >> 2) {
	case 0x0:		// RbR
		reg->lsr &= ~0x1;
		/*
		if (i == 0)
			io.vic.risr &= art_16550->irq;
		*/
		data = reg->rbr;
		break;

	case 0x1:		// ier
		data = reg->ier;
		break;
	case 0x2:		// iir
		data = reg->iir;
		break;
	case 0x3:		// IDR
	case 0x4:		// IMR
	case 0x5:		// LSR
		data = reg->lsr;
		break;
	case 0x6:		// MSR
		data = reg->msr;
		break;
	case 0x7:		// SCR
		data = reg->scr;
		break;

	default:
		//DBG_PRINT ("uart_read(%s=0x%08x)\n", "uart_reg", addr);

		break;
	}
	*(uint32 *)buf = data;
	return No_exp;
}

static exception_t  uart_16550_write(conf_object_t *obj, generic_address_t offset, void* buf, size_t count){
	uart_16550_t *uart = (uart_16550_t *)(obj->obj);
	reg_16550_t* reg = uart->reg;
	/* get a reference of object by its name */
	switch ((offset & 0xfff) >> 2) {
	case 0x0:		// THR
		{
			char c = *(char *)buf;
			skyeye_uart_write(-1, &c, 1, NULL);
			reg->lsr |= 0x20;
		}
	case 0x2:		//FCR
		reg->fcr = *(uint32 *)buf;
		break;
	case 0x7:		// SCR
		reg->scr = *(uint32 *)buf;
		break;
	default:
		//DBG_PRINT ("uart_write(%s=0x%08x)\n", "uart_reg", addr);
		break;
	}
	return No_exp;

}

static void uart_16550_io_do_cycle(void* uart_16550){
	uart_16550_t* uart = (uart_16550_t *)uart_16550;
	reg_16550_t* reg = uart->reg;

	if (reg->ier & 0x2) {	/* THREI enabled */
		//printf("In %s, THR interrupt\n", __FUNCTION__);
		reg->iir = (reg->iir & 0xf0) | 0x2;
		reg->lsr |= 0x60;
	}


	if (reg->ier & 0x1) {	/* RDAI enabled */
		struct timeval tv;
		unsigned char buf;

		tv.tv_sec = 0;
		tv.tv_usec = 0;
	
		if(skyeye_uart_read(-1, &buf, 1, &tv, NULL) > 0)
		{
			//printf("SKYEYE:get input is %c\n",buf);
			reg->rbr = buf;
			reg->lsr |= 0x1;
			reg->iir = (reg->iir & 0xf0) | 0x4;
			skyeye_config_t* config = get_current_config();
			config->mach->mach_intr_signal(uart->irq, High_level);
		}
	}
}

static conf_object_t* create_16550_uart(const char *name){
	uart_16550_t* uart = (uart_16550_t *)skyeye_mm_zero(sizeof(uart_16550_t));
	/* register own cycle handler to scheduler. 
	 * set the cycle to 1 ms, and periodic schedle
	 * */
	if(!uart){
		fprintf(stderr,"MM failed in %s\n",__FUNCTION__);
		return NULL;
	}
	uint32 id;
	create_timer_scheduler(1, Periodic_sched, uart_16550_io_do_cycle, uart, &id);
	uart->obj = new_conf_object(name,(void *)uart);
	reg_16550_t* reg = (reg_16550_t*)skyeye_mm_zero(sizeof(reg_16550_t));
	uart->reg = reg;

	uart->io_memory = (memory_space_intf *)skyeye_mm_zero(sizeof(*uart->io_memory));
	uart->io_memory->conf_obj = uart->obj;
	uart->io_memory->read = uart_16550_read;
	uart->io_memory->write = uart_16550_write;
	/* 
	 * FIXME, we have the same bank data structure both in 
	 * global_memmap and here. Should free one.
	 * */
	return uart->obj;
}
static void del_16550_uart(const char *name){
}

void init_16550_uart(void){
	static skyeye_class_t class_data = {
		.class_name = "uart_16550",
		.class_desc = "uart 16550",
		.new_instance = create_16550_uart,
		.free_instance = del_16550_uart,
		.set_attr = NULL,
		.get_attr = NULL
	};
	SKY_register_class(class_data.class_name,&class_data);
}

/*



static int do_16550_option(skyeye_option_t* this_option, int num_params,
		const char *params[])
{
	char name[MAX_PARAM_NAME], value[MAX_PARAM_NAME];
	uint32 addr, len, irq;
	int i;
	for (i = 0; i < num_params; i++) {
		if (split_param (params[i], name, value) < 0)
			SKYEYE_ERR
				("Error: uart has wrong parameter \"%s\".\n",
				 name);
		if (!strncmp ("base", name, strlen (name))) {
			sscanf (value, "%x", &addr);
		}
		else if (!strncmp ("length", name, strlen (name))) {
			sscanf (value, "%x", &len);
		}
		else if (!strncmp ("irq", name, strlen (name))) {
			sscanf (value, "%x", &irq);
		}
		else
                        SKYEYE_ERR ("Error: Unknown uart_16550 option  \"%s\"\n", params[i]);
	}
	create_16550_uart(addr, len, irq);
}
*/

/*
 * Create a 16550 uart by given base, len ,irq
 */
/*
int com_create_16550(char *arg) {
	return 0;
}*/
/*
void init_uart_16550(){
	register_option("uart_16550", do_16550_option, "Uart settings"); 
	add_command("create_uart_16550", com_create_16550, "Create a new uart of 16550 type.\n");
}
*/
