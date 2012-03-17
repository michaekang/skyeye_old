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
* @file skyeye_mach.c
* @brief 
* @author Michael.Kang blackfin.kang@gmail.com
* @version 78.77
* @date 2011-12-21
*/

#include <skyeye_types.h>
#include <skyeye_class.h>
#include <skyeye_addr_space.h>
#include <skyeye_mach.h>
#include "s3c6410x.h"
#include "skyeye_internal.h"
#include <skyeye_interface.h>
#include <skyeye_lcd_intf.h>
//#define DEBUG
#include <skyeye_log.h>
#include <skyeye_uart.h>
#include <skyeye_arch.h>
#include <skyeye_mm.h>
#include <skyeye_core_intf.h>

typedef struct s3c6410_mach{
	conf_object_t* obj;
	s3c6410x_io_t* io;
	addr_space_t* space;
}s3c6410_mach_t;

static void
s3c6410x_update_int (s3c6410_mach_t* mach)
{
	s3c6410x_io_t* io = mach->io;
	core_signal_intf* signal_intf = (core_signal_intf*)SKY_get_interface(mach->obj, CORE_SIGNAL_INTF_NAME);
	interrupt_signal_t interrupt_signal;
	if (io->vic0fiqstatus != 0 || io->vic1fiqstatus != 0) {
		interrupt_signal.arm_signal.firq =  Low_level;
	} else {
		interrupt_signal.arm_signal.firq =  High_level;
	}

	if (io->vic0irqstatus != 0 || io->vic1irqstatus != 0) {
		interrupt_signal.arm_signal.irq =  Low_level;
	} else {
		interrupt_signal.arm_signal.irq =  High_level;
	}
	interrupt_signal.arm_signal.reset =  Prev_level;
	signal_intf->signal(signal_intf->obj, &interrupt_signal);
}

static void
s3c6410x_uart_read (s3c6410x_io_t* io, u32 offset, u32 *data, int index)
{
	switch (offset) {

	case ULCON:
		*data = io->uart[index].ulcon;
		break;
	case UCON:
		*data = io->uart[index].ucon;
		break;
	case UFCON:
		*data = io->uart[index].ufcon;
		break;
	case UMCON:
		*data = io->uart[index].umcon;
		break;
	case UTRSTAT:
		*data = io->uart[index].utrstat;
		break;
	case UERSTAT:
		*data = io->uart[index].uerstat;
		break;
	case UFSTAT:
		*data = io->uart[index].ufstat;
		break;
	case UMSTAT:
		*data = io->uart[index].umstat;
		break;
	case URXH:
		/* receive char
		 * */
		*data = io->uart[index].urxh;
		io->uart[index].utrstat &= (~0x1);	/* clear strstat register bit[0] */
		io->uart[index].ufstat &= ~(0x1); /* 2007-02-09 by Anthony Lee : for 0 bytes */
		break;
	case UBRDIV:
		*data = io->uart[index].ubrdiv;
		break;
	case UDIVSLOT:
		*data = io->uart[index].ubrdivslot;
		break;
	case UINTP:
		*data = io->uart[index].uintp;
		break;
	case UINTSP:
		*data = io->uart[index].uintsp;
		break;
	case UINTM:
		*data = io->uart[index].uintm;
		break;
	default:
		break;
	}
	SKYEYE_DBG ("%s(UART%d: 0x%x, 0x%x)\n", __FUNCTION__, index, offset, data);
	return;
}
static void
s3c6410x_timer_read (s3c6410x_io_t* io, u32 offset, u32 *data)
{
	switch (offset) {
	case TCFG0:
		*data = io->timer.tcfg0;
		break;
	case TCFG1:
		*data = io->timer.tcfg1;
		break;
	case TCON:
		*data = io->timer.tcon;
		break;
	case TCNTB0:
	case TCNTB1:
	case TCNTB2:
	case TCNTB3:
	case TCNTB4:
		{
			int n = (offset - 0xC) / 0xC;
			*data = io->timer.tcntb[n];
		}
		break;
	case TCMPB0:
	case TCMPB1:
		{
			int n = (offset - 0x10) / 0xC;
			*data = io->timer.tcmpb[n];
		}
		break;
	case TCNTO0:
	case TCNTO1:
	case TCNTO2:
	case TCNTO3:
		{
			int n = (offset - 0x10) / 0xC;
			*data = io->timer.tcnto[n];
		}
		break;
	case TCNTO4:
		*data = io->timer.tcnt[4];
		break;
	default:
		break;
	}
	return;
}

static exception_t mach_read(conf_object_t *opaque, generic_address_t addr, void* buf, size_t count){
	uint32 data;
	exception_t ret = No_exp;
	s3c6410_mach_t* mach = opaque->obj;
	addr_space_t* phys_mem = mach->space;
	s3c6410x_io_t* io = mach->io;
	//DBG("In %s, addr=0x%x\n", __FUNCTION__, addr);
	ret = phys_mem->memory_space->read(phys_mem->obj, addr, &data, count);
	/* Read the data successfully */
	if(ret == No_exp){
		*(uint32_t*)buf = data;
		return ret;
	}

	/* uart */
	if ((addr >= UART_CTL_BASE0)
	    && (addr < (UART_CTL_BASE0 + UART_CTL_SIZE))) {
		s3c6410x_uart_read (io, (u32) ((addr - UART_CTL_BASE0) % 0x400),
				    (u32 *) &data,
				    (addr - UART_CTL_BASE0) / 0x400);
		*(uint32_t*)buf = data;
		return ret;
	}
	if ((addr >= PWM_CTL_BASE) && (addr < (PWM_CTL_BASE + PWM_CTL_SIZE))) {
		s3c6410x_timer_read (io, (u32) (addr - PWM_CTL_BASE),
				     (u32 *) &data);
		*(uint32_t*)buf = data;
		return ret;
	}
	#define BOARD_ID 0x7e00f118
	switch (addr) {
	case BOARD_ID:
		data = 0x36410100;
		break;
	case VIC0IRQSTATUS:
		data = io->vic0irqstatus;
		io->vic0irqstatus = 0;
		s3c6410x_update_int(mach);
		break;
	case VIC0FIQSTATUS:
		data = io->vic0fiqstatus;
		io->vic0fiqstatus = 0;
		break;
	case VIC0RAWINTR:
		data = io->vic0rawintr;
		io->vic0rawintr = 0;
		break;
	case VIC0INTSELECT:
		data = io->vic0intselect;
		break;
	case VIC0INTENABLE:
		data = io->vic0intenable;
		break;

	case VIC0SOFTINT:
		data = io->vic0softint;
		break;
	case VIC0PROTECTION:
		data = io->vic0protection;
		break;
	case VIC0SWPRIORITYMASK:
		data = io->vic0swprioritymask;
		break;
	case VIC0PRIORITYDAISY:
		data = io->vic0prioritydaisy;
		break;

	case VIC1IRQSTATUS:
		data = io->vic1irqstatus;
		io->vic1irqstatus = 0;
		s3c6410x_update_int(mach);
		break;
	case VIC1FIQSTATUS:
		data = io->vic1fiqstatus;
		io->vic1fiqstatus = 0;
		break;
	case VIC1RAWINTR:
		data = io->vic1rawintr;
		io->vic1rawintr = 0;
		break;

	case VIC1INTSELECT:
		data = io->vic1intselect;
		break;
	case VIC1INTENABLE:
		data = io->vic1intenable;
		break;
	case VIC1INTENCLEAR:
		/* data = io.vic1intenclear; */
		data = io->vic1intenable &= ~data;
		break;
	case VIC1SOFTINT:
		data = io->vic1softint;
		break;
	case VIC1SOFTINTCLEAR:
		data = io->vic1softintclear;
		break;
	case VIC1PROTECTION:
		data = io->vic1protection;
		break;
	case VIC1SWPRIORITYMASK:
		data = io->vic1swprioritymask;
		break;
	case VIC1PRIORITYDAISY:
		data = io->vic1prioritydaisy;
		break;

	case APLL_CON:
                data = io->clkpower.apllcon;
		break;
	case MPLL_CON:
		data = io->clkpower.mpllcon;
		break;
	case EPLL_CON0:
		data = io->clkpower.epllcon0;
		break;
	case EPLL_CON1:
		data = io->clkpower.epllcon1;
		break;
	case CLK_SRC:
		data = io->clkpower.clksrc;
		break;
	case CLK_DIV0:
		data = io->clkpower.clkdiv0;
		break;
	case CLK_DIV1:
		data = io->clkpower.clkdiv1;
		break;
	case CLK_DIV2:
		data = io->clkpower.clkdiv2;
		break;
	default:
		/* fprintf(stderr, "ERROR: %s(0x%08x) \n", __FUNCTION__, addr); */
		if (addr - VIC0VECTADDR0 >= 0 && addr - VIC0VECTADDR0 <= 0x7c && (addr - VIC0VECTADDR0) & 0x3 == 0)
			data = io->vic0vectaddr[(addr - VIC0VECTADDR0)>>2];
		else
		if (addr - VIC1VECTADDR0 >= 0 && addr - VIC1VECTADDR0 <= 0x7c && (addr - VIC1VECTADDR0) & 0x3 == 0)
			data = io->vic1vectaddr[(addr - VIC0VECTADDR1)>>2];
		else
		if (addr - VIC0VECPRIORITY0 >= 0 && addr - VIC0VECPRIORITY0 <= 0x7c &&  (addr - VIC0VECPRIORITY0) & 0x3 == 0)
			data = io->vic0vecpriority[(addr - VIC0VECTADDR0)>>2];
		else
		if (addr - VIC1VECPRIORITY0 >= 0 && addr - VIC1VECPRIORITY0 <= 0x7c &&  (addr - VIC1VECPRIORITY0) & 0x3 == 0)
			data = io->vic1vecpriority[(addr - VIC1VECTADDR0)>>2];
 		//fprintf(stderr, "ERROR: %s(0x%08x) = 0x%08x\n", __FUNCTION__, addr ,data); 
		break;
	}
	*(uint32_t*)buf = data;
	return ret;
}

static void
s3c6410x_uart_write (s3c6410_mach_t* mach , u32 offset, u32 data, int index)
{
	s3c6410x_io_t* io = mach->io;
	SKYEYE_DBG ("%s(UART%d: 0x%x, 0x%x)\n", __FUNCTION__, index, offset, data);
	switch (offset) {
	case ULCON:
		io->uart[index].ulcon = data;
		break;
	case UCON:
		io->uart[index].ucon = data;
		break;
	case UFCON:
		io->uart[index].ufcon = data;
		break;
	case UMCON:
		io->uart[index].umcon = data;
		break;
	case UMSTAT:
		io->uart[index].umstat = data;
		break;
	case UTXH:
		{
			char c = data;
			/* 2007-01-18 modified by Anthony Lee : for new uart device frame */
			skyeye_uart_write(index, &c, 1, NULL);
			io->uart[index].utrstat |= 0x6;	/* set strstat register bit[0] */
			io->uart[index].ufstat &= 0xff;	/* set strstat register bit[0] */

			io->uart[index].uintp |= (0x4 & ~io->uart[index].uintm);
			io->uart[index].uintsp |= 0x4;
			{

				io->vic1rawintr	 |=  1 << (INT_UART0 + index);
				io->vic1irqstatus |=  ((1 << (INT_UART0 + index)) & ~(io->vic1intselect) & io->vic1intenable);
				io->vic1fiqstatus |=  ((1 << (INT_UART0 + index)) & io->vic1intselect & io->vic1intenable);
			}

			s3c6410x_update_int (mach);
		}
		break;
	case UBRDIV:
		io->uart[index].ubrdiv = data;
		break;
	case UDIVSLOT:
		io->uart[index].ubrdivslot = data;
		break;
	case UINTP:
		io->uart[index].uintp &= ~data;
		break;
	case UINTSP:
		io->uart[index].uintsp &= ~data;
		break;
	case UINTM:
		io->uart[index].uintm = data;
		{

			io->uart[index].uintp |= (0x4 & ~io->uart[index].uintm);
			io->uart[index].uintsp |= 0x4;
			{

				io->vic1rawintr	 |=  1 << (INT_UART0 + index);
				io->vic1irqstatus |=  ((1 << (INT_UART0 + index)) & ~(io->vic1intselect) & io->vic1intenable);
				io->vic1fiqstatus |=  ((1 << (INT_UART0 + index)) & io->vic1intselect & io->vic1intenable);
			}

			s3c6410x_update_int (mach);

		}
		break;
	default:
		break;
	}
	return;
}
static void
s3c6410x_timer_write (s3c6410_mach_t* mach, u32 offset, u32 data)
{
	s3c6410x_io_t* io = mach->io;
	switch (offset) {
	case TCFG0:
		io->timer.tcfg0 = data;
		break;
	case TCFG1:
		io->timer.tcfg1 = data;
		break;
	case TCON:
		{
			io->timer.tcon = data;
			if (io->timer.tcon) {
			}
		}
		break;
	case TCNTB0:
	case TCNTB1:
	case TCNTB2:
	case TCNTB3:
	case TCNTB4:
		{
			int n = (offset - 0xC) / 0xC;
			/* io.timer.tcntb[n] = data; */
			/* temp data taken from linux source */
			io->timer.tcntb[n] = 25350 / 20;
		}
		break;
	case TCMPB0:
	case TCMPB1:
#if 0
	case TCMPB2:
	case TCMPB3:
#endif
		{
			int n = (offset - 0x10) / 0xC;
			io->timer.tcmpb[n] = data;
		}
		break;
	default:
		break;
	}
	return;
}

static exception_t mach_write(conf_object_t *opaque, generic_address_t addr, uint32_t* buf, size_t count)
{
	exception_t ret = No_exp;
	uint32 data = *(uint32_t*)buf;
	s3c6410_mach_t* mach = opaque->obj;
	addr_space_t* phys_mem = mach->space;

	ret = phys_mem->memory_space->write(phys_mem->obj, addr, &data, count);
	/* Read the data successfully */
	if(ret == No_exp){
		return No_exp;
	}
	return No_exp;

	s3c6410x_io_t* io = mach->io;
	if ((addr >= UART_CTL_BASE0)
	    && (addr < UART_CTL_BASE0 + UART_CTL_SIZE)) {
#if 0
		s3c6410x_uart_write (state, (addr - UART_CTL_BASE0) % 0x4000,
				     data, (addr - UART_CTL_BASE0) / 0x4000);
#endif
		s3c6410x_uart_write (mach, (addr - UART_CTL_BASE0) % 0x400,
				     data, (addr - UART_CTL_BASE0) / 0x400);
		return No_exp;
	}

	if ((addr >= PWM_CTL_BASE) && (addr < (PWM_CTL_BASE + PWM_CTL_SIZE))) {
		s3c6410x_timer_write (mach, addr - PWM_CTL_BASE, data);
		return No_exp;
	}

	switch (addr) {

	case VIC0INTSELECT:
		io->vic0intselect = data;
		break;
	case VIC0INTENABLE:
		io->vic0intenable |= data;
		break;
	case VIC0INTENCLEAR:
		/* io.vic0intenclear = data; */
		/* write 1 clear the intenable register */
		io->vic0intenable &= ~data;
		break;
	case VIC0SOFTINT:
		io->vic0softint = data;
		break;
	case VIC0SOFTINTCLEAR:
		io->vic0softintclear = data;
		break;
	case VIC0PROTECTION:
		io->vic0protection = data;
		break;
	case VIC0SWPRIORITYMASK:
		io->vic0swprioritymask = data;
		break;
	case VIC0PRIORITYDAISY:
		io->vic0prioritydaisy = data;
		break;

	case VIC1INTSELECT:
		io->vic1intselect = data;
		break;
	case VIC1INTENABLE:
		io->vic1intenable |= data;
		break;
	case VIC1INTENCLEAR:
		/* write 1 clear the intenable register */
		io->vic1intenable &= ~data;
		break;
	case VIC1SOFTINT:
		io->vic1softint = data;
		break;
	case VIC1SOFTINTCLEAR:
		io->vic1softintclear = data;
		break;
	case VIC1PROTECTION:
		io->vic1protection = data;
		break;
	case VIC1SWPRIORITYMASK:
		io->vic1swprioritymask = data;
		break;
	case VIC1PRIORITYDAISY:
		io->vic1prioritydaisy = data;
		break;

	case APLL_CON:
		io->clkpower.apllcon = data;
		break;
	case MPLL_CON:
		io->clkpower.mpllcon = data;
		break;
	case EPLL_CON0:
		io->clkpower.epllcon0 = data;
		break;
	case EPLL_CON1:
		io->clkpower.epllcon1 = data;
		break;
	case CLK_SRC:
		io->clkpower.clksrc = data;
		break;
	case CLK_DIV0:
		io->clkpower.clkdiv0 = data;
		break;
	case CLK_DIV1:
		io->clkpower.clkdiv1 = data;
		break;
	case CLK_DIV2:
		io->clkpower.clkdiv2 = data;
		break;

	default:
		if (addr - VIC0VECTADDR0 >= 0 && addr - VIC0VECTADDR0 <= 0x7c &&  (addr - VIC0VECTADDR0) & 0x3 == 0)
			io->vic0vectaddr[(addr - VIC0VECTADDR0)>>2] = data;
		else
		if (addr - VIC1VECTADDR0 >= 0 && addr - VIC1VECTADDR0 <= 0x7c &&  (addr - VIC1VECTADDR0) & 0x3 == 0)
			io->vic1vectaddr[(addr - VIC0VECTADDR1)>>2] = data;
		else
		if (addr - VIC0VECPRIORITY0 >= 0 && addr - VIC0VECPRIORITY0 <= 0x7c &&  (addr - VIC0VECPRIORITY0) & 0x3 == 0)
			io->vic0vecpriority[(addr - VIC0VECTADDR0)>>2] = data;
		else
		if (addr - VIC1VECPRIORITY0 >= 0 && addr - VIC1VECPRIORITY0 <= 0x7c &&  (addr - VIC1VECPRIORITY0) & 0x3 == 0)
			io->vic1vecpriority[(addr - VIC1VECTADDR0)>>2] = data;

/* 		SKYEYE_DBG ("io_write_word(0x%08x) = 0x%08x\n", addr, data); */
 		//fprintf(stderr, "ERROR: %s(0x%08x) = 0x%08x\n", __FUNCTION__, addr ,data); 
		break;
	}
	return ret;
}

/**
* @brief Initialization of mach
*
* @param obj_name
*
* @return 
*/
static conf_object_t* new_s3c6410_mach(char* obj_name){
	s3c6410_mach_t* mach = skyeye_mm_zero(sizeof(s3c6410_mach_t));
	mach->obj = new_conf_object(obj_name, mach);
	s3c6410x_io_t* io =  skyeye_mm_zero(sizeof(s3c6410x_io_t));
	/* uart reigsters reset */
	int i;
	#define UART_UTRSTAT_INIT	0x6
	for (i = 0; i < 3; i++) {
		io->uart[i].utrstat = UART_UTRSTAT_INIT;
	}

	io->tc_prescale = 1;
	/* time reigsters reset */
	io->timer.tcfg0 = 0x101;

	/* clock register reset */
	io->clkpower.locktime = 0x00FFFFFF;
	io->clkpower.apllcon = 0x01900302;
	io->clkpower.mpllcon = 0x02140603;
	io->clkpower.epllcon0 = 0x00200102;
	io->clkpower.epllcon1 = 0x00009111;
	io->clkpower.clkdiv0 = 0x01051000;

	mach->io = io;
	mach->space = new_addr_space("s3c6410_mach_space_1");

	core_signal_intf* core_signal = skyeye_mm_zero(sizeof(core_signal_intf));
	core_signal->obj = NULL;
	core_signal->signal = NULL;
	SKY_register_interface(core_signal, obj_name, CORE_SIGNAL_INTF_NAME);

	/* Register io function to the object */
	memory_space_intf* mach_space = skyeye_mm_zero(sizeof(memory_space_intf));
	mach_space->conf_obj = mach->obj;
	mach_space->read = mach_read;
	mach_space->write = mach_write;
	SKY_register_interface(mach_space, obj_name, MEMORY_SPACE_INTF_NAME);

	/* Register sysctrl */
	conf_object_t* sysctrl = pre_conf_obj("s3c6410_sysctrl_1", "s3c6410_sysctrl");
	memory_space_intf* sysctrl_io_memory = (memory_space_intf*)SKY_get_interface(sysctrl, MEMORY_SPACE_INTF_NAME);
	DBG("In %s, get the interface instance 0x%x\n", __FUNCTION__, lcd_io_memory);
	exception_t ret;
       	ret = add_map(mach->space, 0x7e00f000, 0x1000, 0x0, sysctrl_io_memory, 1, 1);
	if(ret != No_exp){
		skyeye_log(Error_log, __FUNCTION__, "Can not register io memory for system controller\n");
	}

	/* register gpio */
	conf_object_t* gpio = pre_conf_obj("s3c6410_gpio_1", "s3c6410_gpio");
	memory_space_intf* gpio_io_memory = (memory_space_intf*)SKY_get_interface(gpio, MEMORY_SPACE_INTF_NAME);
	DBG("In %s, get the interface instance 0x%x\n", __FUNCTION__, gpio_io_memory);
       	ret = add_map(mach->space, 0x7f008000, 0x1000, 0x0, gpio_io_memory, 1, 1);
	if(ret != No_exp){
		skyeye_log(Error_log, __FUNCTION__, "Can not register io memory for gpio\n");
	}

	/* register touchscreen */
	conf_object_t* touchscreen = pre_conf_obj("s3c6410_touchscreen_1", "s3c6410_touchscreen");
	memory_space_intf* ts_io_memory = (memory_space_intf*)SKY_get_interface(touchscreen, MEMORY_SPACE_INTF_NAME);
	DBG("In %s, get the interface instance 0x%x\n", __FUNCTION__, ts_io_memory);
	ret = add_map(mach->space, 0x7f00b000, 0x1000, 0x0, ts_io_memory, 1, 1);
	if(ret != No_exp){
		skyeye_log(Error_log, __FUNCTION__, "Can not register io memory for touchscreen\n");
	}
	/* Register the ram shared with the other */
	#if 1
	conf_object_t* ram = pre_conf_obj("ram_image_1", "ram_image");
	memory_space_intf* ram_space = (memory_space_intf*)SKY_get_interface(ram, MEMORY_SPACE_INTF_NAME);
	ret = add_map(mach->space, 0x50000000, 0x20000000, 0x0, ram_space, 1, 1);
	if(ret != No_exp){
		skyeye_log(Error_log, __FUNCTION__, "Can not register address space for ram\n");
	}
	#endif

	/* Register lcd io memory to the whole address space */
	conf_object_t* lcd = pre_conf_obj("s3c6410_lcd_1", "s3c6410_lcd");
	if(lcd != NULL){
		memory_space_intf* lcd_io_memory = (memory_space_intf*)SKY_get_interface(lcd, MEMORY_SPACE_INTF_NAME);
		DBG("In %s, get the interface instance 0x%x\n", __FUNCTION__, lcd_io_memory);
		exception_t ret;
		//ret = add_map(mach->space, 0x77100000, 0x100000, 0x0, lcd_io_memory, 1, 1);
#ifdef GTK_LCD
		/* set the lcd_ctrl_0 attribute for lcd */
		conf_object_t* gtk_painter = pre_conf_obj("gtk_lcd_1", "gtk_lcd");

		lcd_control_intf* lcd_ctrl = (lcd_control_intf*)SKY_get_interface(gtk_painter, LCD_CTRL_INTF_NAME);
		attr_value_t* attr = make_new_attr(Val_ptr);
		attr->u.ptr = lcd_ctrl;
		/* set the attribute of lcd */
		SKY_set_attr(lcd, "lcd_ctrl_0", attr);
#endif
	}
	else{
		SKYEYE_WARNING("can not initlize the lcd, maybe the module not exist\n");
	}

	return mach->obj;
}
void free_s3c6410_mach(conf_object_t* mach){
}
void init_s3c6410_mach(){
	static skyeye_class_t class_data = {
		.class_name = "s3c6410_mach",
		.class_desc = "s3c6410 machine",
		.new_instance = new_s3c6410_mach,
		.free_instance = free_s3c6410_mach,
		.get_attr = NULL,
		.set_attr = NULL
	};
	
	SKY_register_class(class_data.class_name, &class_data);
}
