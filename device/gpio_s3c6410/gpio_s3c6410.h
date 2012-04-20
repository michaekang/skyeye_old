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
* @file gpio_s3c6410.h
* @brief The definition of system controller for s3c6410
* @author Michael.Kang blackfin.kang@gmail.com
* @version 78.77
* @date 2011-12-12
*/

#ifndef __SYSCTRL_S3C6410_H__
#define __SYSCTRL_S3C6410_H__

typedef struct gpio_reg{
	uint32_t gpbcon; // 0x7E00_B000
	uint32 eint0mask;
	uint32 eint0pend;
	uint32 eint0con0;
	uint32 gpfcon;
	uint32 gpfdat;
	uint32 gpfpud;

	uint32 gphcon0;
	uint32 gphcon1;
	uint32 gphdat;

	uint32 gpicon;
	uint32 gpjcon;

	uint32 gpkcon0;
	uint32 gpkcon1;
	uint32 gpkdat;
	uint32 gpkpud;

	uint32 gplcon0;
	uint32 gplcon1;
	uint32 gpldat;
	uint32 gplpud;

	uint32 gpncon;
	uint32 gpndat;

}gpio_reg_t; 

typedef struct s3c6410_gpio_device{
	conf_object_t* obj;
	gpio_reg_t* regs;
	int line_no;
	conf_object_t* signal_target;
	general_signal_intf* master;
}s3c6410_gpio_device;

#endif
