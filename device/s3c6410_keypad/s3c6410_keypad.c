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
* @file s3c6410_keypad.c
* @brief The implementation of system controller
* @author xq2537@gmail.com
* @version 78.77
*/

/**
The Key Pad Interface block in S3C6410X facilitates communication with external keypad devices. The ports
multiplexed with GPIO ports provide up to 8 rows and 8 columns. The events of key press or key release are
detected to the CPU by an interrupt. When any of the interrupt from row lines occurs, the software will scan the
column lines using the proper procedure to detect one or multiple key press or release.
**/

#include <skyeye_types.h>
#include <skyeye_sched.h>
#include <skyeye_signal.h>
#include <skyeye_class.h>
#include <skyeye_interface.h>
#include <skyeye_keypad_intf.h>
#include <skyeye_obj.h>
#include <skyeye_mm.h> 
#include <memory_space.h>
#include <skyeye_device.h>
#include <skyeye_addr_space.h>
#define DEBUG
#include <skyeye_log.h>

#include "s3c6410_keypad.h"
#include "android/hw-events.h"

#define SAMSUNG_MAX_ROWS        8
#define SAMSUNG_MAX_COLS        8
#define SAMSUNG_MAX_KEY_COUNTS        64
#define KEY(row, col, val)	{row,col,val}

/*GPIOL for col output ;GPIOK for col input*/
#define GPIOLDAT 0x7F008818 
#define GPIOKDAT 0x7F008808 

typedef enum {
	GPIOL,
	GPIOK
}GpioType;

/**if we press a key,the key's col and row will connected together.
   status mark the the two pin is connected or not. **/
typedef struct {
	int row;
	int col;
	int status;
}Connect;

static Connect connect; 

/*We need layout the keypad refer to the driver*/
typedef struct {
	int row;
	int col;
	int code;
}KeypadLayout;


/*The key layout,just for android*/
static KeypadLayout key_layout[SAMSUNG_MAX_KEY_COUNTS] = {
	/* KEY(row, col, keycode) */
	KEY(0, 0, KEY_ESC), KEY(0, 1, KEY_1),
	KEY(0, 2, KEY_2), KEY(0, 3, KEY_3),
	KEY(0, 4, KEY_4), KEY(0, 5, KEY_5),
	KEY(0, 6, KEY_6), KEY(0, 7, KEY_7),

	KEY(1, 0, KEY_Q), KEY(1, 1, KEY_W),
	KEY(1, 2, KEY_E), KEY(1, 3, KEY_R),
	KEY(1, 4, KEY_T), KEY(1, 5, KEY_Y),
	KEY(1, 6, KEY_U), KEY(1, 7, KEY_I),

	KEY(2, 0, KEY_O), KEY(2, 1, KEY_P),
	KEY(2, 2, KEY_A), KEY(2, 3, KEY_S),
	KEY(2, 4, KEY_D), KEY(2, 5, KEY_F),
	KEY(2, 6, KEY_G), KEY(2, 7, KEY_H),

	KEY(3, 0, KEY_J), KEY(3, 1, KEY_K),
	KEY(3, 2, KEY_L), KEY(3, 3, KEY_BACKSPACE),
	KEY(3, 4, KEY_LEFTSHIFT), KEY(3, 5, KEY_Z),
	KEY(3, 6, KEY_X), KEY(3, 7, KEY_C),

	KEY(4, 0, KEY_V), KEY(4, 1, KEY_B),
	KEY(4, 2, KEY_N), KEY(4, 3, KEY_M),
	KEY(4, 4, KEY_DOT), KEY(4, 5, KEY_ENTER),
	KEY(4, 6, KEY_LEFTALT), KEY(4, 7, KEY_COMPOSE),

	KEY(5, 0, KEY_EMAIL), KEY(5, 1, KEY_SPACE),
	KEY(5, 2, KEY_SLASH), KEY(5, 3, KEY_COMMA),
	KEY(5, 4, KEY_RIGHTALT), KEY(5, 5, KEY_VOLUMEDOWN),
	KEY(5, 6, KEY_VOLUMEUP), KEY(5, 7, KEY_POWER),

	KEY(6, 0, KEY_SOUND), KEY(6, 1, KEY_UP),
	KEY(6, 2, KEY_DOWN), KEY(6, 3, KEY_LEFT),
	KEY(6, 4, KEY_RIGHT), KEY(6, 5, KEY_REPLY),
	KEY(6, 6, KEY_END), KEY(6, 7, KEY_HOME),

	KEY(7, 0, KEY_MENU), KEY(7, 1, KEY_BACK),
	KEY(7, 2, KEY_SEARCH), KEY(7, 3, KEY_8),
	KEY(7, 4, KEY_9), KEY(7, 5, KEY_0)
};

/*We use matrix to scan the key layout.So GPIOL for the matrix col and GPIOK for the matrix row.*/
static void write_gpio(uint32_t var,GpioType gpio)
{
	uint32 data = 0;
	exception_t ret;

	conf_object_t* conf_obj = get_conf_obj("s3c6410_mach_space");
	addr_space_t* phys_mem = (addr_space_t*)conf_obj->obj;

	if (gpio == GPIOL){
		ret = phys_mem->memory_space->read(conf_obj, GPIOLDAT, &data, 4);
		data = data & var;
		ret = phys_mem->memory_space->write(conf_obj,GPIOLDAT, &data, 4);

		if (var == (SAMSUNG_KEYIFCOL_MASK & (~(1 << connect.col))) && connect.status)
		{
			ret = phys_mem->memory_space->read(conf_obj, GPIOKDAT, &data, 4);
			data =  data & (~(1 << connect.row)) << 8;
			ret = phys_mem->memory_space->write(conf_obj,GPIOKDAT,&data, 4);
			connect.status = 0;//disconnet the tow pin
		}
	}
	else{
		ret = phys_mem->memory_space->read(conf_obj, GPIOKDAT, &data, 4);
		data = data & var << 8;
		ret = phys_mem->memory_space->write(conf_obj,GPIOKDAT, &data, 4);
	}
}

static void reset_gpio()
{
	uint32 data = 0;
	exception_t ret;
	conf_object_t* conf_obj = get_conf_obj("s3c6410_mach_space");
	addr_space_t* phys_mem = (addr_space_t*)conf_obj->obj;
	ret = phys_mem->memory_space->write(conf_obj,GPIOLDAT, &data, 4);
	data = 0xff << 8;
	ret = phys_mem->memory_space->write(conf_obj,GPIOKDAT, &data, 4);
}

static uint32_t read_gpio(GpioType gpio)
{
	uint32_t data;
	conf_object_t* conf_obj = get_conf_obj("s3c6410_mach_space");
	addr_space_t* phys_mem = (addr_space_t*)conf_obj->obj;

	if (gpio == GPIOK){
		exception_t ret = phys_mem->memory_space->read(conf_obj, GPIOKDAT, &data, 4);
		reset_gpio();
		return (data >> 8) & 0xff;
	}
	return 0xff;
}

static void keypad_update_status(conf_object_t* object, int code)
{
	int i = 0;
	int support_key = 0;
	int status = 0;

	struct s3c6410_keypad_device *dev = object->obj;
	s3c6410_keypad_reg_t* regs = dev->regs;

	for (i = 0;i < SAMSUNG_MAX_KEY_COUNTS;i++)
	{
		if ((code & 0x1ff) == key_layout[i].code)
		{
			support_key = 1;
			break;
		}
	}

	if (support_key)
	{
		connect.row = key_layout[i].row;
		connect.col = key_layout[i].col;
		connect.status = 1;
		support_key = 0;

		/*KEYPAD input “press” interrupts (falling edge) status*/
		if ((code&0x200) ? 1 : 0)
		{
			dev->master->lower_signal(dev->master->conf_obj, dev->line_no);
			regs->keyifstsclr &= 1; 
		}
		else
		{
			/*KEYPAD input “release” interrupts (rising edge) status*/
			dev->master->raise_signal(dev->master->conf_obj, dev->line_no);
			regs->keyifstsclr &= 1 << SAMSUNG_KEYIFSTSCLR_R_INT_OFFSET; 
		}

	}
	else{
		fprintf(stderr, "in %s the key[%d] not support,you can read android/hw-events.h\n", __FUNCTION__,code & 0x1ff);
	}

}

static exception_t s3c6410_keypad_read(conf_object_t *opaque, generic_address_t offset, void* buf, size_t count)
{
	struct s3c6410_keypad_device *dev = opaque->obj;
	s3c6410_keypad_reg_t* regs = dev->regs;
	switch(offset) {
		case 0x0:
			*(uint32_t*)buf = regs->keyifcon;
			break;
		case 0x4:
			*(uint32_t*)buf = regs->keyifstsclr;
			break;
		case 0x8: // nothing to do
			*(uint32_t*)buf = regs->keyifcol;
			break;
		case 0xc:
			regs->keyifrow = read_gpio(GPIOK);
			*(uint32_t*)buf = regs->keyifrow;
			break;
		case 0x10:
			*(uint32_t*)buf = regs->keyiffc;
			break;
		default:
			printf("Can not read the register at 0x%x in s3c6410_keypad\n", offset);
			return Invarg_exp;
	}
	return No_exp;
}

static exception_t s3c6410_keypad_write(conf_object_t *opaque, generic_address_t offset, uint32_t* buf, size_t count)
{
	struct s3c6410_keypad_device *dev = opaque->obj;
	s3c6410_keypad_reg_t* regs = dev->regs;
	uint32_t val = *(uint32_t*)buf;
	switch(offset) {
		case 0x0:
			regs->keyifcon = val;
			break;
		case 0x4:
			//regs->keyifstsclr = val;
			regs->keyifstsclr &= 1 << SAMSUNG_KEYIFSTSCLR_R_INT_OFFSET;
			break;
		case 0x8:
			regs->keyifcol = val;
			write_gpio(regs->keyifcol,GPIOL);
			break;
		case 0xc:
			regs->keyifrow = val;
			break;
		case 0x10:
			regs->keyiffc = val;
			break;
		default:
			printf("Can not write the register at 0x%x in s3c6410_keypad\n", offset);
			return Invarg_exp;
	}
	return No_exp;
}

static conf_object_t* new_s3c6410_keypad(char* obj_name){
	s3c6410_keypad_device* dev = skyeye_mm_zero(sizeof(s3c6410_keypad_device));
	s3c6410_keypad_reg_t* regs =  skyeye_mm_zero(sizeof(s3c6410_keypad_reg_t));
	dev->obj = new_conf_object(obj_name, dev);
	/* init s3c6410_keypad regs */
	dev->regs = regs;
	regs->keyifcol = 0x0000FF00;
	regs->keyifcon = SAMSUNG_KEYIFCON_INT_F_EN | SAMSUNG_KEYIFCON_INT_R_EN;;
	regs->keyifstsclr &= 1 << SAMSUNG_KEYIFSTSCLR_R_INT_OFFSET;

	lcd_keypad_t* lcd_keypad = skyeye_mm_zero(sizeof(lcd_keypad_t));
	lcd_keypad->keypad_update_status = keypad_update_status;
	lcd_keypad->obj = dev->obj;
	SKY_register_interface(lcd_keypad, obj_name, LCD_KEYPAD_INTF_NAME);

	/* Register io function to the object */
	memory_space_intf* io_memory = skyeye_mm_zero(sizeof(memory_space_intf));
	io_memory->conf_obj = dev->obj;
	io_memory->read = s3c6410_keypad_read;
	io_memory->write = s3c6410_keypad_write;
	SKY_register_interface(io_memory, obj_name, MEMORY_SPACE_INTF_NAME);	

	general_signal_intf* keypad_signal = skyeye_mm_zero(sizeof(general_signal_intf));
	keypad_signal->conf_obj = NULL;
	keypad_signal->raise_signal = NULL;
	keypad_signal->lower_signal = NULL;
	dev->master = keypad_signal;
	dev->line_no = 22; /* Frame sync */
	SKY_register_interface(keypad_signal, obj_name, GENERAL_SIGNAL_INTF_NAME);

	reset_gpio();
	return dev->obj;
}
void free_s3c6410_keypad(conf_object_t* dev){
	
}

void init_s3c6410_keypad(){
	static skyeye_class_t class_data = {
		.class_name = "s3c6410_keypad",
		.class_desc = "s3c6410_keypad",
		.new_instance = new_s3c6410_keypad,
		.free_instance = free_s3c6410_keypad,
		.get_attr = NULL,
		.set_attr = NULL
	};

	SKY_register_class(class_data.class_name, &class_data);
}
