/*
 * Copyright (C)
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

/*
 * @file touchscreen_s3c6410.c
 * @brief The implementation of touchscreen
 * @author David.Yu keweihk@gmail.com
 * @version 78.77
 * @date 2012-2-2
 */

#include <skyeye_types.h>
#include <skyeye_sched.h>
#include <skyeye_signal.h>
#include <skyeye_class.h>
#include <skyeye_interface.h>
#include <skyeye_obj.h>
#include <skyeye_mm.h> 
#include <memory_space.h>
#include <skyeye_device.h>

#include "touchscreen_s3c6410.h"

static exception_t s3c6410_touchscreen_raise(conf_object_t* object, int line_no)
{
	/* Get vic interrupt interface */
	general_signal_intf* vic_signal = (general_signal_intf*)SKY_get_interface(object, GENERAL_SIGNAL_INTF_NAME);
	vic_signal->raise_signal(vic_signal->conf_obj, line_no);

	return 0;
}

static exception_t s3c6410_touchscreen_down(conf_object_t* object, int line_no)
{
	return 0;
}


static exception_t adc_interrupt(conf_object_t* object)
{
		s3c6410_touchscreen_raise(object, 63);

		return 0;
}

static void touchscreen_update_status(conf_object_t* object, int *Pen_buffer)
{
	s3c6410_touchscreen_device* dev_ts = (s3c6410_touchscreen_device*)object->obj;
	touchscreen_reg_t* regs = dev_ts->regs;
	s3c6410_touchscreen_status* status = dev_ts->status;

	/**************************************************************************************
	The origin of touchscreen coordinates and the LCD coordinate transformation as follows:
	XT = ((XL * (XTmax - XTmin)) / W + XTmin
	YT = ((YL * (YTmax - YTmin)) / W + YTmin
	XT for the origin of touch screen  coordinates.
	XL for the LCD coordinate.
	Xtmin, Xtmax, Ytmin, Ytmax:The minimum and maximum values on the Y-axis direction for touchscreen.
	W for the LCD's width.
	H for the LCD's height.
	***************************************************************************************/

	int xmin = 2; //XTmin
	int xrange = 1023; //XTmax - XTmin
	int ymin = 2; // YTmin
	int yrange = 1023;//YTmax - YTmin
	//int lcd_width = 800;//W
	int lcd_width = 320;//W
	//int lcd_height = 480;// H
	int lcd_height = 480;// H
	
	int lcd_x;// XL
	int lcd_y;// YL

	int stylus = status->stylus;
	lcd_x = Pen_buffer[0];
	lcd_y = Pen_buffer[1];

	//coordinate transformation
	status->x = ((lcd_x * xrange) / lcd_width) + xmin;
	status->y = ((lcd_y * yrange) / lcd_height) + ymin;

	status->event = Pen_buffer[4];
	status->stylus = Pen_buffer[5];
	// set touchscreen status in regs
	DEBUG_TS("x = %d, y = %d, envent = %d, stylus = %d\n",
		status->x, status->y, status->event, status->stylus);

	if(regs->adctsc == 0xd3 && stylus != status->stylus){
		/* send interrupt */
		s3c6410_touchscreen_raise(object, 62);
		/* set ADCUPDN */
		if(status->stylus)
			regs->adcupdn &= 0x2; /* Stylus Up Interrupt history */
		else
			regs->adcupdn &= 0x1; /* Stylus Down Interrupt history */

	}
}

#if 0
static void skPenEvent(int *buffer, int eventType, int stateType, int x, int y)
{
	buffer[0] = x;
	buffer[1] = y;
	buffer[2] = 0;          // dx
	buffer[3] = 0;          // dy
	buffer[4] = eventType;  // event from pen (DOWN,UP,CLICK : 0, MOVE : 1)
	buffer[5] = stateType;  // state of pen (DOWN : 0,UP : 1,ERROR)
	buffer[6] = 1;          // no of the event
	buffer[7] = 0;          // time of the event (ms) since ts_open
}

static void callback_button_press(GtkWidget *w, GdkEventButton *event, gpointer object)
{
	int *Pen_buffer;
	Pen_buffer = get_pen_buffer();
	skPenEvent(Pen_buffer, 0, 0, event->x, event->y);
	touchscreen_update_status((conf_object_t*)object, Pen_buffer);
}


static void callback_button_release(GtkWidget *w, GdkEventButton *event, gpointer object)
{
	int *Pen_buffer;
	Pen_buffer = get_pen_buffer();
	skPenEvent(Pen_buffer, 0, 1, event->x, event->y);
	touchscreen_update_status((conf_object_t*)object, Pen_buffer);
}

static void callback_motion_notify(GtkWidget *w, GdkEventMotion *event, gpointer object)
{
	int *Pen_buffer;
	Pen_buffer = get_pen_buffer();
	/*
	 * when mouse is moving, generate an skyeye pen motion event
	 * should changed to "when mouse is pressed and moving"
	 */
	if (Pen_buffer[5] == 0){
		skPenEvent(Pen_buffer, 1, 0, event->x, event->y);
		touchscreen_update_status((conf_object_t*)object, Pen_buffer);
		/*
		if(regs->adctsc & TSC_AUTO_PST)
			s3c6410_touchscreen_raise(63);
		*/
	}
}
#endif

static exception_t s3c6410_touchscreen_read(conf_object_t *opaque, generic_address_t offset, void* buf, size_t count)
{
	struct s3c6410_touchscreen_device *dev = opaque->obj;
	s3c6410_touchscreen_status* status = dev->status;
	touchscreen_reg_t* regs = dev->regs;
	DBG("In %s, offset=0x%x\n", __FUNCTION__, offset);
	switch(offset) {
		case 0x0:
			/* End of adc conversion */
			regs->adccon |= ECFLG;
			*(uint32_t*)buf = regs->adccon; // 0x7E00_B000
			break;
		case 0x4:
			*(uint32_t*)buf = regs->adctsc; // 0x7E00_B004
			break;
		case 0x8:
			*(uint32_t*)buf = regs->adcdly; // 0x7E00_B008
			break;
		case 0x0C:
			if(status->adcbit)
				regs->adcdat0 = (status->x & 0x3ff) | (regs->adcdat0 & ~0x3ff);
			else
				regs->adcdat0 = (status->x & 0xfff) | (regs->adcdat0 & ~0xfff);

			if(status->stylus == 0)		// DOWN and set regs
				regs->adcdat0 &= ~(1 << 15);
			else
				regs->adcdat0 |= (status->stylus << 15);

			*(uint32_t*)buf = regs->adcdat0; // 0x7E00_B00C
			break;
		case 0x10:
			if(status->adcbit)
				regs->adcdat1 = (status->y & 0x3ff) | (regs->adcdat1 & ~0x3ff);
			else
				regs->adcdat1 = (status->y & 0xfff) | (regs->adcdat1 & ~0xfff);;

			if(status->stylus == 0)		// DOWN and set regs
				regs->adcdat1 &= ~(1 << 15);
			else
				regs->adcdat1 |= (status->stylus << 15);
			*(uint32_t*)buf = regs->adcdat1; // 0x7E00_B010
			break;
		case 0x14:
			*(uint32_t*)buf = regs->adcupdn; // 0x7E00_B014
			break;
		case 0x18:
			printf("ADCCLRINT is Write Only\n");
			break;
		case 0x20:
			printf("ADCCLRINTPNDNUP is Write Only\n");
			break;
		default:
			printf("Can not read the register at 0x%x in touchscreen\n", offset);
			return 0;
	}
	DEBUG_TS("In %s, offset=0x%x data=0x%x\n",
			__FUNCTION__, offset, *(uint32_t*)buf);

	return No_exp;
}

static exception_t s3c6410_touchscreen_write(conf_object_t *opaque, generic_address_t offset, uint32_t* buf, size_t count)
{
	struct s3c6410_touchscreen_device *dev = opaque->obj;
	s3c6410_touchscreen_status* status = dev->status;
	touchscreen_reg_t* regs = dev->regs;
	uint32_t val = *(uint32_t*)buf;
	DEBUG_TS("In %s, offset=0x%x data=0x%x\n",
			__FUNCTION__, offset, val);
	switch(offset) {
		case 0x0:
			regs->adccon = val; // 0x7E00_B000
			status->adcbit = GET_RESSEL(regs->adccon);
			if((regs->adctsc & TSC_AUTO_PST) &&
					(regs->adccon & ENABLE_START)){
				if(status->adc_con < 0){
					create_thread_scheduler(1000, Periodic_sched,
							adc_interrupt, opaque, &status->adc_con);
				}
			/*
			 * TSC_AUTO_PST is open, waiting for opening ENABLE_START,
			 * conversion start, but adc conversion that we need not simulate
			 */
			}
			regs->adccon &= ~ENABLE_START;
			break;
		case 0x4:
			regs->adctsc = val; // 0x7E00_B004
			if(~(regs->adctsc & TSC_AUTO_PST)){
				if(status->adc_con >= 0){
					del_thread_scheduler(status->adc_con);
					status->adc_con = -1;
				}
			}
			break;
		case 0x8:
			regs->adcdly = val; // 0x7E00_B008
			break;
		case 0x0C:
			regs->adcdat0 = val; // 0x7E00_B00C
			break;
		case 0x10:
			regs->adcdat1 = val; // 0x7E00_B010
			break;
		case 0x14:
			regs->adcupdn = val; // 0x7E00_B014
			break;
		case 0x18:
			regs->adcclrint = val; // 0x7E00_B018
			break;
		case 0x20:
			regs->adcclrintpndnup = val; // 0x7E00_B020
			break;
		default:
			printf("Can not write the register at 0x%x in touchscreen\n", offset);
			return 0;
	}
	return No_exp;
}
static conf_object_t* new_s3c6410_touchscreen(char* obj_name){
	s3c6410_touchscreen_device* dev_ts = skyeye_mm_zero(sizeof(s3c6410_touchscreen_device));
	touchscreen_reg_t* regs =  skyeye_mm_zero(sizeof(touchscreen_reg_t));
	s3c6410_touchscreen_status* status = skyeye_mm_zero(sizeof(s3c6410_touchscreen_status));
	dev_ts->obj = new_conf_object(obj_name, dev_ts);
	int *Pen_buffer = get_pen_buffer();
	Pen_buffer[5] = 1;
	status->stylus = Pen_buffer[5];
	status->adc_con = -1;
	/* init touchscreen regs */
	regs->adccon = 0x3fc4;
	regs->adctsc = 0x0058;
	regs->adcdly = 0x00ff;
	regs->adcupdn = 0x0000;
	dev_ts->regs = regs;
	dev_ts->status = status;
	/* registe lcd_ts callback functions */
	lcd_touchscreen_t* lcd_ts = skyeye_mm_zero(sizeof(lcd_touchscreen_t));
	lcd_ts->touchscreen_update_status = touchscreen_update_status;
	lcd_ts->obj = dev_ts->obj;
	SKY_register_interface(lcd_ts, obj_name, LCD_TS_INTF_NAME);
	/* Register io function to the object */
	memory_space_intf* io_memory = skyeye_mm_zero(sizeof(memory_space_intf));
	io_memory->conf_obj = dev_ts->obj;
	io_memory->read = s3c6410_touchscreen_read;
	io_memory->write = s3c6410_touchscreen_write;
	SKY_register_interface(io_memory, obj_name, MEMORY_SPACE_INTF_NAME);

	return dev_ts->obj;
}
void free_s3c6410_touchscreen(conf_object_t* dev){
	
}

void init_s3c6410_touchscreen(){
	static skyeye_class_t class_data = {
		.class_name = "s3c6410_touchscreen",
		.class_desc = "s3c6410 touchscreen",
		.new_instance = new_s3c6410_touchscreen,
		.free_instance = free_s3c6410_touchscreen,
		.get_attr = NULL,
		.set_attr = NULL
	};

	SKY_register_class(class_data.class_name, &class_data);
}
