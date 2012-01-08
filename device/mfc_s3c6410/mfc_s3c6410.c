/* Copyright (C) 
* 2012 - xq2537@gmail.com
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
* @file mfc_s3c6410.c
* @brief The implementation of system controller
* @author xq2537@gmail.com
* @version 78.77
* @date 2012-1-8
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
#define DEBUG
#include <skyeye_log.h>

#include "mfc_s3c6410.h"

static exception_t s3c6410_mfc_read(conf_object_t *opaque, generic_address_t offset, void* buf, size_t count)
{
	struct s3c6410_mfc_device *dev = opaque->obj;
	mfc_reg_t* regs = dev->regs;
	switch(offset) {
		case 0x0:
			*(uint32_t*)buf = regs->mfc_coderun;
			break;
		case 0x4:
			*(uint32_t*)buf = regs->mfc_codedownload;
			break;
		case 0x8:
			*(uint32_t*)buf = regs->mfc_hostintreq;
			break;
		case 0xc:
			*(uint32_t*)buf = regs->mfc_bitintclear;
			break;
		case 0x10:
			*(uint32_t*)buf = regs->mfc_bitintsts;
			break;
		case 0x14:
			*(uint32_t*)buf = regs->mfc_bitcodereset;
			break;
		case 0x18:
			*(uint32_t*)buf = regs->mfc_bitcurpc;
			break;

		case 0x100:
			*(uint32_t*)buf = regs->mfc_codebufaddr;
			break;
		case 0x104:
			*(uint32_t*)buf = regs->mfc_workbufaddr;
			break;
		case 0x108:
			*(uint32_t*)buf = regs->mfc_parabufaddr;
			break;
		case 0x10c:
			*(uint32_t*)buf = regs->mfc_bitstreamctrl;
			break;
		case 0x110:
			*(uint32_t*)buf = regs->mfc_framememctrl;
			break;
		case 0x114:
			*(uint32_t*)buf = regs->mfc_decfunctrl;
			break;
		case 0x11c:
			*(uint32_t*)buf = regs->mfc_bitworkbufctrl;
			break;
		case 0x120:
			*(uint32_t*)buf = regs->mfc_bitstreamrdptr0;
			break;
		case 0x124:
			*(uint32_t*)buf = regs->mfc_bitstreamwrptr0;
			break;
		case 0x128:
			*(uint32_t*)buf = regs->mfc_bitstreamrdptr1;
			break;
		case 0x12c:
			*(uint32_t*)buf = regs->mfc_bitstreamwrptr1;
			break;
		case 0x130:
			*(uint32_t*)buf = regs->mfc_bitstreamrdptr2;
			break;
		case 0x134:
			*(uint32_t*)buf = regs->mfc_bitstreamwrptr2;
			break;
		case 0x138:
			*(uint32_t*)buf = regs->mfc_bitstreamrdptr3;
			break;
		case 0x13c:
			*(uint32_t*)buf = regs->mfc_bitstreamwrptr3;
			break;
		case 0x140:
			*(uint32_t*)buf = regs->mfc_bitstreamrdptr4;
			break;
		case 0x144:
			*(uint32_t*)buf = regs->mfc_bitstreamwrptr4;
			break;
		case 0x148:
			*(uint32_t*)buf = regs->mfc_bitstreamrdptr5;
			break;
		case 0x14c:
			*(uint32_t*)buf = regs->mfc_bitstreamwrptr5;
			break;
		case 0x150:
			*(uint32_t*)buf = regs->mfc_bitstreamrdptr6;
			break;
		case 0x154:
			*(uint32_t*)buf = regs->mfc_bitstreamwrptr6;
			break;
		case 0x158:
			*(uint32_t*)buf = regs->mfc_bitstreamrdptr7;
			break;
		case 0x15c:
			*(uint32_t*)buf = regs->mfc_bitstreamwrptr7;
			break;
		case 0x160:
			*(uint32_t*)buf = regs->mfc_busyflag;
			break;
		case 0x164:
			*(uint32_t*)buf = regs->mfc_runcommand;
			break;
		case 0x168:
			*(uint32_t*)buf = regs->mfc_runindex;
			break;
		case 0x16c:
			*(uint32_t*)buf = regs->mfc_runcodstd;
			break;
		case 0x170:
			*(uint32_t*)buf = regs->mfc_intenable;
			break;
		case 0x174:
			*(uint32_t*)buf = regs->mfc_reason;
			break;
		case 0x180:
			*(uint32_t*)buf = regs->mfc_command[0];
			break;
		case 0x184:
			*(uint32_t*)buf = regs->mfc_command[1];
			break;
		case 0x188:
			*(uint32_t*)buf = regs->mfc_command[2];
			break;
		case 0x18c:
			*(uint32_t*)buf = regs->mfc_command[3];
			break;
		case 0x190:
			*(uint32_t*)buf = regs->mfc_command[4];
			break;
		case 0x194:
			*(uint32_t*)buf = regs->mfc_command[5];
			break;
		case 0x198:
			*(uint32_t*)buf = regs->mfc_command[6];
			break;
		case 0x19c:
			*(uint32_t*)buf = regs->mfc_command[7];
			break;
		case 0x1a0:
			*(uint32_t*)buf = regs->mfc_command[8];
			break;
		case 0x1a4:
			*(uint32_t*)buf = regs->mfc_command[9];
			break;
		case 0x1a8:
			*(uint32_t*)buf = regs->mfc_command[10];
			break;
		case 0x1ac:
			*(uint32_t*)buf = regs->mfc_command[11];
			break;
		case 0x1b0:
			*(uint32_t*)buf = regs->mfc_command[12];
			break;
		case 0x1b4:
			*(uint32_t*)buf = regs->mfc_command[13];
			break;
		case 0x1b8:
			*(uint32_t*)buf = regs->mfc_command[14];
			break;
		case 0x1bc:
			*(uint32_t*)buf = regs->mfc_command[15];
			break;
		case 0x1c0:
			*(uint32_t*)buf = regs->mfc_command[16];
			break;
		case 0x1c4:
			*(uint32_t*)buf = regs->mfc_command[17];
			break;
		case 0x1c8:
			*(uint32_t*)buf = regs->mfc_command[18];
			break;
		case 0x1cc:
			*(uint32_t*)buf = regs->mfc_command[19];
			break;
		case 0x1d0:
			*(uint32_t*)buf = regs->mfc_command[20];
			break;
		case 0x1d4:
			*(uint32_t*)buf = regs->mfc_command[21];
			break;
		case 0x1d8:
			*(uint32_t*)buf = regs->mfc_command[22];
			break;
		default:
			printf("Can not read the register at 0x%x in mfc\n", offset);
			return Invarg_exp;
	}
	return No_exp;
}

static exception_t s3c6410_mfc_write(conf_object_t *opaque, generic_address_t offset, uint32_t* buf, size_t count)
{
	struct s3c6410_mfc_device *dev = opaque->obj;
	mfc_reg_t* regs = dev->regs;
	uint32_t val = *(uint32_t*)buf;
	switch(offset) {
		case 0x0:
			regs->mfc_coderun = val;
			break;
		case 0x4:
			regs->mfc_codedownload = val;
			break;
		case 0x8:
			regs->mfc_hostintreq = val;
			break;
		case 0xc:
			regs->mfc_bitintclear = val;
			break;
		case 0x10:
			regs->mfc_bitintsts = val;
			break;
		case 0x14:
			regs->mfc_bitcodereset = val;
			break;
		case 0x18:
			regs->mfc_bitcurpc = val;
			break;

		case 0x100:
			regs->mfc_codebufaddr = val;
			break;
		case 0x104:
			regs->mfc_workbufaddr = val;
			break;
		case 0x108:
			regs->mfc_parabufaddr = val;
			break;
		case 0x10c:
			regs->mfc_bitstreamctrl = val;
			break;
		case 0x110:
			regs->mfc_framememctrl = val;
			break;
		case 0x114:
			regs->mfc_decfunctrl = val;
			break;
		case 0x11c:
			regs->mfc_bitworkbufctrl = val;
			break;
		case 0x120:
			regs->mfc_bitstreamrdptr0 = val;
			break;
		case 0x124:
			regs->mfc_bitstreamwrptr0 = val;
			break;
		case 0x128:
			regs->mfc_bitstreamrdptr1 = val;
			break;
		case 0x12c:
			regs->mfc_bitstreamwrptr1 = val;
			break;
		case 0x130:
			regs->mfc_bitstreamrdptr2 = val;
			break;
		case 0x134:
			regs->mfc_bitstreamwrptr2 = val;
			break;
		case 0x138:
			regs->mfc_bitstreamrdptr3 = val;
			break;
		case 0x13c:
			regs->mfc_bitstreamwrptr3 = val;
			break;
		case 0x140:
			regs->mfc_bitstreamrdptr4 = val;
			break;
		case 0x144:
			regs->mfc_bitstreamwrptr4 = val;
			break;
		case 0x148:
			regs->mfc_bitstreamrdptr5 = val;
			break;
		case 0x14c:
			regs->mfc_bitstreamwrptr5 = val;
			break;
		case 0x150:
			regs->mfc_bitstreamrdptr6 = val;
			break;
		case 0x154:
			regs->mfc_bitstreamwrptr6 = val;
			break;
		case 0x158:
			regs->mfc_bitstreamrdptr7 = val;
			break;
		case 0x15c:
			regs->mfc_bitstreamwrptr7 = val;
			break;
		case 0x160:
			regs->mfc_busyflag = val;
			break;
		case 0x164:
			regs->mfc_runcommand = val;
			break;
		case 0x168:
			regs->mfc_runindex = val;
			break;
		case 0x16c:
			regs->mfc_runcodstd = val;
			break;
		case 0x170:
			regs->mfc_intenable = val;
			break;
		case 0x174:
			regs->mfc_reason = val;
			break;
		case 0x180:
			regs->mfc_command[0] = val;
			break;
		case 0x184:
			regs->mfc_command[1] = val;
			break;
		case 0x188:
			regs->mfc_command[2] = val;
			break;
		case 0x18c:
			regs->mfc_command[3] = val;
			break;
		case 0x190:
			regs->mfc_command[4] = val;
			break;
		case 0x194:
			regs->mfc_command[5] = val;
			break;
		case 0x198:
			regs->mfc_command[6] = val;
			break;
		case 0x19c:
			regs->mfc_command[7] = val;
			break;
		case 0x1a0:
			regs->mfc_command[8] = val;
			break;
		case 0x1a4:
			regs->mfc_command[9] = val;
			break;
		case 0x1a8:
			regs->mfc_command[10] = val;
			break;
		case 0x1ac:
			regs->mfc_command[11] = val;
			break;
		case 0x1b0:
			regs->mfc_command[12] = val;
			break;
		case 0x1b4:
			regs->mfc_command[13] = val;
			break;
		case 0x1b8:
			regs->mfc_command[14] = val;
			break;
		case 0x1bc:
			regs->mfc_command[15] = val;
			break;
		case 0x1c0:
			regs->mfc_command[16] = val;
			break;
		case 0x1c4:
			regs->mfc_command[17] = val;
			break;
		case 0x1c8:
			regs->mfc_command[18] = val;
			break;
		case 0x1cc:
			regs->mfc_command[19] = val;
			break;
		case 0x1d0:
			regs->mfc_command[20] = val;
			break;
		case 0x1d4:
			regs->mfc_command[21] = val;
			break;
		case 0x1d8:
			regs->mfc_command[22] = val;
			break;
		default:
			printf("Can not write the register at 0x%x in mfc\n", offset);
			return Invarg_exp;
	}
	return No_exp;
}
static conf_object_t* new_s3c6410_mfc(char* obj_name){
	s3c6410_mfc_device* dev = skyeye_mm_zero(sizeof(s3c6410_mfc_device));
	mfc_reg_t* regs =  skyeye_mm_zero(sizeof(mfc_reg_t));
	dev->obj = new_conf_object(obj_name, dev);
	/* init mfc regs */
	dev->regs = regs;

	/* Register io function to the object */
	memory_space_intf* io_memory = skyeye_mm_zero(sizeof(memory_space_intf));
	io_memory->conf_obj = dev->obj;
	io_memory->read = s3c6410_mfc_read;
	io_memory->write = s3c6410_mfc_write;
	SKY_register_interface(io_memory, obj_name, MEMORY_SPACE_INTF_NAME);	
	return dev->obj;
}
void free_s3c6410_mfc(conf_object_t* dev){
	
}

void init_s3c6410_mfc(){
	static skyeye_class_t class_data = {
		.class_name = "s3c6410_mfc",
		.class_desc = "s3c6410 mfc",
		.new_instance = new_s3c6410_mfc,
		.free_instance = free_s3c6410_mfc,
		.get_attr = NULL,
		.set_attr = NULL
	};

	SKY_register_class(class_data.class_name, &class_data);
}
