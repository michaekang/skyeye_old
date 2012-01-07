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
* @file ac97_s3c6410.h
* @brief The definition of USBHOST for s3c6410
* @author Michael.Kang blackfin.kang@gmail.com
* @version 78.77
* @date 2011-12-12
*/

#ifndef __USBHOST_S3C6410_H__
#define __USBHOST_S3C6410_H__

typedef struct ac97_reg{
	uint32 ac_glbctrl;
	uint32 ac_glbstat;
	uint32 ac_codec_cmd;
	uint32 ac_codec_stat;
	uint32 ac_pcmaddr;
	uint32 ac_micaddr;
	uint32 ac_pcmdata;
	uint32 ac_micdata;
}ac97_reg_t; 

typedef struct s3c6410_ac97_device{
	conf_object_t* obj;
	ac97_reg_t* regs;
}s3c6410_ac97_device;

#endif
