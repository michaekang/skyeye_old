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
* @file rtc_s3c6410.h
* @brief The definition of system controller for s3c6410
* @author xq2537@gmail.com
* @version 78.77
* @date 2012-1-8
*/

#ifndef __RTC_S3C6410_H__
#define __RTC_S3C6410_H__

typedef struct rtc_reg{
	uint32_t rtc_intp;
	uint32_t rtc_rtccon;
	uint32_t rtc_ticcnt;
	uint32_t rtc_rtcalm;
	uint32_t rtc_almsec;
	uint32_t rtc_almmin;
	uint32_t rtc_almhour;
	uint32_t rtc_almdate;
	uint32_t rtc_almmon;
	uint32_t rtc_almyear;
	uint32_t rtc_bcdsec;
	uint32_t rtc_bcdmin;
	uint32_t rtc_bcdhour;
	uint32_t rtc_bcddate;
	uint32_t rtc_bcdday;
	uint32_t rtc_bcdmon;
	uint32_t rtc_bcdyear;
	uint32_t rtc_curticcnt;
}rtc_reg_t; 

typedef struct s3c6410_rtc_device{
	conf_object_t* obj;
	rtc_reg_t* regs;
}s3c6410_rtc_device;

#endif
