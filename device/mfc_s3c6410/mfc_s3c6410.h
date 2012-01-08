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
* @file mfc_s3c6410.h
* @brief The definition of system controller for s3c6410
* @author xq2537@gmail.com
* @version 78.77
* @date 2012-1-8
*/

#ifndef __MFC_S3C6410_H__
#define __MFC_S3C6410_H__

typedef struct mfc_reg{
	/*BIT Processor Common Register Summary*/
	uint32_t mfc_coderun;
	uint32_t mfc_codedownload;
	uint32_t mfc_hostintreq;
	uint32_t mfc_bitintclear;
	uint32_t mfc_bitintsts;
	uint32_t mfc_bitcodereset;
	uint32_t mfc_bitcurpc;

	uint32_t mfc_codebufaddr;
	uint32_t mfc_workbufaddr;
	uint32_t mfc_parabufaddr;
	uint32_t mfc_bitstreamctrl;
	uint32_t mfc_framememctrl;
	uint32_t mfc_decfunctrl;
	uint32_t mfc_bitworkbufctrl;
	uint32_t mfc_bitstreamrdptr0;
	uint32_t mfc_bitstreamwrptr0;
	uint32_t mfc_bitstreamrdptr1;
	uint32_t mfc_bitstreamwrptr1;
	uint32_t mfc_bitstreamrdptr2;
	uint32_t mfc_bitstreamwrptr2;
	uint32_t mfc_bitstreamrdptr3;
	uint32_t mfc_bitstreamwrptr3;
	uint32_t mfc_bitstreamrdptr4;
	uint32_t mfc_bitstreamwrptr4;
	uint32_t mfc_bitstreamrdptr5;
	uint32_t mfc_bitstreamwrptr5;
	uint32_t mfc_bitstreamrdptr6;
	uint32_t mfc_bitstreamwrptr6;
	uint32_t mfc_bitstreamrdptr7;
	uint32_t mfc_bitstreamwrptr7;
	uint32_t mfc_busyflag;
	uint32_t mfc_runcommand;
	uint32_t mfc_runindex;
	uint32_t mfc_runcodstd;
	uint32_t mfc_intenable;
	uint32_t mfc_reason;
	uint32_t mfc_command[22];
}mfc_reg_t; 

typedef struct s3c6410_mfc_device{
	conf_object_t* obj;
	mfc_reg_t* regs;
}s3c6410_mfc_device;

#endif
