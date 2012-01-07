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
* @file spi_s3c6410.h
* @brief The definition of system controller for s3c6410
* @author Michael.Kang blackfin.kang@gmail.com
* @version 78.77
* @date 2011-12-12
*/

#ifndef __SPI_S3C6410_H__
#define __SPI_S3C6410_H__

typedef struct spi_reg{
	uint32 ch_cfg; 
	uint32 clk_cfg;
	uint32 mode_cfg;
	uint32 cs_reg;
	uint32 spi_int_en;
	uint32 spi_status;
	uint32 spi_tx_data;
	uint32 spi_rx_data;
	uint32 packet_cnt_reg;
	uint32 swap_cfg;
	uint32 fb_clk_sel;
	uint32 pending_clr_reg;
	 
}spi_reg_t; 

typedef struct s3c6410_spi_device{
	conf_object_t* obj;
	spi_reg_t* regs;
}s3c6410_spi_device;

#endif
