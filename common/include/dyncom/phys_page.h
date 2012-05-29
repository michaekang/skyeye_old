/* Copyright (C) 
* 2012 - Michael.Kang blackfin.kang@gmail.com
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
* @file page.h
* @brief The definition of physial page
* @author Michael.Kang blackfin.kang@gmail.com
* @version 7849
* @date 2012-05-23
*/
#ifndef __DYNCOM_PAGE_H__
#define __DYNCOM_PAGE_H__
#include <algorithm>
#include <vector>
#include <stdint.h>
#include "dyncom_types.h"
using namespace std;
typedef struct phys_page_desc{
	int jit_num;
	vector<int> jit_func;
	vector<uint32_t> virt_page;
} phys_page_desc_t;
void init_phys_pages();
phys_page_desc_t* get_phys_page_desc(addr_t addr);
void inc_jit_num(addr_t addr);
int get_jit_num(addr_t addr);
void add_virt_addr(addr_t pa, addr_t va);
/* FIXME, the physical address for s3c6410, should get 
these value from skyeye.conf */
#define BANK0_START 0x40000000
#define BANK0_SIZE 0x30000000
#define BANK0_END (BANK0_START + BANK0_SIZE)

#define IO_BANK(addr) ((addr < BANK0_START) || (addr >= BANK0_END))
#define PAGE_INDEX(addr) ((addr >> 12) & 0x3FFFF)
#define PAGE_NUM (PAGE_INDEX(0xFFFFFFFF) + 1)
#endif
