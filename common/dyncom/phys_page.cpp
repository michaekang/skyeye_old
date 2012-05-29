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
* @file phys_page.cpp
* @brief Descripe the attribute of all the physical page
* @author Michael.Kang blackfin.kang@gmail.com
* @version 7849
* @date 2012-05-23
*/
#include <skyeye_log.h>
#include <skyeye_mm.h>
#include <dyncom/phys_page.h>
#include <dyncom_types.h>
#include <stdio.h>
static phys_page_desc_t phys_pages[PAGE_NUM];
void init_phys_pages(){
	#if 0
	phys_pages = (phys_page_desc_t*)skyeye_mm_zero(PAGE_NUM * sizeof(phys_page_desc_t));
	if(phys_pages != NULL)
		printf("initial %d physical pages, sizeof()\n", PAGE_NUM);
	#endif
	return;
}

phys_page_desc_t* get_phys_page_desc(addr_t addr){
	return &phys_pages[PAGE_INDEX(addr)];
}

void add_virt_addr(addr_t pa, addr_t va){
	vector<uint32_t> virt_pages = phys_pages[PAGE_INDEX(pa)].virt_page;
	const vector<uint32_t>::iterator it = find(virt_pages.begin(),
                                                   virt_pages.end(),
                                                   va);
	if(it == virt_pages.end())
		virt_pages.push_back(va);
	else
		; /* Do nothing, va exists already */
	return;
}
void inc_jit_num(addr_t addr){
	phys_pages[PAGE_INDEX(addr)].jit_num++;
	return;
}
int get_jit_num(addr_t addr){
	//DBG("addr=0x%x, index=0x%x in %s\n", addr, PAGE_INDEX(addr), __FUNCTION__);
	return phys_pages[PAGE_INDEX(addr)].jit_num;
	//return 0;
}
