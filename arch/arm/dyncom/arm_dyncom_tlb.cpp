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
* @file arm_dyncom_tlb.cpp
* @brief The tlb implementation with less memory
* @author Michael.Kang blackfin.kang@gmail.com
* @version 7849
* @date 2012-03-28
*/
#include <stdint.h> 
#include <string.h>
#include <stdlib.h>
//typedef tlb_item 
#include "arm_dyncom_tlb.h"
struct tlb_item {
	uint32_t pa;
	uint32_t va;
};
tlb_item tlb_cache[256][2048];
int get_phys_page(unsigned int va, unsigned int &pa)
{
	tlb_item *tlb_entry = &tlb_cache[va & 0xff][(va >> 12) % 2048];
	if (va == tlb_entry->va) {
		pa = tlb_entry->pa;
		return 0;
	} else {
		return -1;
	}
}

void insert(unsigned int va, unsigned int pa)
{
	tlb_item* tlb_entry = &tlb_cache[va & 0xff][(va >> 12) % 2048];
	tlb_entry->va = va;
	tlb_entry->pa = pa;
}

void erase_by_mva(cpu_t* cpu, unsigned int va)
{
	tlb_cache[va & 0xff][(va >> 12) % 2048].va = 0;
}

void erase_by_asid(cpu_t* cpu, unsigned int asid)
{
	memset(&tlb_cache[asid], 0, sizeof(tlb_item) * 2048);
}

void clear()
{
	memset(tlb_cache, 0, sizeof(tlb_item) * 2048 * 256);
}

uint64_t* new_tlb(){
	return (uint64_t*)tlb_cache;
}
