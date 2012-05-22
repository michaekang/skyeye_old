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
//static tlb_item* tlb_cache = NULL;
static uint64_t tlb_cache[TLB_TOTAL][ASID_SIZE][TLB_SIZE];
//static tlb_table tlb[TLB_TOTAL];
int get_phys_page(unsigned int va, int context_id, unsigned int &pa, tlb_type_t access_type)
{
	tlb_item *tlb_entry = (tlb_item *)&tlb_cache[access_type][context_id][(va >> 12) % TLB_SIZE];
	if (va == tlb_entry->va) {
		pa = tlb_entry->pa;
		//printf("get pa 0x%x for va 0x%x in %s\n", va, pa, __FUNCTION__);
		return 0;
	} else {
		return -1;
	}
}

void insert(unsigned int va, int context_id, unsigned int pa, tlb_type_t access_type)
{
	tlb_item* tlb_entry = (tlb_item* )&tlb_cache[access_type][context_id][(va >> 12) % TLB_SIZE];
	//printf("In %s, index=0x%x, va=0x%x, pa=0x%x, tlb_entry=0x%llx, access_type=%d\n", __FUNCTION__, ((va & 0xff) * TLB_SIZE) + ((va >> 12) % TLB_SIZE), va, pa, (unsigned long)tlb_entry, access_type);
	tlb_entry->va = va;
	/* mark the io page */
	//if(pa < BANK0_START || pa >= BANK0_END)
	pa |= IO_FLAG_MASK;

	tlb_entry->pa = pa;
	if(pa & 0x3 == 0){
		printf("\n\nap = %d for va=0x%x, we exit here\n\n", pa & 0x3, va);
		sleep(2);
		exit(-1);
	}
}

void erase_by_mva(cpu_t* cpu, unsigned int va, tlb_type_t access_type)
{
	tlb_cache[access_type][va & (ASID_SIZE - 1)][(va >> 12) % TLB_SIZE] = 0;
}

void erase_by_asid(cpu_t* cpu, unsigned int asid, tlb_type_t access_type)
{
	memset(&tlb_cache[access_type][asid], 0, sizeof(tlb_item) * TLB_SIZE);
}

void erase_all(cpu_t* cpu, tlb_type_t access_type)
{
	memset(&tlb_cache[access_type], 0, sizeof(tlb_item) * TLB_SIZE * ASID_SIZE);
}

uint64_t* new_tlb(){
	int size = TLB_TOTAL * TLB_SIZE * ASID_SIZE;
	#if 0
	tlb_cache = (tlb_item*)mmap(NULL, size, PROT_WRITE, MAP_32BIT|MAP_ANONYMOUS, NULL, 0);
	if(tlb_cache == MAP_FAILED){
		skyeye_debug("mmap failed errno is %d\n", errno);
	}
	#endif
	printf("In %s, get TLB 0x%llx\n", __FUNCTION__, (unsigned long)tlb_cache);
	return (uint64_t*)tlb_cache;
}
