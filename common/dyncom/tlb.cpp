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
//#define DEBUG
#include "skyeye_log.h"

//typedef tlb_item 
#include "dyncom/phys_page.h"
#include "dyncom/tlb.h"
//static tlb_item* tlb_cache = NULL;
static uint64_t tlb_cache[TLB_TOTAL][ASID_SIZE][TLB_SIZE];
static int max_context_id = 0;
//static tlb_table tlb[TLB_TOTAL];
int get_phys_page(unsigned int va, int context_id, unsigned int &pa, tlb_type_t access_type)
{
	//DBG("type=%d in %s\n", access_type, __FUNCTION__);	
	tlb_item *tlb_entry = (tlb_item *)&tlb_cache[access_type][context_id][(va >> 12) % TLB_SIZE];
	if (va == tlb_entry->va) {
		pa = tlb_entry->pa;
		//DBG("get pa 0x%x for va 0x%x in %s\n", va, pa, __FUNCTION__);
		return 0;
	} else {
		return -1;
	}
}

void insert(unsigned int va, int context_id, unsigned int pa, tlb_type_t access_type)
{
	tlb_item* tlb_entry = NULL;
	DBG("In %s, index=0x%x, va=0x%x, pa=0x%x, tlb_entry=0x%llx, access_type=%d\n", __FUNCTION__, ((va & 0xff) * TLB_SIZE) + ((va >> 12) % TLB_SIZE), va, pa, (unsigned long)tlb_entry, access_type);
	/* mark the io page */
	assert(access_type < TLB_TOTAL && access_type >= 0);
	if(context_id > max_context_id)
		max_context_id = context_id;
	#if 1
	if((access_type == DATA_USER_WRITE) || (access_type == DATA_KERNEL_WRITE)){
		/* set to MIXED type for the page also contain some translated instructions */
		if(get_jit_num(pa) != 0)
			access_type = MIXED_TLB;
	}
	if(access_type == INSN_USER || access_type == INSN_KERNEL){
		/* also need to check if the corresponding page exist at data tlb */
		tlb_item* kernel_item = (tlb_item* )&tlb_cache[DATA_KERNEL_WRITE][context_id][(va >> 12) % TLB_SIZE];
		tlb_item* user_item = (tlb_item* )&tlb_cache[DATA_USER_WRITE][context_id][(va >> 12) % TLB_SIZE];
		if(kernel_item->va == va && kernel_item->pa == pa){
			kernel_item->va = kernel_item->pa = INVAILAD_ITEM;
			tlb_entry = (tlb_item* )&tlb_cache[MIXED_TLB][context_id][(va >> 12) % TLB_SIZE];
			tlb_entry->pa = pa;
			tlb_entry->va = va;
			//access_type = MIXED_TLB;
		}
		if(user_item->va == va && user_item->pa == pa){
			user_item->va = user_item->pa = INVAILAD_ITEM;
			tlb_entry = (tlb_item* )&tlb_cache[MIXED_TLB][context_id][(va >> 12) % TLB_SIZE];
			tlb_entry->pa = pa;
			tlb_entry->va = va;
			//access_type = MIXED_TLB;
		}
	}
	#endif
	if(IO_BANK(pa)){
		assert(access_type != MIXED_TLB);
		access_type = IO_TLB;
	}
	tlb_entry = (tlb_item* )&tlb_cache[access_type][context_id][(va >> 12) % TLB_SIZE];
	tlb_entry->pa = pa;
	tlb_entry->va = va;
	//add_virt_addr(pa, va);
	if(pa & 0x3 == 0){
		printf("\n\nap = %d for va=0x%x, we exit here\n\n", pa & 0x3, va);
		sleep(2);
		exit(-1);
	}
}

void erase_by_mva(cpu_t* cpu, unsigned int va, tlb_type_t access_type)
{
	if(access_type == DATA_TLB){
		if((va & (ASID_SIZE - 1)) == 0){
			int i = 0;
			for(; i <= max_context_id; i++){
				tlb_cache[DATA_USER_READ][i][(va >> 12) % TLB_SIZE] = 0;
				tlb_cache[DATA_KERNEL_READ][i][(va >> 12) % TLB_SIZE] = 0;
				tlb_cache[DATA_USER_WRITE][i][(va >> 12) % TLB_SIZE] = 0;
				tlb_cache[DATA_KERNEL_WRITE][i][(va >> 12) % TLB_SIZE] = 0;
				tlb_cache[IO_TLB][i][(va >> 12) % TLB_SIZE] = 0;
				tlb_cache[MIXED_TLB][i][(va >> 12) % TLB_SIZE] = 0;
			}
		}
		else{
			tlb_cache[DATA_USER_READ][va & (ASID_SIZE - 1)][(va >> 12) % TLB_SIZE] = 0;
			tlb_cache[DATA_KERNEL_READ][va & (ASID_SIZE - 1)][(va >> 12) % TLB_SIZE] = 0;
			tlb_cache[DATA_USER_WRITE][va & (ASID_SIZE - 1)][(va >> 12) % TLB_SIZE] = 0;
			tlb_cache[DATA_KERNEL_WRITE][va & (ASID_SIZE - 1)][(va >> 12) % TLB_SIZE] = 0;
			tlb_cache[IO_TLB][va & (ASID_SIZE - 1)][(va >> 12) % TLB_SIZE] = 0;
			tlb_cache[MIXED_TLB][va & (ASID_SIZE - 1)][(va >> 12) % TLB_SIZE] = 0;
		}
	}
	else if(access_type == INSN_TLB){
		if((va & (ASID_SIZE - 1)) == 0){
			int i = 0;
			for(; i <= max_context_id; i++){
				tlb_cache[INSN_USER][i][(va >> 12) % TLB_SIZE] = 0;
				tlb_cache[INSN_KERNEL][i][(va >> 12) % TLB_SIZE] = 0;
			}
		}
		else{
			tlb_cache[INSN_USER][va & (ASID_SIZE - 1)][(va >> 12) % TLB_SIZE] = 0;
			tlb_cache[INSN_KERNEL][va & (ASID_SIZE - 1)][(va >> 12) % TLB_SIZE] = 0;
		}
	}else{
		skyeye_error("Wrong tlb type %d\n", access_type);
	}
}

void erase_by_asid(cpu_t* cpu, unsigned int asid, tlb_type_t access_type)
{
	if(access_type == DATA_TLB){
		memset(&tlb_cache[DATA_USER_READ][asid], 0, sizeof(tlb_item) * TLB_SIZE);
		memset(&tlb_cache[DATA_USER_WRITE][asid], 0, sizeof(tlb_item) * TLB_SIZE);
		memset(&tlb_cache[DATA_KERNEL_READ][asid], 0, sizeof(tlb_item) * TLB_SIZE);
		memset(&tlb_cache[DATA_KERNEL_WRITE][asid], 0, sizeof(tlb_item) * TLB_SIZE);
		memset(&tlb_cache[IO_TLB][asid], 0, sizeof(tlb_item) * TLB_SIZE);
		memset(&tlb_cache[MIXED_TLB][asid], 0, sizeof(tlb_item) * TLB_SIZE);
	}else if(access_type == INSN_TLB){
		memset(&tlb_cache[INSN_USER][asid], 0, sizeof(tlb_item) * TLB_SIZE);
		memset(&tlb_cache[INSN_KERNEL][asid], 0, sizeof(tlb_item) * TLB_SIZE);
	}
	else{
		skyeye_error("Wrong tlb type %d\n", access_type);
	}
}

void erase_all(cpu_t* cpu, tlb_type_t access_type)
{
	if(access_type == DATA_TLB){
		memset(&tlb_cache[DATA_USER_READ], 0, sizeof(tlb_item) * TLB_SIZE * ASID_SIZE);
		memset(&tlb_cache[DATA_USER_WRITE], 0, sizeof(tlb_item) * TLB_SIZE * ASID_SIZE);
		memset(&tlb_cache[DATA_KERNEL_READ], 0, sizeof(tlb_item) * TLB_SIZE * ASID_SIZE);
		memset(&tlb_cache[DATA_KERNEL_WRITE], 0, sizeof(tlb_item) * TLB_SIZE * ASID_SIZE);
		memset(&tlb_cache[IO_TLB], 0, sizeof(tlb_item) * TLB_SIZE * ASID_SIZE);
		memset(&tlb_cache[MIXED_TLB], 0, sizeof(tlb_item) * TLB_SIZE * ASID_SIZE);
	}else if(access_type == INSN_TLB){
		memset(&tlb_cache[INSN_USER], 0, sizeof(tlb_item) * TLB_SIZE * ASID_SIZE);
		memset(&tlb_cache[INSN_KERNEL], 0, sizeof(tlb_item) * TLB_SIZE * ASID_SIZE);
	}else{
		skyeye_error("Wrong tlb type %d\n", access_type);
	}
}

uint64_t get_tlb(tlb_type_t access_type){
	return (unsigned long)&tlb_cache[access_type];
}

uint64_t* new_tlb(){
	int size = TLB_TOTAL * TLB_SIZE * ASID_SIZE;
	#if 0
	tlb_cache = (tlb_item*)mmap(NULL, size, PROT_WRITE, MAP_32BIT|MAP_ANONYMOUS, NULL, 0);
	if(tlb_cache == MAP_FAILED){
		skyeye_debug("mmap failed errno is %d\n", errno);
	}
	#endif
	DBG("In %s, get TLB 0x%llx\n", __FUNCTION__, (unsigned long)tlb_cache);
	return (uint64_t*)tlb_cache;
}
