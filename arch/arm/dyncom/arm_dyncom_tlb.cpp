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
* @brief TLB implementation
* @author Michael.Kang blackfin.kang@gmail.com
* @version 7849
* @date 2012-03-26
*/

#include "armdefs.h"
#include "arm_dyncom_tlb.h"
#include <skyeye_mm.h>
#include <skyeye_log.h>

#define ASID (state->mmu.context_id & ASID_SIZE)
static uint32_t** tlb_entry_array = NULL;
static inline void invalidate_all_tlb(ARMul_State *state){
	int i = 0;
	//printf("In %s, all tlb is freed\n", __FUNCTION__);
	if(tlb_entry_array == NULL)
		return;
	while(i < ASID){
		if(tlb_entry_array[i] != NULL){
			skyeye_free(tlb_entry_array[i]);
			tlb_entry_array[i] = NULL;
		}
		i++;
	}
	//skyeye_free(tlb_entry_array);
	//tlb_entry_array = NULL;
	return;
}
void invalidate_by_mva(cpu_t* cpu, ARMword va){
	int i;
	LOG("In %s, va 0x%llx is freed, tlb_entry_array=0x%x\n", __FUNCTION__, va, tlb_entry_array);
	for(i = 0; i < ASID_SIZE; i++){
		if(tlb_entry_array[i] != NULL)
			tlb_entry_array[i][va >> 12]= 0xFFFFFFFF;
	}
	LOG("In %s, finished.\n", __FUNCTION__);
	return;
}
void invalidate_by_asid(ARMul_State *state, ARMword asid){
	//LOG("In %s, asid %d is freed\n", __FUNCTION__, asid);
	if(tlb_entry_array[asid] != NULL){
		skyeye_free(tlb_entry_array[asid]);
		tlb_entry_array[asid] = NULL;
	}
	return;
}

uint32_t get_phys_page(ARMul_State* state, ARMword va){
	uint32_t phys_page = 0xFFFFFFFF;
	if(tlb_entry_array && tlb_entry_array[ASID])
		phys_page = tlb_entry_array[ASID][va >> 12];
	//printf("In %s, for va=0x%x, page=0x%x\n", __func__, va, phys_page);
	return phys_page;
}

inline void insert_tlb(ARMul_State* state, ARMword va, ARMword pa){
	int asid = ASID;
	printf("In %s, asid=0x%x, tlb_entry_array=0x%x\n", __FUNCTION__, asid, tlb_entry_array);
	if(tlb_entry_array[asid] == NULL){
		printf("In %s, add new asid=0x%x\n", __FUNCTION__, asid);
		tlb_entry_array[asid] = (uint32_t *)skyeye_mm(TLB_SIZE * sizeof(uint32_t));
		memset(tlb_entry_array[asid], 0xFF, TLB_SIZE * sizeof(uint32_t));
	}
	tlb_entry_array[asid][va >> 12] = pa & 0xFFFFF000;
	//printf("In %s, asid=0x%x, va=0x%x, pa=0x%x\n", __FUNCTION__, asid, va, pa);
	return;
}

uint32_t** new_tlb(int line, int way){
	tlb_entry_array = (uint32_t**)skyeye_mm_zero(ASID_SIZE * sizeof(unsigned long));
	//printf("In %s, tlb allocated %d bytes, tlb_entry_array=0x%x\n", __FUNCTION__, ASID_SIZE * sizeof(unsigned long), tlb_entry_array);
	return tlb_entry_array;
}
void fini_tlb(){
	int i = 0;
	while(i++ < (TLB_SIZE - 1))
		if(tlb_entry_array[i] != NULL)
			skyeye_free(tlb_entry_array[i]);
	skyeye_free(tlb_entry_array);
	return;
}
