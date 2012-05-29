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
* @file arm_dyncom_tlb.h
* @brief The interface of tlb
* @author Michael.Kang blackfin.kang@gmail.com
* @version 7849
* @date 2012-03-28
*/

#ifndef __ARM_DYNCOM_TLB_H__
#define __ARM_DYNCOM_TLB_H__
#include <skyeye_dyncom.h>
#define TLB_SIZE 4096
#define ASID_SIZE 256
typedef enum _tlb_type {
	DATA_USER_READ = 0,
	DATA_USER_WRITE,
	DATA_KERNEL_READ,
	DATA_KERNEL_WRITE,
	IO_TLB,
	MIXED_TLB,
	INSN_USER,
	INSN_KERNEL,
	TLB_TOTAL,
	INSN_TLB,
	DATA_TLB,

} tlb_type_t;

struct tlb_item {
	uint32_t pa;
	uint32_t va;
};

#define TLB_ENTRY_SIZE sizeof(struct tlb_item)
int get_phys_page(unsigned int va, int context_id, unsigned int &pa, tlb_type_t access_type);
void insert(unsigned int va, int context_id, unsigned int pa, tlb_type_t access_type);
uint64_t* new_tlb();
//inline void insert_tlb(ARMul_State* state, ARMword va, ARMword pa);
//void invalidate_by_asid(ARMul_State *state, ARMword asid);
//void invalidate_by_mva(cpu_t* cpu, ARMword va);
void erase_by_asid(cpu_t* cpu, unsigned int asid, tlb_type_t access_type);
void erase_by_mva(cpu_t* cpu, unsigned int va, tlb_type_t access_type);
void erase_all(cpu_t* cpu, tlb_type_t access_type);

uint64_t get_tlb(tlb_type_t access_type);
#define GET_AP(phys_page) (phys_page & 0x3)

#define IO_FLAG_MASK 0x4
#define GET_IO_FLAG(p) ((p >> 2) & 0x1)
#define INVAILAD_ITEM 0x0
#endif
