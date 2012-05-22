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
* @file arm_dyncom_mmu.h
* @brief The interface for mmu
* @author Michael.Kang blackfin.kang@gmail.com
* @version 7849
* @date 2012-04-01
*/

#ifndef ARM_DYNCOM_MMU_H
#define ARM_DYNCOM_MMU_H

#include <skyeye_dyncom.h>
#include <skyeye_types.h>
#include <skyeye_obj.h>
#include <skyeye.h>
#include <bank_defs.h>
#include <skyeye_pref.h>

#include "arm_regformat.h"
#include "armdefs.h"
#include "memory.h"
#include "armmmu.h"


#include "dyncom/frontend.h"
#include "dyncom/tlb.h"

#define CP15(idx)	(idx - CP15_BASE)
#define CP15REG(idx)	(core->CP15[CP15(idx)])
#define MMU_ENABLED	core->CP15[CP15(CP15_CONTROL)]

#define MMU_DEBUG	0
void remove_tlb_by_asid(uint32_t asid, tlb_type_t type);
void remove_tlb(tlb_type_t type);
void remove_tlb_by_mva(uint32_t mva, tlb_type_t type);

int fill_tlb(arm_core_t* core);

extern arch_mem_ops_t arm_dyncom_mem_ops;

#define USER_MODE(core) ((core->Mode == USER32MODE) || (core->Mode == USER26MODE) || (core->Mode == SYSTEM32MODE))
#define is_kernel_code(pc) (pc > 0xc0000000)
/* FIXME, the physical address for s3c6410, should get 
these value from skyeye.conf */
#define BANK0_START 0x40000000
#define BANK0_SIZE 0x30000000
#define BANK0_END (BANK0_START + BANK0_SIZE)

#define CHECK_IN_WRITE 1
#endif
