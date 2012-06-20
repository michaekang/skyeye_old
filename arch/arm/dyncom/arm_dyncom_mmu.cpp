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
* @file arm_dyncom_mmu.cpp
* @brief The memory stuff for arm dyncom
* @author Michael.Kang blackfin.kang@gmail.com
* @version 7849
* @date 2012-03-08
*/
//#include "lru_tlb.h"
#include "arm_dyncom_mmu.h"
#include "dyncom/tlb.h"
#include "arm_dyncom_thumb.h"
#include "arm_dyncom_translate.h"
#include "arm_dyncom_interpreter.h"
#include "skyeye_dyncom.h"
#include <skyeye_log.h>
#include "skyeye_obj.h"
#include "arm_dyncom_run.h"
#include "arm_dyncom_dec.h"
#include "dyncom/tag.h"
#include "dyncom/defines.h"
#include "skyeye_ram.h"
/* shenoubang add win32 2102-6-12 */
#ifndef __WIN32__
#include <execinfo.h>
#endif

#if FAST_MEMORY
static inline void mem_read_raw(void *mem_ptr, uint32_t offset, uint32_t &value, int size)
{
	switch(size) {
	case 8:
		value = *((uint8_t *)mem_ptr + offset);
		break;
	case 16:
		//offset &= 0xFFFFFFFE;
		value = *(uint16_t *)((uint8_t *)mem_ptr + offset);
		break;
	case 32:
		//offset &= 0xFFFFFFFC;
		value = *(uint32_t *)((uint8_t *)mem_ptr + offset);
		break;
	}
}

static inline void mem_write_raw(void *mem_ptr, uint32_t offset, uint32_t value, int size)
{
	switch(size) {
	case 8:
		*((uint8_t *)mem_ptr + offset) = value & 0xff;
		break;
	case 16:
		//offset &= 0xFFFFFFFE;
		*(uint16_t *)((uint8_t *)mem_ptr + offset) = value & 0xffff;
		break;
	case 32:
		//offset &= 0xFFFFFFFC;
		*(uint32_t *)((uint8_t *)mem_ptr + offset) = value;
		break;
	}
}

static inline int mem_read_directly(cpu_t* cpu, uint32_t phys_addr, uint32_t &value, int size)
{
	int ret = -1;
	uint32_t offset;
	if (phys_addr >= BANK0_START && phys_addr < BANK0_END) {
		//offset = phys_addr - BANK0_START;
		//mem_read_raw(cpu->dyncom_engine->RAM, offset, value, size);
		mem_read_raw(0, phys_addr, value, size);
		ret = 0;
	}
	return ret;
}

static inline int mem_write_directly(cpu_t* cpu, uint32_t phys_addr, uint32_t value, int size)
{
	int ret = -1;
	uint32_t offset;
	if (phys_addr >= BANK0_START && phys_addr < BANK0_END) {
		//offset = phys_addr - BANK0_START;
		mem_write_raw(0, phys_addr, value, size);
		ret = 0;
	}
	return ret;
}
#endif

static bool_t is_inside_page(cpu_t *cpu, addr_t a)
{
//	return ((a & 0xfffff000) == cpu->current_page_phys) ? True : False;
	return True;
}
static bool_t is_page_start(cpu_t* cpu, addr_t a)
{
	return ((a & 0x00000fff) == 0x0) ? True : False;
}

static bool_t is_page_end(cpu_t* cpu, addr_t a)
{
	uint32 page_end_addr = 0x1000 - cpu->f.get_instr_length(cpu);
	return ((a & 0x00000fff) == page_end_addr) ? True : False;
}

int
dyncom_check_perms (arm_core_t *core, int ap, int read)
{
	int s, r, user;

	s = CP15REG(CP15_CONTROL) & CONTROL_SYSTEM;
	r = CP15REG(CP15_CONTROL) & CONTROL_ROM;
	/* chy 2006-02-15 , should consider system mode, don't conside 26bit mode */
	user = (core->Mode == USER32MODE) || (core->Mode == USER26MODE) || (core->Mode == SYSTEM32MODE);
//	printf("mode is %x\n", core->Mode);
//	printf("ap is %x, user is %x, s is %x, read is %x\n", ap, user, s, read);
	switch (ap) {
	case 0:
		return read && ((s && !user) || r);
	case 1:
		return !user;
	case 2:
		return read || !user;
	case 3:
		return 1;
	}
	return 0;
}

fault_t
dyncom_mmu_translate (arm_core_t *core, ARMword virt_addr, ARMword *phys_addr, int *ap, int *sop)
{
	{
		/* walk the translation tables */
		ARMword l1addr, l1desc;
		if (CP15REG(CP15_TRANSLATION_BASE_CONTROL) && virt_addr << CP15REG(CP15_TRANSLATION_BASE_CONTROL) >> (32 - CP15REG(CP15_TRANSLATION_BASE_CONTROL) - 1)) {
			l1addr = CP15REG(CP15_TRANSLATION_BASE_TABLE_1);
			l1addr = (((l1addr >> 14) << 14) | (virt_addr >> 18)) & ~3;
		} else {
			l1addr = CP15REG(CP15_TRANSLATION_BASE_TABLE_0);
			l1addr = (((l1addr >> (14 - CP15REG(CP15_TRANSLATION_BASE_CONTROL))) << (14 - CP15REG(CP15_TRANSLATION_BASE_CONTROL))) | (virt_addr << CP15REG(CP15_TRANSLATION_BASE_CONTROL)) >> (18 + CP15REG(CP15_TRANSLATION_BASE_CONTROL))) & ~3;
		}

		/* l1desc = mem_read_word (state, l1addr); */
		#if FAST_MEMORY
		mem_read_raw(0, l1addr, l1desc, 32);
		#else
		bus_read(32, l1addr, &l1desc);
		#endif
		switch (l1desc & 3) {
		case 0:
		case 3:
			/*
			 * according to Figure 3-9 Sequence for checking faults in arm manual,
			 * section translation fault should be returned here.
			 */
			{
				#if 0
				printf("virt_addr is %x\n", virt_addr);
				printf("l1addr is %x l1desc is %x\n", l1addr, l1desc);
				printf("mmu_control is %x\n", CP15REG(CP15_TRANSLATION_BASE_CONTROL));
				printf("mmu_table_0 is %x\n", CP15REG(CP15_TRANSLATION_BASE_TABLE_0));
				printf("mmu_table_1 is %x\n", CP15REG(CP15_TRANSLATION_BASE_TABLE_1));
				bus_read(32, 0x50007004, &l1desc);
				printf("l1desc is %x\n", l1desc);
				#endif
				return SECTION_TRANSLATION_FAULT;
			}
		case 1:
			/* coarse page table */
			{
				ARMword l2addr, l2desc;


				l2addr = l1desc & 0xFFFFFC00;
				l2addr = (l2addr |
					  ((virt_addr & 0x000FF000) >> 10)) &
					~3;
				#if FAST_MEMORY
				mem_read_raw(0, l2addr, l2desc, 32);
				#else
				bus_read(32, l2addr, &l2desc);
				#endif
				/* chy 2003-09-02 for xscale */
				*ap = (l2desc >> 4) & 0x3;
				*sop = 1;	/* page */
				switch (l2desc & 3) {
				case 0:
					#if 0
					printf("virt_addr is %x\n", virt_addr);
					printf("l1addr is %x l1desc is %x\n", l1addr, l1desc);
					printf("mmu_control is %x\n", CP15REG(CP15_TRANSLATION_BASE_CONTROL));
					printf("mmu_table_0 is %x\n", CP15REG(CP15_TRANSLATION_BASE_TABLE_0));
					printf("mmu_table_1 is %x\n", CP15REG(CP15_TRANSLATION_BASE_TABLE_1));
					printf("l2addr is %x l2desc is %x\n", l2addr, l2desc);
					#endif
					return PAGE_TRANSLATION_FAULT;
					break;
				case 1:
					*phys_addr = (l2desc & 0xFFFF0000) | (virt_addr & 0x0000FFFF);
					break;
				case 2:
				case 3:
					*phys_addr = (l2desc & 0xFFFFF000) | (virt_addr & 0x00000FFF);
					break;

				}
			}
			break;
		case 2:
			/* section */

			*ap = (l1desc >> 10) & 3;
			*sop = 0; 	/* section */
			if (l1desc & 0x30000)
				*phys_addr = (l1desc & 0xFF000000) | (virt_addr & 0x00FFFFFF);
			else
				*phys_addr = (l1desc & 0xFFF00000) | (virt_addr & 0x000FFFFF);
			break;
		}
	}
	return NO_FAULT;
}
fault_t check_address_validity(arm_core_t *core, addr_t virt_addr, addr_t *phys_addr, uint32_t rw, tlb_type_t access_type = DATA_TLB)
{
	fault_t fault = NO_FAULT;
	int ap, sop;
	uint32_t p;

	int user_mode = USER_MODE(core);
	if(access_type == DATA_TLB){
                if(user_mode == 0){
                        if(rw == 1)
                                access_type = DATA_KERNEL_READ;
                        else
                                access_type = DATA_KERNEL_WRITE;
                }else{
                        if(rw == 1)
                                access_type = DATA_USER_READ;
                        else
                                access_type = DATA_USER_WRITE;
                }
	
		if (!get_phys_page((virt_addr & 0xfffff000) , (CP15REG(CP15_CONTEXT_ID) & 0xff), p, access_type)) {
			*phys_addr = (p & 0xfffff000) | (virt_addr & 0xfff);
			return fault;
		}
		else if(!get_phys_page((virt_addr & 0xfffff000) , (CP15REG(CP15_CONTEXT_ID) & 0xff), p, IO_TLB)){
        	        if (dyncom_check_perms(core, GET_AP(p), rw)) {
                	        *phys_addr = (p & 0xfffff000) | (virt_addr & 0xfff);
                        	return fault;
                	}
        	}
		/* also need to check instruction tlb for some insn and data mixed page */
		else if(!get_phys_page((virt_addr & 0xfffff000), (CP15REG(CP15_CONTEXT_ID) & 0xff), p, MIXED_TLB)){
                	if (dyncom_check_perms(core, GET_AP(p), rw)){
				*phys_addr = (p & 0xfffff000) | (virt_addr & 0xfff);
				return fault;
			}
		}
        }
        else if(access_type == INSN_TLB){
                if(user_mode == 1)
                        access_type = INSN_USER;
                else
                        access_type = INSN_KERNEL;

		if (!get_phys_page((virt_addr & 0xfffff000) , (CP15REG(CP15_CONTEXT_ID) & 0xff), p, access_type)) {
			*phys_addr = (p & 0xfffff000) | (virt_addr & 0xfff);
			return fault;
		}
	}
	else{
		skyeye_error("Wrong tlb type\n");
	}

	if ((CP15REG(CP15_CONTROL) & 1) == 0) {
		/* MMU or MPU disabled. */
		*phys_addr = virt_addr;
		insert((virt_addr & 0xfffff000), (CP15REG(CP15_CONTEXT_ID) & 0xff), ((*phys_addr) & 0xfffff000) | (0x3), access_type);
		return NO_FAULT;
	} else {
	
		fault = dyncom_mmu_translate(core, virt_addr, phys_addr, &ap, &sop);
		if (fault) {
		#if MMU_DEBUG
			printf("fault:%d\n", fault);
			printf("virt_addr:0x%08x\n", virt_addr);
			printf("icounter:%lld\n", core->icounter);
		#endif
			return fault;
		}
		/* no tlb, only check permission */
		if (!dyncom_check_perms(core, ap, rw)) {
			if (sop == 0) {
				return SECTION_PERMISSION_FAULT;
			} else {
				return SUBPAGE_PERMISSION_FAULT;
			}
		}
		//printf("In %s, get phys_addr=0x%x\n", __FUNCTION__, *phys_addr);
		insert((virt_addr & 0xfffff000), (CP15REG(CP15_CONTEXT_ID) & 0xff), ((*phys_addr) & 0xfffff000) | (ap), access_type);
		//insert_tlb(core, (virt_addr & 0xfffff000) | (CP15REG(CP15_CONTEXT_ID) & 0xff), ((*phys_addr) & 0xfffff000) | ap);
	}
	return fault;
}

fault_t interpreter_fetch(cpu_t *cpu, addr_t virt_addr, uint32_t &value, uint32_t size)
{
	addr_t phys_addr = 0;
	fault_t fault = NO_FAULT;
#if 1
	//arm_core_t* core = (arm_core_t*)get_cast_conf_obj(cpu->cpu_data, "arm_core_t");
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);

	fault = check_address_validity(core, virt_addr, &phys_addr, 1, INSN_TLB);

	if (fault) {
		printf("fetch inst exception.\n");
		printf("virt_addr:0x%08x\n", virt_addr);
//		exit(-1);
		return fault;
	}
#endif
#if FAST_MEMORY
	if(mem_read_directly(cpu, phys_addr & 0xFFFFFFFC, value, 32) == 0)
		return fault;
#endif
	if (size == 8)
		bus_read(8, phys_addr | (virt_addr & 3), &value);
	else if (size == 16)
		bus_read(16, phys_addr | (virt_addr & 3), &value);
	else
		bus_read(32, phys_addr, &value);
	return fault;
}

fault_t interpreter_read_memory(cpu_t *cpu, addr_t virt_addr, addr_t phys_addr, uint32_t &value, uint32_t size)
{
	fault_t fault = NO_FAULT;

#if FAST_MEMORY
	phys_addr = phys_addr | (virt_addr & 3);
	if(mem_read_directly(cpu, phys_addr, value, size) == 0){
		return fault;
	}
#endif
	if (size == 8)
		bus_read(8, phys_addr | (virt_addr & 3), &value);
	else if (size == 16)
		bus_read(16, phys_addr | (virt_addr & 3), &value);
	else
	{
		bus_read(32, phys_addr, &value);
		/* Unaligned read word */
		if ((virt_addr & 3) && (size == 32)) {
			virt_addr = (virt_addr & 3) << 3;       /* Get the word address.  */
			value =  ((value >> virt_addr) | (value << (32 - virt_addr)));  /* rot right */
		}
	}

	return fault;
}

fault_t interpreter_write_memory(cpu_t *cpu, addr_t virt_addr, addr_t phys_addr, uint32_t value, uint32_t size)
{
	fault_t fault = NO_FAULT;
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
#if DIFF_WRITE
	if(core->icounter > core->debug_icounter){
		/* out of the array */
		if(core->CurrWrite >= 17 ){
			printf("In %s, Wrong write array, %d@0x%x",  __FUNCTION__, core->CurrWrite, core->Reg[15]);
			exit(-1);
		}
		core->WriteAddr[core->CurrWrite] = phys_addr | (virt_addr & 3);
		core->WriteData[core->CurrWrite] = value;
		core->WritePc[core->CurrWrite] = core->Reg[15];
		core->CurrWrite++;
		//printf("In %s, pc=0x%x, addr=0x%x, data=0x%x\n", __FUNCTION__, core->Reg[15],  phys_addr | (virt_addr & 3), value);
		#if 0
		/* found the instruction of write action */
		if(value == 0xbee0f770 && core->Reg[15] == 0x400bb584){
			printf("############In %s,  va=0x%x, phys_addr=0x%x, value=0x%x\n", __FUNCTION__, virt_addr, phys_addr, value);
			int j, nptrs;
			#define SIZE 100
	       	   	void *buffer[100];
	        	   char **strings;

        		   nptrs = backtrace(buffer, SIZE);
		           printf("backtrace() returned %d addresses\n", nptrs);

           /* The call backtrace_symbols_fd(buffer, nptrs, STDOUT_FILENO)
              would produce similar output to the following: */

	           strings = backtrace_symbols(buffer, nptrs);
        	   if (strings == NULL) {
	               perror("backtrace_symbols");
        	       exit(EXIT_FAILURE);
	           }

        	   for (j = 0; j < nptrs; j++)
	               printf("%s\n", strings[j]);

        	   free(strings);

		}
	#endif
	}
#endif

#if CHECK_IN_WRITE
	if(is_fast_interp_code(cpu, phys_addr) || is_translated_code(cpu, phys_addr)){
		//printf("In %s, selfmodified code is 0x%x, pc=0x%x\n", __FUNCTION__, phys_addr, core->Reg[15]);
		//if(is_fast_interp_code(cpu, phys_addr)){
		//printf("In %s, fast interp page 0x%x is written, at 0x%x\n", __FUNCTION__, phys_addr & 0xFFFFF000, core->Reg[15]);
		flush_bb(phys_addr);
		//}
		//printf("In %s, end fast interp page 0x%x is written\n", __FUNCTION__, phys_addr & 0xFFFFF000);

		//addr_t addr = find_bb_start(cpu, phys_addr);
		//if(is_translated_code(cpu, phys_addr)){
		//printf("clear code cache 0x%x in %s\n", phys_addr, __FUNCTION__);
		pthread_rwlock_wrlock(&(cpu->dyncom_engine->rwlock));
#if L3_HASHMAP
                clear_cache_item(cpu->dyncom_engine->fmap, phys_addr);
#else
               	fprintf(stderr, "Warnning: not clear the cache");
#endif
		if(pthread_rwlock_unlock(&(cpu->dyncom_engine->rwlock))){
			fprintf(stderr, "unlock error\n");
		}
		//}
                clear_tag_page(cpu, phys_addr);
	}
#endif
#if FAST_MEMORY
	phys_addr = phys_addr | (virt_addr & 3);
	if(mem_write_directly(cpu, phys_addr, value, size) == 0){
		return fault;
	}
#endif
	if (size == 8)
		bus_write(8, phys_addr | (virt_addr & 3), value);
	else if (size == 16)
		bus_write(16, phys_addr | (virt_addr & 3), value);
	else
		bus_write(32, phys_addr, value);

	return fault;
	#if 0
	if (is_translated_code(cpu, phys_addr)) {
		//clear native code when code section was written.
		addr_t addr = find_bb_start(cpu, phys_addr);
	}
	#endif
}

static uint32_t arch_arm_read_memory(cpu_t *cpu, addr_t virt_addr, uint32_t size, int need_exclusive)
{
	uint32_t value;
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	addr_t phys_addr = 0;
	fault_t fault = NO_FAULT;
	
	fault = check_address_validity(core, virt_addr, &phys_addr, 1, DATA_TLB);
	if(fault != NO_FAULT){
		core->CP15[CP15_TLB_FAULT_STATUS - CP15_BASE] = 1;
		//core->CP15[CP15_TLB_FAULT_ADDR - CP15_BASE] = virt_addr;
		core->abortSig = true;
		core->Aborted = ARMul_DataAbortV;
		core->AbortAddr = virt_addr;
		core->CP15[CP15(CP15_FAULT_STATUS)] = fault & 0xff;
		core->CP15[CP15(CP15_FAULT_ADDRESS)] = virt_addr;

		return 0;
	}
	if(need_exclusive){
		core->exclusive_tag = phys_addr;
		core->exclusive_state = 1;
	}
	//phys_addr = virt_addr;
	#if MMU_DEBUG
	printf("bus read at %x, pc=0x%x\n", phys_addr, core->Reg[15]);
	#endif

#if FAST_MEMORY
        phys_addr = phys_addr | (virt_addr & 3);
        if(mem_read_directly(cpu, phys_addr, value, size) == 0){
		goto skip_read;
        }
#endif
	if (size == 8) {
		bus_read(8, phys_addr | (virt_addr & 3), &value);
	} else if (size == 16) {
		bus_read(16, phys_addr | (virt_addr & 3), &value);
	} else {
		bus_read(32, phys_addr, &value);
		/* Unaligned read word */
		if ((virt_addr & 3) && (size == 32)) {
			virt_addr = (virt_addr & 3) << 3;       /* Get the word address.  */
			value =  ((value >> virt_addr) | (value << (32 - virt_addr)));  /* rot right */
		}

	}
skip_read:
	return value;
}
#define LOG_IN_CLR	skyeye_printf_in_color
static void arch_arm_write_memory(cpu_t *cpu, addr_t virt_addr, uint32_t value, uint32_t size, int need_exclusive)
{
	addr_t phys_addr = 0;
	fault_t fault = NO_FAULT;
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	fault = check_address_validity(core, virt_addr, &phys_addr, 0, DATA_TLB);
	if(fault != NO_FAULT){
		core->CP15[CP15_TLB_FAULT_STATUS - CP15_BASE] = 1;
		//core->CP15[CP15_TLB_FAULT_ADDR - CP15_BASE] = virt_addr;
		core->abortSig = true;
		core->Aborted = ARMul_DataAbortV;
		core->AbortAddr = virt_addr;
		core->CP15[CP15(CP15_FAULT_STATUS)] = fault & 0xff;
		core->CP15[CP15(CP15_FAULT_ADDRESS)] = virt_addr;

		return;
	}
	if(need_exclusive){
		if(core->exclusive_state && (core->exclusive_tag == phys_addr)){
			core->exclusive_tag = 0xFFFFFFFF;
			core->exclusive_state = 0;
			core->exclusive_result = 0;
		}
		else{
			core->exclusive_result = 1;
			return;
		}
	}
	//phys_addr = virt_addr;
	#if MMU_DEBUG
	printf("bus write at %x pc=0x%x\n", phys_addr, core->Reg[15]);
	#endif
#if 0
	if (phys_addr == 0x50c27fe4 && cpu->icounter > 248306791) {
		LOG_IN_CLR(RED, "pc is %x bus write at %x value is %x size is %d\n", ((arm_core_t *)cpu->cpu_data)->Reg[15], phys_addr, value, size);
		exit(-1);
	}
#endif
#if DIFF_WRITE
	if(core->icounter > core->debug_icounter){
		/* out of the array */
		if(core->CurrWrite >= 17 ){
			printf("In %s, Wrong write array, %d@0x%x",  __FUNCTION__, core->CurrWrite, core->Reg[15]);
			exit(-1);
		}
		core->WriteAddr[core->CurrWrite] = phys_addr | (virt_addr & 3);
		core->WriteData[core->CurrWrite] = value;
		core->WritePc[core->CurrWrite] = core->Reg[15];
		core->CurrWrite++;
		//printf("In %s, pc=0x%x, addr=0x%x, data=0x%x\n", __FUNCTION__, core->Reg[15],  phys_addr | (virt_addr & 3), value);
		#if 0
		/* found the instruction of write action */
		if(value == 0xbee0f770 && core->Reg[15] == 0x400bb584){
			printf("############In %s,  va=0x%x, phys_addr=0x%x, value=0x%x\n", __FUNCTION__, virt_addr, phys_addr, value);
			int j, nptrs;
			#define SIZE 100
	       	   	void *buffer[100];
	        	   char **strings;

        		   nptrs = backtrace(buffer, SIZE);
		           printf("backtrace() returned %d addresses\n", nptrs);

           /* The call backtrace_symbols_fd(buffer, nptrs, STDOUT_FILENO)
              would produce similar output to the following: */

	           strings = backtrace_symbols(buffer, nptrs);
        	   if (strings == NULL) {
	               perror("backtrace_symbols");
        	       exit(EXIT_FAILURE);
	           }

        	   for (j = 0; j < nptrs; j++)
	               printf("%s\n", strings[j]);

        	   free(strings);

		}
	#endif
	}
#endif
	//printf("pc=0x%x, addr=0x%x, data=0x%x\n", core->Reg[15],  phys_addr | (virt_addr & 3), value);
#if CHECK_IN_WRITE 
        if (is_translated_code(cpu, phys_addr) || is_fast_interp_code(cpu, phys_addr)) {
		//printf("In %s, selfmodified code is 0x%x\n", __FUNCTION__, phys_addr);
		//printf("In %s, selfmodified code is 0x%x, pc=0x%x\n", __FUNCTION__, phys_addr, core->Reg[15]);
		//if(is_translated_code(cpu, phys_addr)){
	                //clear native code when code section was written.
		//printf("clear code cache 0x%x in %s\n", phys_addr, __FUNCTION__);
#if L3_HASHMAP
                clear_cache_item(cpu->dyncom_engine->fmap, phys_addr);
#else
		//fprintf(stderr, "Warnning: not clear the cache");
#endif
		//}
		//if(is_fast_interp_code(cpu, phys_addr)){
		flush_bb(phys_addr);
		//printf("In %s, end fast interp page 0x%x is written at 0x%x\n", __FUNCTION__, phys_addr & 0xFFFFF000, core->Reg[15]);
		//}
                clear_tag_page(cpu, phys_addr);
        }
#endif
#if FAST_MEMORY
        phys_addr = phys_addr | (virt_addr & 3);
        if(mem_write_directly(cpu, phys_addr, value, size) == 0){
		return;
        }
#endif
	//bus_write(size, phys_addr, value);
	if (size == 8) {
		bus_write(8, phys_addr | (virt_addr & 3), value);
	} else if (size == 16) {
		bus_write(16, phys_addr | (virt_addr & 3), value);
	} else {
		bus_write(32, phys_addr, value);
	}
}
static uint32_t arch_arm_check_mm(cpu_t *cpu, uint32_t addr, int count, uint32_t read)
{
	return 1;
}

int fill_tlb(arm_core_t* core){
	fault_t fault = NO_FAULT;
	int read;
	addr_t addr, phys_addr;
	addr = core->CP15[CP15(CP15_TLB_FAULT_ADDR)];
	if((core->CP15[CP15(CP15_TLB_FAULT_STATUS)] & 0xF0)== TLB_READ_MISS)
		read = 1;
	else if((core->CP15[CP15(CP15_TLB_FAULT_STATUS)] & 0xF0) == TLB_WRITE_MISS)
		read = 0;
	else /* NOT tlb */{
		/* something wrong */
		printf("not tlb fault , fault is %d, addr=0x%x\n", core->CP15[CP15(CP15_FAULT_STATUS)], core->AbortAddr);
		;
	}
	//printf("try to fill tlb for addr 0x%x\n", addr);
	fault = check_address_validity(core, addr, &phys_addr, read, DATA_TLB);
	if(fault != NO_FAULT){
		//LOG("mmu fault in %s addr is %x\n", __FUNCTION__, addr);
		//LOG("fault is %d\n", fault);
		//printf("In %s, fault happened, addr=0x%x, fault=%d\n", __FUNCTION__, addr, fault);
		core->abortSig = true;
		core->Aborted = ARMul_DataAbortV;
		core->AbortAddr = addr;
		core->CP15[CP15(CP15_FAULT_STATUS)] = fault & 0xff;
		core->CP15[CP15(CP15_FAULT_ADDRESS)] = addr;

		core->CP15[CP15(CP15_TLB_FAULT_STATUS)] = 0x0;
		core->CP15[CP15(CP15_TLB_FAULT_ADDR)] = 0xdeadc0de;

		return 1;
	}
	else{
		core->CP15[CP15(CP15_TLB_FAULT_STATUS)] = 0x0;
		core->CP15[CP15(CP15_TLB_FAULT_ADDR)] = 0xdeadc0de;
		/* fill tlb successfully */
		return 0;
	}
}

static int arch_arm_effective_to_physical(cpu_t *cpu, uint32_t addr, uint32_t *result){
        //arm_core_t* core = (arm_core_t*)get_cast_conf_obj(cpu->cpu_data, "arm_core_t");
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	fault_t fault = NO_FAULT;
	addr_t phys_addr;
	//fault = get_phys_addr(cpu, addr, &phys_addr, 32, 1);

	fault = check_address_validity(core, addr, &phys_addr, 1, INSN_TLB);
	if (fault) {
		LOG("mmu fault in %s addr is %x\n", __FUNCTION__, addr);
		LOG("fault is %d\n", fault);
		core->abortSig = true;
		core->Aborted = ARMul_PrefetchAbortV;
		core->AbortAddr = addr;
		core->CP15[CP15(CP15_INSTR_FAULT_STATUS)] = fault & 0xff;
		core->CP15[CP15(CP15_FAULT_ADDRESS)] = addr;
		return fault;
	}
	*result = phys_addr;
	return 0;
}


arch_mem_ops_t arm_dyncom_mem_ops = {
	is_inside_page,
	is_page_start,
	is_page_end,
	arch_arm_read_memory,
	arch_arm_write_memory,
	arch_arm_check_mm,
	arch_arm_effective_to_physical
};
