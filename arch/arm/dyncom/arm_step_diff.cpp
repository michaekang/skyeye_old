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
* @file arm_step_diff.cpp
* @brief The step diff with the other simulator
* @author Michael.Kang blackfin.kang@gmail.com
* @version 78.77
* @date 2011-12-31
*/
#include "arm_dyncom_mmu.h"
#include "arm_dyncom_run.h"
#include "armdefs.h"
#include <skyeye_class.h>
#include <skyeye_core_intf.h>
#include <skyeye_interface.h>
#include <memory_space.h>

/**
* @brief Diff with old interpreter in every step
*
* @param cpu
*/
int diff_single_step(cpu_t *cpu){
#if 1
	static conf_object_t* arm11_core_obj = NULL;
	static core_run_intf* arm_run = NULL;
	static arm_core_t* state = NULL;	
	uint32_t temp_cpsr;
	int i;

	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	core->debug_icounter = 0;
	//core->debug_icounter = 310008252;
	if(core->icounter % 10000000 == 0)
		printf("ICOUNTER=%lld\n", core->icounter);
	#if 0
	static int begin_print = 0;
	static FILE* log = NULL;
	if(log == NULL){
		log = fopen("./log.txt", "w");
		if(log == NULL){
			exit(-1);
		}	
	}
	if(begin_print || ((core->icounter > core->debug_icounter) && (core->Reg[15] == 0x93a8))){
		begin_print = 1;
		if(core->icounter >= 163018754){
		fprintf(log, "\ncpu_ICOUNTER=%lld, pc=0x%x, cpsr=0x%x\n", cpu->icounter, *(addr_t*)cpu->rf.pc,  core->Cpsr);
		fprintf(log,"----------------------------\n");
		fprintf(log, "ICOUNTER=%lld(0x%x), diff Fail, last_pc=0x%x,\n",core->icounter, core->Reg[15], core->last_pc);
		int j;
		uint32 regval;
		for(j = 0; j < 16; j++){
			fprintf(log, "R[%d]=0x%x\t", j, core->Reg[j]);
		}
		fprintf(log, "\n\n");
		}
		//dump_ir(cpu);
		//if(core->Reg[15] >= 0xa11c && core->Reg[15] <= 0xa17c){
		#if 0
		skyeye_printf_in_color(PURPLE, "cpu_ICOUNTER=%lld, effec_page=0x%x, pc=0x%x, phys_pc=0x%x, cpsr=0x%x\n", cpu->icounter, cpu->current_page_effec, *(addr_t*)cpu->rf.pc, *(uint32_t*)cpu->rf.phys_pc, core->Cpsr);
		skyeye_printf_in_color(PURPLE, "ICOUNTER=%lld(0x%x), diff Fail, last_pc=0x%x,\n",core->icounter, core->Reg[15], core->last_pc);
		int j;
		uint32 regval;
		for(j = 0; j < 16; j++){
			skyeye_printf_in_color(BLUE, "R[%d]=0x%x\t", j, core->Reg[j]);
		}
		printf("\n");
		#endif
		//}
	}
	#endif
	if(core->last_pc == core->Reg[15])
                return 0;
        core->last_pc = core->Reg[15];

	if(core->Reg[15] == 0xa134 && core->Reg[14] == 0)
		exit(-1);
	if(core->icounter < core->debug_icounter)
		return 0;
	cpu_set_flags_debug(cpu, 0
	//	| CPU_DEBUG_PRINT_IR
	//	| CPU_DEBUG_LOG
	//	| CPU_DEBUG_PROFILE
               );

	/* initialization for original interpreter */
	if(arm11_core_obj == NULL){
		/* initilize a arm11 core */
		arm11_core_obj = pre_conf_obj("arm11_core_0", "arm11_core");
		/* get the interface from the core */
		arm_run = (core_run_intf*)SKY_get_interface(arm11_core_obj, CORE_RUN_INTF_NAME);
		core_signal_intf* core_signal = (core_signal_intf*)SKY_get_interface(arm11_core_obj, CORE_SIGNAL_INTF_NAME);
		memory_space_intf* core_space = (memory_space_intf*)SKY_get_interface(arm11_core_obj, MEMORY_SPACE_INTF_NAME);

		/* initlize the s3c6410 machine */
		conf_object_t* s3c6410_mach = pre_conf_obj("s3c6410_mach_0", "s3c6410_mach");
		/* get the interface from the machine */	
		core_signal_intf* mach_signal = (core_signal_intf*)SKY_get_interface(s3c6410_mach, CORE_SIGNAL_INTF_NAME);
		memory_space_intf* mach_space = (memory_space_intf*)SKY_get_interface(s3c6410_mach, MEMORY_SPACE_INTF_NAME);

		/* connect the signal line between core and mach */	
		if(mach_signal->obj != NULL || mach_signal->signal != NULL){
			SKYEYE_ERR("Wrong value for interface\n");
			exit(-1);
		}
		mach_signal->obj = core_signal->obj;
		mach_signal->signal = core_signal->signal;

		/* connect the address space between core and mach */
		if(core_space->conf_obj != NULL || core_space->read != NULL || core_space->write != NULL)
		{
			SKYEYE_ERR("Wrong value for interface\n");
			exit(-1);

		}	
		core_space->conf_obj = mach_space->conf_obj;
		core_space->read = mach_space->read;
		core_space->write = mach_space->write;
		/* Should keep the same with the one in arm11_core.c */
		typedef struct arm11_core{
			conf_object_t* obj;
			ARMul_State* state;
			memory_space_intf* space;
		}arm11_core_t;
		arm11_core_t* arm11_core = (arm11_core_t*)(arm11_core_obj->obj);
		state = arm11_core->state;
		state->mmu.control = core->CP15[CP15(CP15_CONTROL)];
		state->mmu.auxiliary_control = core->CP15[CP15(CP15_AUXILIARY_CONTROL)];
		state->mmu.coprocessor_access_control = core->CP15[CP15(CP15_COPROCESSOR_ACCESS_CONTROL)];
		state->mmu.domain_access_control = core->CP15[CP15(CP15_DOMAIN_ACCESS_CONTROL)];
		state->mmu.fault_status = core->CP15[CP15(CP15_FAULT_STATUS)];
		state->mmu.fault_address = core->CP15[CP15(CP15_FAULT_ADDRESS)];
		state->mmu.translation_table_ctrl = core->CP15[CP15(CP15_TRANSLATION_BASE_CONTROL)];
		state->mmu.translation_table_base0 = core->CP15[CP15(CP15_TRANSLATION_BASE_TABLE_0)];
		state->mmu.translation_table_base1 = core->CP15[CP15(CP15_TRANSLATION_BASE_TABLE_1)];
		state->mmu.process_id = core->CP15[CP15(CP15_PID)];
		state->mmu.context_id = core->CP15[CP15(CP15_CONTEXT_ID)];
		state->mmu.thread_uro_id = core->CP15[CP15(CP15_THREAD_URO)];
		state->VFP[VFP_OFFSET(VFP_FPSCR)] = core->VFP[VFP_OFFSET(VFP_FPSCR)];
		/* the core os*/
		#if 0
		arm11_core = (arm_core_t*)(arm11_core_obj->obj);
		arm11_core->mmu.phys_space.conf_obj = mach_space->conf_obj;
		arm11_core->mmu.phys_space.read = mach_space->read;
		arm11_core->mmu.phys_space.write = mach_space->write;
		#endif
		/* machine ID for SMDK6410 */
		arm_run->set_regval_by_id(arm11_core_obj, 1, 1626);
		//arch_instance->set_regval_by_id(1, 1626);			/* Sync all the register with fast interpreter */
		for(i = 0; i < 16; i++){
			arm_run->set_regval_by_id(arm11_core_obj, i, core->Reg[i]);
		}

		core->CurrWrite = state->CurrWrite = 0;
		state->debug_icounter = core->debug_icounter;
	}
	state->icounter = core->icounter;

	/* the code of fast interpreter */
	//printf("ICOUNTER(0x%x)\n", core->icounter);
#if 0
	if(core->icounter == 100)
		exit(0);
#endif	
	static int last_tflag = 0;
	if((last_tflag != core->TFlag) && 0){
	//if(core->Reg[15] <= 0xc0000000){
		last_tflag = core->TFlag;
		if(core->TFlag){
			skyeye_printf_in_color(BLUE, "\nSwitch to THUMB state, at pc=0x%x, instr=0x%x, last_pc=0x%x, last_instr=0x%x\n", core->Reg[15], state->CurrInstr, state->last_pc, state->last_instr);
			;
		}
		else{
			skyeye_printf_in_color(BLUE, "\nSwitch to ARM state, at pc=0x%x, instr=0x%x, last_pc=0x%x, last_instr=0x%x\n", core->Reg[15], state->CurrInstr, state->last_pc, state->last_instr);
			;
		}
	}
	static uint32_t spsr_copy = core->Spsr_copy;
	static uint32_t spsr_copy_1, spsr_copy_2, spsr_copy_pc;
	#if 0
	if(spsr_copy != core->Spsr_copy){
		//printf("\n!!!!!!!!!!!!!!! 3\n");
		uint32_t regval;
		uint32_t instr = arm_run->get_regval_by_id(arm11_core_obj, 0xFF);
		spsr_copy_1 = spsr_copy;
		spsr_copy_2 = core->Spsr_copy;
		spsr_copy_pc = core->Reg[15];
		skyeye_printf_in_color(RED, "ICOUNTER=%lld(0x%x), last_pc=0x%x, last_instr=0x%x, instr=0x%x, diff Fail, orginal spsr_copy  0x%x, spsr_copy=0x%x\n",core->icounter, core->Reg[15], state->last_pc, state->last_instr, instr, spsr_copy, core->Spsr_copy);
		spsr_copy = core->Spsr_copy;
	}
	#if 0
	if(core->Reg[15] == 0xc002df8c)
		printf("@@@@@@@@@@@@@@@@@@@@@ in %s, cpsr=0x%x, mode=0x%x ,spsr_copy=0x%x\n", __FUNCTION__, core->Cpsr, core->Mode, core->Spsr_copy);
	#endif
	#endif
	if(core->Reg[13] >= 0xc0000000 && (core->Reg[15] < 0xc0000000) && (core->icounter > core->debug_icounter)){
		printf("\n#######################3\n");
		uint32_t regval;
		uint32_t instr = arm_run->get_regval_by_id(arm11_core_obj, 0xFF);
		skyeye_printf_in_color(RED, "ICOUNTER=%lld(0x%x), last_pc=0x%x, last_instr=0x%x, instr=0x%x, diff Fail, orginal R[%d]=0x%x, wrong value 0x%x, spsr_copy=0x%x\n",core->icounter, core->Reg[15], state->last_pc, state->last_instr, instr, i, regval, core->Reg[i], core->Spsr_copy);
		int j;
		for(j = 0; j < 16; j++){
			regval = arm_run->get_regval_by_id(arm11_core_obj, j);
			skyeye_printf_in_color(BLUE, "R[%d]=0x%x:0x%x\t", j, regval, core->Reg[j]);
		}
		skyeye_printf_in_color(BLUE, "\norginal CPSR=0x%x, wrong value 0x%x\n", arm_run->get_regval_by_id(arm11_core_obj, CPSR_REG), core->Cpsr);
		printf("#######################3\n");
		printf("spsr_copy change from 0x%x to 0x%x at 0x%x\n", spsr_copy_1, spsr_copy_2, spsr_copy_pc);
		//exit(0);
	}
#if 0
	if(core->TFlag || state->TFlag){
		/* for BLX instruction of thumb mode */
		if(((state->last_instr  & 0xF8000000) >> 27) == 31)
			return 0;
	}
#endif
	if(core->Reg[15] == 0xffff0214 
#if 0 /* android testcase */
		|| core->Reg[15] == 0xffff020c /* irq */
		|| core->Reg[15] == 0xffff0018 /* irq */
		|| core->Reg[15] == 0xc002dba0 /* irq_svc */
		|| core->Reg[15] == 0xc002dbdc
		|| core->Reg[15] == 0xc002fcc0 /* __irq_svc */
		|| core->Reg[15] == 0xc002fd10 /* __irq_svc */
		|| core->Reg[15] == 0xc004abe4
		|| core->Reg[15] == 0xc004678c
		|| core->Reg[15] == 0xc002dda0 /* irq_usr */
		|| core->Reg[15] == 0xffff0208 /* irq */
		|| core->Reg[15] == 0xffff0010 /* abort irq */
		|| core->Reg[15] == 0xffff000c /* abort irq */
		|| core->Reg[15] == 0xc0049700 /* interrupt enable */
		|| core->Reg[15] == 0xffff0308 /* irq */
		|| core->Reg[15] == 0xffff030c /* irq */
		|| core->Reg[15] == 0xffff0310 /* irq */
		|| core->Reg[15] == 0xffff0314 /* irq */
		|| core->Reg[15] == 0xffff0318 /* irq */

		|| core->Reg[15] == 0xffff0288 /* irq */
		|| core->Reg[15] == 0xc002da80 /* irq */
		|| core->Reg[15] == 0xc002db20 /* irq */
		|| core->Reg[15] == 0xc002dad0 /* irq */
		|| core->Reg[15] == 0xc002dc80 /* irq */
		|| core->Reg[15] == 0xc002dca0 /* irq */
		|| core->Reg[15] == 0xc002dae0 /* irq */
		|| core->Reg[15] == 0xc0033f88 /* serial */
		|| core->Reg[15] == 0xc0033fb8 /* serial */
		|| core->Reg[15] == 0xc0193d10 /* serial */
		|| core->Reg[15] == 0xc002fd20 /* serial */
		|| core->Reg[15] == 0xc003ce64 /* serial */
		|| core->Reg[15] == 0xc003a4bc /* serial */
		|| core->Reg[15] == 0xc00b651c /* serial */
		|| core->Reg[15] == 0xc00e61bc /* serial */
		|| core->Reg[15] == 0xc00810ec /* serial */
		|| core->Reg[15] == 0xc0033f94 /* serial */
		|| core->Reg[15] == 0xc0193d24 /* serial */
		|| core->Reg[15] == 0xc002dccc /* serial */
		|| core->Reg[15] == 0xc0033ea0 /* serial */
		|| core->Reg[15] == 0xc004b0ec /* serial */
		|| core->Reg[15] == 0xc002dcc0 /* serial */
		|| core->Reg[15] == 0xc0062e3c /* serial */
		|| core->Reg[15] == 0xc02a4e28 /* serial */
		|| core->Reg[15] == 0xc003941c /* serial */
		|| core->Reg[15] == 0xc0039420 /* serial */
		|| core->Reg[15] == 0xc0039434 /* serial */
#else /* Qt testcase */
		|| core->Reg[15] == 0xffff020c /* serial */
		|| core->Reg[15] == 0xc002dba0 /* serial */
		|| core->Reg[15] == 0xc002dbdc /* serial */
		|| core->Reg[15] == 0xc002dd60 /* serial */
		|| core->Reg[15] == 0xc002dd80 /* serial */
		|| core->Reg[15] == 0xc002dda0 /* serial */
		|| core->Reg[15] == 0xc0049cb4 /* serial */
		|| core->Reg[15] == 0xffff000c /* serial */
		|| core->Reg[15] == 0xffff0010 /* serial */
		|| core->Reg[15] == 0xffff0308 /* serial */
		|| core->Reg[15] == 0xc006101c /* serial */
		|| core->Reg[15] == 0xc0081808 /* serial */
		|| core->Reg[15] == 0xc017779c /* serial */
		|| core->Reg[15] == 0xc002dbec /* serial */
		|| core->Reg[15] == 0xc0034208 /* serial */
		|| core->Reg[15] == 0xc00341d8 /* serial */
		|| core->Reg[15] == 0xffff0310 /* serial */
		|| core->Reg[15] == 0xc0039ae0 /* serial */
		|| core->Reg[15] == 0xc0039ae4 /* serial */
		|| core->Reg[15] == 0xc0039ae8 /* serial */
		|| core->Reg[15] == 0xc0039af8 /* serial */
		|| core->Reg[15] == 0xffff0018 /* irq */
#endif
		)
	goto SYNC;
	/* diff all the register for last instruction */
#if 1
	//temp_cpsr = (core->Cpsr & 0x0fffffdf) | 
	core->Cpsr = (core->Cpsr & 0x0fffffdf) | \
                                                (core->NFlag << 31)   |                 \
                                                (core->ZFlag << 30)   |                 \
                                                (core->CFlag << 29)   |                 \
                                                (core->VFlag << 28)   |			\		
                                                (core->TFlag << 5);
	temp_cpsr = core->Cpsr;
#endif
	uint32 regval;
	uint32 instr;
	uint32 cpsr;
	for(i = 0; i < 16; i++){
		regval = arm_run->get_regval_by_id(arm11_core_obj, i);
		if(core->Cpsr & 0xF == 2){/* irq */
			if(i == 13){
				regval = arm_run->get_regval_by_id(arm11_core_obj, R13_IRQ);
			}
			else if(i == 14){
				regval = arm_run->get_regval_by_id(arm11_core_obj, R14_IRQ);
			}
		}
		if(core->Cpsr & 0xF == 3){/* svc */
			if(i == 13){
				regval = arm_run->get_regval_by_id(arm11_core_obj, R13_SVC);
			}
			else if(i == 14){
				regval = arm_run->get_regval_by_id(arm11_core_obj, R14_SVC);
			}
		}

		if(core->Cpsr & 0xF == 7){/* abort */
			if(i == 13){
				regval = arm_run->get_regval_by_id(arm11_core_obj, R13_ABORT);
			}
			else if(i == 14){
				regval = arm_run->get_regval_by_id(arm11_core_obj, R14_ABORT);
			}
		}

		if(regval != core->Reg[i]){
			instr = arm_run->get_regval_by_id(arm11_core_obj, 0xFF);
			skyeye_printf_in_color(RED, "ICOUNTER=%lld(0x%x), last_pc=0x%x, last_instr=0x%x, instr=0x%x, diff Fail, orginal R[%d]=0x%x, wrong value 0x%x, spsr_copy=0x%x\n",core->icounter, core->Reg[15], state->last_pc, state->last_instr, instr, i, regval, core->Reg[i], core->Spsr_copy);
			int j;
			for(j = 0; j < 16; j++){
				regval = arm_run->get_regval_by_id(arm11_core_obj, j);
				skyeye_printf_in_color(BLUE, "R[%d]=0x%x:0x%x\t", j, regval, core->Reg[j]);
			}
			skyeye_printf_in_color(BLUE, "\norginal CPSR=0x%x, wrong value 0x%x\n", arm_run->get_regval_by_id(arm11_core_obj, CPSR_REG), core->Cpsr);
			printf("\n");
		}
		//printf("R[%d]=0x%x:0x%x\t", i, regval, core->Reg[i]);
	}
	#if 0
	if(state->mmu.control != core->CP15[CP15(CP15_CONTROL)] ||
		state->mmu.domain_access_control != core->CP15[CP15(CP15_DOMAIN_ACCESS_CONTROL)]
		|| state->mmu.fault_status != core->CP15[CP15(CP15_FAULT_STATUS)]
		|| state->mmu.fault_address != core->CP15[CP15(CP15_FAULT_ADDRESS)]
		|| state->mmu.translation_table_ctrl != core->CP15[CP15(CP15_TRANSLATION_BASE_CONTROL)]
		|| state->mmu.translation_table_base0 != core->CP15[CP15(CP15_TRANSLATION_BASE_TABLE_0)]
		|| state->mmu.translation_table_base1 != core->CP15[CP15(CP15_TRANSLATION_BASE_TABLE_1)]
		|| state->mmu.process_id != core->CP15[CP15(CP15_PID)]
		|| state->mmu.context_id != core->CP15[CP15(CP15_CONTEXT_ID)]
		|| state->mmu.thread_uro_id != core->CP15[CP15(CP15_THREAD_URO)]
		){
		skyeye_printf_in_color(RED, "ICOUNTER=%lld(0x%x), diff Fail\n",core->icounter, core->Reg[15]);
		printf("control(0x%x : 0x%x)\n",state->mmu.control, core->CP15[CP15(CP15_CONTROL)]);
		printf("domain_access(0x%x : 0x%x)\n", state->mmu.domain_access_control, core->CP15[CP15(CP15_DOMAIN_ACCESS_CONTROL)]);
		printf("fault status (0x%x : 0x%x)\n", state->mmu.fault_status, core->CP15[CP15(CP15_FAULT_STATUS)]);
		printf("fault addr(0x%x, 0x%x)\n", state->mmu.fault_address, core->CP15[CP15(CP15_FAULT_ADDRESS)]);
		printf("table_ctrl(0x%x: 0x%x)\n", state->mmu.translation_table_ctrl, core->CP15[CP15(CP15_TRANSLATION_BASE_CONTROL)]);
		printf("base0 (0x%x : 0x%x)\n", state->mmu.translation_table_base0, core->CP15[CP15(CP15_TRANSLATION_BASE_TABLE_0)]);
		printf("base1 (0x%x : 0x%x)\n", state->mmu.translation_table_base1, core->CP15[CP15(CP15_TRANSLATION_BASE_TABLE_1)]);

		printf("state->mmu.process_id(0x%x : 0x%x)\n", state->mmu.process_id, core->CP15[CP15(CP15_PID)]);
		printf("state->mmu.context_id(0x%x : 0x%x)\n", state->mmu.context_id, core->CP15[CP15(CP15_CONTEXT_ID)]);
		printf("state->mmu.thread_id(0x%x : 0x%x)\n", state->mmu.thread_uro_id, core->CP15[CP15(CP15_THREAD_URO)]);
		state->mmu.fault_status = core->CP15[CP15(CP15_FAULT_STATUS)];
		state->mmu.fault_address = core->CP15[CP15(CP15_FAULT_ADDRESS)];

	}
	#endif
	#if 0
	if(core->Reg[15] == 0x87dc){
		//dump_ir(cpu);
		instr = arm_run->get_regval_by_id(arm11_core_obj, 0xFF);
		skyeye_printf_in_color(PURPLE, "ICOUNTER=%lld(0x%x), last_pc=0x%x, last_instr=0x%x, instr=0x%x, diff Fail, SUB PC\n",core->icounter, core->Reg[15], state->last_pc, state->last_instr, instr);
		int j;
		for(j = 0; j < 16; j++){
			regval = arm_run->get_regval_by_id(arm11_core_obj, j);
			skyeye_printf_in_color(BLUE, "R[%d]=0x%x:0x%x\t", j, regval, core->Reg[j]);
		}
		printf("\n");
	}
	#endif
	#if 0
	if(core->Reg[15] >= 0xc002da80 && core->Reg[15] <= 0xc002da80){
		instr = arm_run->get_regval_by_id(arm11_core_obj, 0xFF);
		skyeye_printf_in_color(RED, "ICOUNTER=%lld(0x%x), last_pc=0x%x, last_instr=0x%x, instr=0x%x, diff Fail, orginal R[%d]=0x%x, wrong value 0x%x, spsr_copy=0x%x\n",core->icounter, core->Reg[15], state->last_pc, state->last_instr, instr, i, regval, core->Reg[i], core->Spsr_copy);
		int j;
		for(j = 0; j < 16; j++){
			regval = arm_run->get_regval_by_id(arm11_core_obj, j);
			skyeye_printf_in_color(BLUE, "R[%d]=0x%x:0x%x\t", j, regval, core->Reg[j]);
		}
		skyeye_printf_in_color(BLUE, "\norginal CPSR=0x%x, wrong value 0x%x\n", arm_run->get_regval_by_id(arm11_core_obj, CPSR_REG), core->Cpsr);
		printf("\n");

	}
	#endif
	if((state->exclusive_access_state != core->exclusive_state) || (state->exclusive_tag_array[0] != core->exclusive_tag)){
		skyeye_printf_in_color(RED, "ICOUNTER=%lld(0x%x), last_pc=0x%x, last_instr=0x%x, instr=0x%x, diff Fail, access_state[%d:0x%x], tag_array[0x%x:0x%x]\n",core->icounter, core->Reg[15], state->last_pc, state->last_instr, instr, state->exclusive_access_state, core->exclusive_state, state->exclusive_tag_array[0], core->exclusive_tag);
		state->exclusive_access_state = core->exclusive_state;
		state->exclusive_tag_array[0] = core->exclusive_tag;
	}

	/* Comparing the VFP register */
	for(i = 0; i < VFP_REG_NUM; i++){
		if(core->ExtReg[i] != state->ExtReg[i]){
			skyeye_printf_in_color(RED, "ICOUNTER=%lld(0x%x), instr=0x%x, last_instr=0x%x, last_pc=0x%x, diff Fail, orginal ExtReg[%d]=0x%x, wrong ExtReg[%d] = 0x%x\n",core->icounter, core->Reg[15], instr, state->last_instr, state->last_pc, i, state->ExtReg[i], i, core->ExtReg[i]);
		}
	}
	cpsr = arm_run->get_regval_by_id(arm11_core_obj, CPSR_REG);
	if(((temp_cpsr & 0xF0000020) != (cpsr & 0xF0000020)) ||
	//if(((core->Cpsr & 0xF0000020) != (cpsr & 0xF0000020)) ||
		 (core->VFP[VFP_OFFSET(VFP_FPSCR)] != state->VFP[VFP_OFFSET(VFP_FPSCR)]) ||
		(core->VFP[VFP_OFFSET(VFP_FPEXC)] != state->VFP[VFP_OFFSET(VFP_FPEXC)])){
		instr = arm_run->get_regval_by_id(arm11_core_obj, 0xFF);
		skyeye_printf_in_color(RED, "ICOUNTER=%lld(0x%x), instr=0x%x, last_instr=0x%x, diff Fail, orginal CPSR=0x%x, wrong value 0x%x, spsr_copy=0x%x\n",core->icounter, core->Reg[15], instr, state->last_instr, arm_run->get_regval_by_id(arm11_core_obj, CPSR_REG), temp_cpsr, core->Spsr_copy);
		int j;
		for(j = 0; j < 16; j++){
			regval = arm_run->get_regval_by_id(arm11_core_obj, j);
			skyeye_printf_in_color(BLUE, "R[%d]=0x%x:0x%x\t", j, regval, core->Reg[j]);
		}
		printf("\n");
		skyeye_printf_in_color(BLUE, "fpsid=0x%x:0x%x\tfpscr=0x%x:0x%x, fpexc=0x%x:0x%x\n", state->VFP[VFP_OFFSET(VFP_FPSID)], core->VFP[VFP_OFFSET(VFP_FPSID)], state->VFP[VFP_OFFSET(VFP_FPSCR)], core->VFP[VFP_OFFSET(VFP_FPSCR)], state->VFP[VFP_OFFSET(VFP_FPEXC)], core->VFP[VFP_OFFSET(VFP_FPEXC)]);
	}
	/* Comparing the writen data */
	if((state->CurrWrite != 0) || (core->CurrWrite != 0)){
		if(state->CurrWrite != core->CurrWrite){
			skyeye_printf_in_color(RED, "ICOUNTER=%lld(0x%x:0x%x), instr=0x%x, last_instr=0x%x, last_pc=0x%x, Data Write diff Fail, orginal CurrWrite=%d, wrong CurrWrite %d\n",core->icounter, core->Reg[15], state->Reg[15], instr, state->last_instr, state->last_pc, state->CurrWrite, core->CurrWrite);
			int j;
			for(j = 0; j < 16; j++){
				regval = arm_run->get_regval_by_id(arm11_core_obj, j);
				skyeye_printf_in_color(BLUE, "R[%d]=0x%x:0x%x\t", j, regval, core->Reg[j]);
			}
		}
		for(i = 0; i < state->CurrWrite; i++){
			if(state->WriteAddr[state->CurrWrite] != core->WriteAddr[state->CurrWrite]){
				skyeye_printf_in_color(RED, "ICOUNTER=%d(0x%x), instr=0x%x, last_pc=0x%x, last_instr=0x%x, Data Write diff Fail, orginal WriteAddr=0x%x, wrong WriteAddr 0x%x\n",core->icounter, core->Reg[15], instr, state->last_pc, state->last_instr, state->WriteAddr[state->CurrWrite], core->WriteAddr[state->CurrWrite]);
			}
			if(state->WriteData[state->CurrWrite] != core->WriteData[state->CurrWrite]){
				skyeye_printf_in_color(RED, "ICOUNTER=%d(0x%x), instr=0x%x, Data Write diff Fail, orginal WriteData=0x%x, wrong WriteData 0x%x\n",core->icounter, core->Reg[15], instr, state->WriteData[state->CurrWrite], core->WriteData[state->CurrWrite]);
			}
			if((state->WritePc[state->CurrWrite] != core->WritePc[state->CurrWrite] + GET_INST_SIZE(core) * 2) && state->WritePc[state->CurrWrite] != 0){
				//skyeye_printf_in_color(RED, "ICOUNTER=%d(0x%x), instr=0x%x, last_instr=0x%x, last_pc=0x%x,Data Write diff Fail, orginal WritePc=0x%x, wrong WritePc 0x%x\n",core->icounter, core->Reg[15], instr, state->last_instr, state->last_pc, state->WritePc[state->CurrWrite], core->WritePc[state->CurrWrite] + GET_INST_SIZE(core) * 2);
			}
		}
	}
	//printf("\n");
SYNC:
	state->CurrWrite = core->CurrWrite = 0;
	/* Sync all the register with fast interpreter */
	for(i = 0; i < 16; i++){
		if(core->Cpsr & 0xF == 2){/* irq */
			if(i == 13){
				arm_run->set_regval_by_id(arm11_core_obj, R13_IRQ, core->Reg[i]);
			}
			else if(i == 14){
				arm_run->set_regval_by_id(arm11_core_obj, R14_IRQ, core->Reg[i]);
			}
			continue;
		}
		if(core->Cpsr & 0xF == 3){/* svc */
			if(i == 13){
				arm_run->set_regval_by_id(arm11_core_obj, R13_SVC, core->Reg[i]);
			}
			else if(i == 14){
				arm_run->set_regval_by_id(arm11_core_obj, R14_SVC, core->Reg[i]);
			}
			continue;
		}
		if(core->Cpsr & 0xF == 7){/* abort */
			if(i == 13){
				arm_run->set_regval_by_id(arm11_core_obj, R13_ABORT, core->Reg[i]);
			}
			else if(i == 14){
				arm_run->set_regval_by_id(arm11_core_obj, R14_ABORT, core->Reg[i]);
			}
			continue;
		}

		arm_run->set_regval_by_id(arm11_core_obj, i, core->Reg[i]);
	}
	/* Sync the VFP register */
	for(i = 0; i < VFP_REG_NUM; i++){
		state->ExtReg[i] = core->ExtReg[i];
	}

	arm_run->set_regval_by_id(arm11_core_obj, CPSR_REG, core->Cpsr);
	/* run the instruction before the fast interpreter */
	arm_run->step_once(arm11_core_obj);
#if 0
	skyeye_printf_in_color(BLUE, "After old interp run:");
	for(i = 0; i < 16; i++){
		regval = arm_run->get_regval_by_id(arm11_core_obj, i);
		skyeye_printf_in_color(BLUE, "R[%d]=0x%x:0x%x\t", i, regval, core->Reg[i]);
	}
	printf("\n");
#endif
#endif
	return 0;
}
