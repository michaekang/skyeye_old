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
	int i;

	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	core->debug_icounter = 100000000;
	if(core->icounter < core->debug_icounter)
		return 0;
	if(core->icounter % 10000000 == 0)
		printf("ICOUNTER=%lld\n", core->icounter);
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
		state->mmu.context_id = core->CP15[(CP15_CONTEXT_ID)];
		state->mmu.thread_uro_id = core->CP15[(CP15_THREAD_URO)];
		state->VFP[VFP_FPSCR] = core->VFP[VFP_FPSCR];
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
	if(last_tflag != core->TFlag){
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
#if 0
	if(core->TFlag || state->TFlag){
		/* for BLX instruction of thumb mode */
		if(((state->last_instr  & 0xF8000000) >> 27) == 31)
			return 0;
	}
#endif
	if(core->Reg[15] == 0xffff0018 || core->Reg[15] == 0xffff0214 
		|| core->Reg[15] == 0xffff020c /* irq */
		|| core->Reg[15] == 0xc002dba0 /* irq_svc */
		|| core->Reg[15] == 0xc002dbdc
		|| core->Reg[15] == 0xc002fcc0 /* __irq_svc */
		|| core->Reg[15] == 0xc002fd10 /* __irq_svc */
		|| core->Reg[15] == 0xc004abe4
		|| core->Reg[15] == 0xc004678c
		|| core->Reg[15] == 0xc002dda0 /* irq_usr */
		|| core->Reg[15] == 0xffff0208 /* irq */
		|| core->Reg[15] == 0xffff0010 /* abort irq */
		|| core->Reg[15] == 0xc0049700 /* interrupt enable */
		|| core->Reg[15] == 0xffff0308 /* irq */
		|| core->Reg[15] == 0xffff000c /* irq */
		|| core->Reg[15] == 0xffff0288 /* irq */
		|| core->Reg[15] == 0xc002da80 /* irq */
		|| core->Reg[15] == 0xc002db20 /* irq */
		|| core->Reg[15] == 0xc002dad0 /* irq */
		|| core->Reg[15] == 0xc002dae0 /* irq */
		|| core->Reg[15] == 0xc0033f88 /* serial */
		|| core->Reg[15] == 0xc0033fb8 /* serial */
		|| core->Reg[15] == 0xc0193d10 /* serial */
		|| core->Reg[15] == 0xc002fd20 /* serial */
		|| core->Reg[15] == 0xc003ce64 /* serial */
		|| core->Reg[15] == 0xc003a4bc /* serial */
		|| core->Reg[15] == 0xffff0fe0 /* serial */
		)
		goto SYNC;
	/* diff all the register for last instruction */
	core->Cpsr = (core->Cpsr & 0x0fffffdf) | \
                                                (core->NFlag << 31)   |                 \
                                                (core->ZFlag << 30)   |                 \
                                                (core->CFlag << 29)   |                 \
                                                (core->VFlag << 28)	|		\
                                                (core->TFlag << 5);

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
			skyeye_printf_in_color(RED, "ICOUNTER=%d(0x%x), last_instr=0x%x, instr=0x%x, diff Fail, orginal R[%d]=0x%x, wrong value 0x%x\n",core->icounter, core->Reg[15], state->last_instr, instr, i, regval, core->Reg[i]);
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
	cpsr = arm_run->get_regval_by_id(arm11_core_obj, CPSR_REG);
	if(((core->Cpsr & 0xF0000020) != (cpsr & 0xF0000020)) ||
		 (core->VFP[VFP_FPSCR] != state->VFP[VFP_FPSCR]) ||
		(core->VFP[VFP_FPEXC] != state->VFP[VFP_FPEXC])){
		instr = arm_run->get_regval_by_id(arm11_core_obj, 0xFF);
		skyeye_printf_in_color(RED, "ICOUNTER=%d(0x%x), instr=0x%x, last_instr=0x%x, diff Fail, orginal CPSR=0x%x, wrong value 0x%x\n",core->icounter, core->Reg[15], instr, state->last_instr, arm_run->get_regval_by_id(arm11_core_obj, CPSR_REG), core->Cpsr);
		int j;
		for(j = 0; j < 16; j++){
			regval = arm_run->get_regval_by_id(arm11_core_obj, j);
			skyeye_printf_in_color(BLUE, "R[%d]=0x%x:0x%x\t", j, regval, core->Reg[j]);
		}
		printf("\n");
		skyeye_printf_in_color(BLUE, "fpsid=0x%x:0x%x\tfpscr=0x%x:0x%x, fpexc=0x%x:0x%x\n", state->VFP[VFP_FPSID], core->VFP[VFP_FPSID], state->VFP[VFP_FPSCR], core->VFP[VFP_FPSCR], state->VFP[VFP_FPEXC], core->VFP[VFP_FPEXC]);
	}
	/* Comparing the writen data */
	if((state->CurrWrite != 0) || (state->CurrWrite != 0)){
		if(state->CurrWrite != core->CurrWrite){
			skyeye_printf_in_color(RED, "ICOUNTER=%d(0x%x:0x%x), instr=0x%x, Data Write diff Fail, orginal CurrWrite=%d, wrong CurrWrite %d\n",core->icounter, core->Reg[15], state->Reg[15], instr, state->CurrWrite, core->CurrWrite);
		}
		for(i = 0; i < state->CurrWrite; i++){
			if(state->WriteAddr[state->CurrWrite] != core->WriteAddr[state->CurrWrite]){
				skyeye_printf_in_color(RED, "ICOUNTER=%d(0x%x), instr=0x%x, Data Write diff Fail, orginal WriteAddr=0x%x, wrong WriteAddr 0x%x\n",core->icounter, core->Reg[15], instr, state->WriteAddr[state->CurrWrite], core->WriteAddr[state->CurrWrite]);
			}
			if(state->WriteData[state->CurrWrite] != core->WriteData[state->CurrWrite]){
				skyeye_printf_in_color(RED, "ICOUNTER=%d(0x%x), instr=0x%x, Data Write diff Fail, orginal WriteData=0x%x, wrong WriteData 0x%x\n",core->icounter, core->Reg[15], instr, state->WriteData[state->CurrWrite], core->WriteData[state->CurrWrite]);
			}
			if((state->WritePc[state->CurrWrite] != core->WritePc[state->CurrWrite] + GET_INST_SIZE(core) * 2) && state->WritePc[state->CurrWrite] != 0){
				skyeye_printf_in_color(RED, "ICOUNTER=%d(0x%x), instr=0x%x, Data Write diff Fail, orginal WritePc=0x%x, wrong WritePc 0x%x\n",core->icounter, core->Reg[15], instr, state->WritePc[state->CurrWrite], core->WritePc[state->CurrWrite]);
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

