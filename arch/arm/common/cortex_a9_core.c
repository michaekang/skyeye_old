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
* @file cortex_a9_core.c
* @brief The arm core class
* @author Michael.Kang blackfin.kang@gmail.com
* @version 78.77
* @date 2011-12-21
*/

#include "armdefs.h"
#include "armcpu.h"
#include "armemu.h"
#include "arm_regformat.h"

#include <skyeye_mm.h>
#include <skyeye_class.h>
#include <skyeye_core_intf.h>
#include <skyeye_interface.h>
#include <memory_space.h>
#define CP15(idx)       (idx - CP15_BASE)
typedef struct cortex_a9_core{
	conf_object_t* obj;
	ARMul_State* state;
	memory_space_intf* space;
}cortex_a9_core_t;

static void
cortex_a9_set_pc (conf_object_t* opaque, generic_address_t pc)
{
	cortex_a9_core_t* core = (cortex_a9_core_t*)opaque->obj;
	ARMul_State *state = core->state;
	state->Reg[15] = pc;
	return;
}
static void
cortex_a9_step_once (conf_object_t* opaque)
{
	//ARMul_DoInstr(state);
	cortex_a9_core_t* core = (cortex_a9_core_t*)opaque->obj;
	ARMul_State *state = core->state;
	if(state->space.conf_obj == NULL){
		state->space.conf_obj = core->space->conf_obj;
		state->space.read = core->space->read;
		state->space.write = core->space->write;
	}

	state->step++;
	state->cycle++;
	state->EndCondition = 0;
	state->stop_simulator = 0;
	state->NextInstr = RESUME;      /* treat as PC change */
	state->last_pc = state->Reg[15];
	//core->space->read(core->space->conf_obj, state->Reg[15], state->last_instr, 4);
	//state->Reg[15] = ARMul_DoProg(state);
        state->Reg[15] = ARMul_DoInstr(state);
	//core->space->read(core->space->conf_obj, state->Reg[15], state->CurrInstr, 4);
	state->Cpsr = (state->Cpsr & 0x0fffffdf) | \
                     (state->NFlag << 31)   |                 \
                     (state->ZFlag << 30)   |                 \
                     (state->CFlag << 29)   |                 \
                     (state->VFlag << 28)   |		\
                     (state->TFlag << 5);

        FLUSHPIPE;
}
static exception_t cortex_a9_set_regval_by_id(conf_object_t* opaque, int id, uint32 value){
	cortex_a9_core_t* core = (cortex_a9_core_t*)opaque->obj;
	ARMul_State *state = core->state;
	if(id == R13_IRQ)
		state->Reg_irq[0] = value;
	if(id == R14_IRQ)
		state->Reg_irq[1]= value;
	if(id == R13_SVC)
		state->Reg_svc[0] = value;
	if(id == R14_SVC)
		state->Reg_svc[1] = value;
	if(id == R13_ABORT)
		state->Reg_abort[0] = value;
	if(id == R14_ABORT)
		state->Reg_abort[1] = value;
        if(id == CP15_FAULT_STATUS)
                state->mmu.fault_status = value;
	if (id == CPSR_REG){
		state->Cpsr = value;
		state->NFlag = (value >> 31) & 0x1;
		state->ZFlag = (value >> 30) & 0x1;
		state->CFlag = (value >> 29) & 0x1;
		state->VFlag = (value >> 28) & 0x1;
		state->TFlag = (value >> 5) & 0x1;
		switch(value & MODEBITS){
			case 0x10:
				state->Mode = USER32MODE;
				break;
			case 0x12:
				state->Mode = IRQ32MODE;
				break;
			case 0x13:
				state->Mode = SVC32MODE;
				break;
		};
	}
	else
        state->Reg[id] = value;
        return No_exp;
}

/**
* @brief
*
* @param id
*
* @return
*/
static uint32 cortex_a9_get_regval_by_id(conf_object_t* opaque, int id){
	cortex_a9_core_t* core = (cortex_a9_core_t*)opaque->obj;
	ARMul_State *state = core->state;
	if(id == 0xFF)
		return state->CurrInstr;
	if(id == R13_IRQ)
		return state->Reg_irq[0];
	if(id == R14_IRQ)
		return state->Reg_irq[1];
	if(id == R13_SVC)
		return state->Reg_svc[0];
	if(id == R14_SVC)
		return state->Reg_svc[1];
	if(id == R13_ABORT)
		return state->Reg_abort[0];
	if(id == R14_ABORT)
		return state->Reg_abort[1];
	if(id == CP15_FAULT_ADDRESS)
		return state->mmu.fault_address;
	if(id == CP15_FAULT_STATUS)
		return state->mmu.fault_status;
	if (id == CPSR_REG){
		ARMword cpsr = state->Cpsr & 0x0FFFFFDF;
		cpsr |= (state->NFlag & 0x1) << 31;
	        cpsr |= (state->ZFlag & 0x1) << 30;
		cpsr |= (state->CFlag & 0x1) << 29;
	        cpsr |= (state->VFlag & 0x1) << 28;
		cpsr |= (state->TFlag & 0x1) << 5;
		return cpsr;
	}
	else
        return state->Reg[id];
}
fault_t
cortex_a9_read_byte (ARMul_State *state, ARMword virt_addr, ARMword *data){
	//mem_read(short size, int offset, uint32_t * value){
	return NO_FAULT;
}
fault_t
cortex_a9_read_halfword (ARMul_State *state, ARMword virt_addr,
			   ARMword *data){
	return NO_FAULT;
}
fault_t
cortex_a9_read_word (ARMul_State *state, ARMword virt_addr, ARMword *data){
	return NO_FAULT;
}
static fault_t
cortex_a9_write_byte (ARMul_State *state, ARMword virt_addr, ARMword data)
{
	return NO_FAULT;
}

static fault_t
cortex_a9_write_halfword (ARMul_State *state, ARMword virt_addr,
			    ARMword data)
{
	return NO_FAULT;
}

static fault_t
cortex_a9_write_word (ARMul_State *state, ARMword virt_addr, ARMword data)
{
	return NO_FAULT;
}
static exception_t arm_signal(conf_object_t* opaque, interrupt_signal_t *signal){
	cortex_a9_core_t* core = (cortex_a9_core_t*)opaque->obj;
	ARMul_State *state = core->state;

	arm_signal_t *arm_signal = &signal->arm_signal;
	if (arm_signal->irq != Prev_level)
		state->NirqSig = arm_signal->irq;
	if (arm_signal->firq != Prev_level)
		state->NfiqSig = arm_signal->firq;

	/* reset signal in arm dyf add when move sa1100 to soc dir  2010.9.21*/
	if (arm_signal->reset != Prev_level)
		state->NresetSig = arm_signal->reset;
	return No_exp;
}


/**
* @brief The initialization for arm core instance
*
* @return
*/
const static cpu_config_t cortex_a9_cpu_info = {"armv7",  "cortex_a9", 0x413Fc090, 0x0000fff0, NONCACHE};
static conf_object_t* new_cortex_a9_core(char* obj_name){
	cortex_a9_core_t* core = skyeye_mm_zero(sizeof(cortex_a9_core_t));
	core->obj = new_conf_object(obj_name, core);
	ARMul_EmulateInit ();
	ARMul_State *state = skyeye_mm_zero(sizeof(ARMul_State));
	ARMul_NewState (state);
	state->abort_model = 0;
	state->cpu = &cortex_a9_cpu_info;
	state->bigendSig = LOW;

	ARMul_SelectProcessor (state, ARM_v7_Prop);
	state->lateabtSig = LOW;
	mmu_init(state);
	/* reset the core to initial state */
	ARMul_Reset (state);
	state->NextInstr = 0;
	state->Emulate = 3;
	core->state = state;

	core_run_intf* run = skyeye_mm_zero(sizeof(core_run_intf));
	run->conf_obj = core->obj;
	run->set_pc = cortex_a9_set_pc;
	run->step_once = cortex_a9_step_once;
	run->get_regval_by_id = cortex_a9_get_regval_by_id;
	run->set_regval_by_id = cortex_a9_set_regval_by_id;
	SKY_register_interface(run, obj_name, CORE_RUN_INTF_NAME);

	core_signal_intf* core_signal = skyeye_mm_zero(sizeof(core_signal_intf));
	core_signal->obj = core->obj;
	core_signal->signal = arm_signal;
	SKY_register_interface(core_signal, obj_name, CORE_SIGNAL_INTF_NAME);
	/* Register io function to the object */
	memory_space_intf* master_space = skyeye_mm_zero(sizeof(memory_space_intf));
	master_space->conf_obj = NULL;
	master_space->read = NULL;
	master_space->write = NULL;
	SKY_register_interface(master_space, obj_name, MEMORY_SPACE_INTF_NAME);
	core->space = master_space;
	state->space.conf_obj = NULL;
	return core->obj;
}

static void free_cortex_a9_core(conf_object_t* dev){

}
void init_cortex_a9_core(){
	static skyeye_class_t class_data = {
		.class_name = "cortex_a9_core",
		.class_desc = "cortex_a9_core",
		.new_instance = new_cortex_a9_core,
		.free_instance = free_cortex_a9_core,
		.get_attr = NULL,
		.set_attr = NULL
	};

	SKY_register_class(class_data.class_name, &class_data);
}
