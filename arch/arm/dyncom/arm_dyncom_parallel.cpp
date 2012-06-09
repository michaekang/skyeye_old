/* 
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2, or (at your option)
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  */

/*
 * 03/27/2011   Michael.Kang  <blackfin.kang@gmail.com> 
 * 10/2011 rewritten by Alexis He <ahe.krosk@gmail.com> 
 */
#include "armdefs.h"
#include "armcpu.h"
#include "arm_dyncom_parallel.h"
#include "arm_dyncom_mmu.h"
#include "arm_dyncom_translate.h"
#include "arm_dyncom_interpreter.h"

#include <pthread.h>

#include "skyeye_dyncom.h"
#include "skyeye_thread.h"
#include "stat.h"
#include "dyncom/tag.h"
#include "dyncom/basicblock.h"
#include "dyncom/phys_page.h"
#include "bank_defs.h"

#include <stack>
#include <hash_map>
using namespace std;
#ifndef __ARMEMU_H__
#define __ARMEMU_H__
/* Different ways to start the next instruction.  */
#define SEQ           0
#define NONSEQ        1
#define PCINCEDSEQ    2
#define PCINCEDNONSEQ 3
#define PRIMEPIPE     4
#define RESUME        8

#ifdef __cplusplus
extern "C" {
#endif
extern ARMword ARMul_Emulate32 (ARMul_State *);
#ifdef __cplusplus
}
#endif

#define INTBITS (0xc0L)
#endif

#define QUEUE_LENGTH 1024
/* Monothread: threshold compilation only
   Multithread: threshold compilation or Polling compilation (cpu intensive) */
#define MULTI_THREAD 0
#define LIFO 0
static uint32_t compiled_queue[QUEUE_LENGTH]; /* list of tagged addresses. Note: is not a shared resource */
static stack<uint32_t> compile_stack; /* stack of untranslated addresses. Note: is a shared resource */
static stack<uint8_t> func_attr_stack; /* record the attribute for untranslated function . Note: is a shared resource */
static pthread_rwlock_t compile_stack_rwlock;
static pthread_rwlock_t translation_rwlock;
static uint32_t translated_block = 0; /* translated block count, for block threshold */
static void* compiled_worker(void* cpu);
static void push_compiled_work(cpu_t* cpu, uint32_t pc, uint8_t func_attr);
/*
 * Three running mode: PURE_INTERPRET, PURE_DYNCOM, HYBRID
 */
//running_mode_t running_mode = PURE_INTERPRET;
running_mode_t running_mode = PURE_DYNCOM;
//running_mode_t running_mode = HYBRID;
static char* running_mode_str[] = {
	"pure interpret running",
	"pure dyncom running",
	"hybrid running",
	"fast interpret running",
	NULL
};


/**
* @brief the handler for log option
*
* @param this_option
* @param num_params
* @param params[]
*
* @return 
*/
int
do_mode_option (skyeye_option_t * this_option, int num_params,
	       const char *params[])
{
	char name[MAX_PARAM_NAME], value[MAX_PARAM_NAME];
	running_mode_t mode = running_mode;
	int i;
	for (i = 0; i < num_params; i++) {
		if (split_param (params[i], name, value) < 0){
			SKYEYE_ERR
				("log_info: Error: log has wrong parameter \"%s\".\n",
				 name);
			continue;
		}
		if (!strncmp ("mode", name, strlen (name))) {
			sscanf (value, "%d", &mode);
			if (mode < PURE_INTERPRET || mode >= MAX_RUNNING_MODE){
				SKYEYE_ERR
					("log_info: Error log level %d\n",
					 mode);
			}
			else
				running_mode = mode;
			break;
		}
	}
	return 0;
}
void init_compiled_queue(cpu_t* cpu){
	memset(&compiled_queue[0], 0xff, sizeof(uint32_t) * QUEUE_LENGTH);
	if (get_skyeye_pref()->interpret_mode) {
		running_mode = PURE_INTERPRET;
	}
	skyeye_log(Info_log, __FUNCTION__, "Current running mode: %s\n", running_mode_str[running_mode]);
	//if (running_mode == HYBRID){
		if(pthread_rwlock_init(&compile_stack_rwlock, NULL)){
			fprintf(stderr, "can not initilize the rwlock\n");
		}
		if(pthread_rwlock_init(&translation_rwlock, NULL)){
			fprintf(stderr, "can not initilize the rwlock\n");
		}
	#if MULTI_THREAD
		/* Create a thread to compile IR to native code */
		pthread_t id;
		create_thread(compiled_worker, (void*)cpu, &id);
	#endif
	//}
	cpu->dyncom_engine->cur_tagging_pos = 0;
}

void interpret_cpu_step(conf_object_t * running_core){
	arm_core_t *state = (arm_core_t *)running_core->obj;

	/* Initialize interpreter step */
	state->step++;
	state->cycle++;
	state->EndCondition = 0;
	if (running_mode == HYBRID)
		state->stop_simulator = 1;
	else
		state->stop_simulator = 1; /* 1 if debugging */
	
	/* There might be an issue with Emulate26 */
	state->pc = ARMul_Emulate32(state);
	
	return;
}


static int flush_current_page(cpu_t *cpu){
	//arm_core_t* core = (arm_core_t*)(cpu->cpu_data);
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	addr_t effec_pc = *(addr_t*)cpu->rf.pc;
	//printf("effec_pc is %x\n", effec_pc);
	//printf("in %s\n", __FUNCTION__);
	int ret; //= cpu->mem_ops.effective_to_physical(cpu, effec_pc, (uint32_t*)cpu->rf.phys_pc);
	ret = cpu->mem_ops.effective_to_physical(cpu, effec_pc, &core->phys_pc);
	if (ret != 0)
	{
		printf("##### in %s, effec_pc %x phys_pc %x ret %d\n", __FUNCTION__, effec_pc, *((uint32_t*)cpu->rf.phys_pc), ret);
	}
	cpu->current_page_phys = core->phys_pc & 0xfffff000;
	cpu->current_page_effec = core->pc & 0xfffff000;
	return ret;
}

/* This function clears the whole basic block cache (translation and tags), and frees the memory */

inline int clear_cache(cpu_t *cpu, fast_map hash_map)
{
#if 0
	uint32_t index;
	void* pfunc = NULL;
	for(index = 0; index <= cpu->dyncom_engine->cur_tagging_pos; index++) {
		compiled_queue[index] = -1;
		cpu->dyncom_engine->startbb[index].clear();
	}

	clear_fmap(hash_map);
	clear_tag_table(cpu);
	cpu_flush(cpu);
	cpu->dyncom_engine->cur_tagging_pos = 0;
	translated_block = 0;
	//cpu->dyncom_engine->exec_engine = ExecutionEngine::create(cpu->dyncom_engine->mod);
#endif
	return No_exp;
}

void clear_translated_cache(addr_t phys_addr){
	arm_core_t* core = get_current_core();
        cpu_t* cpu = (cpu_t *)core->dyncom_cpu->obj;
	clear_cache_item(cpu->dyncom_engine->fmap, phys_addr);
	clear_tag_page(cpu, phys_addr);
#if 0
	phys_addr = phys_addr & 0xFFFFF000;
        /* flush two pages of code cache for dyncom */
        for(int i = 0; i < 1024 * 2; i++){
                //phys_addr = phys_addr + 4;
                if (is_translated_code(cpu, phys_addr)) {
                        //clear native code when code section was written.
                        addr_t addr = find_bb_start(cpu, phys_addr);
                        //clear_tag(cpu, phys_addr);
#if L3_HASHMAP
                        extern pthread_rwlock_t translation_rwlock;
                        pthread_rwlock_wrlock(&translation_rwlock);
                        clear_cache_item(cpu->dyncom_engine->fmap, addr);
                        pthread_rwlock_unlock(&translation_rwlock);
#else
			//fprintf(stderr, "Warnning: not clear the cache");
			cpu->dyncom_engine->fmap[(phys_addr) & (HASH_FAST_MAP_SIZE - 1)] = (void *)func_not_found_func;
#endif
                }
                phys_addr = phys_addr + 2;
        }
#endif
}
/* Only for HYBRID .
   In HYBRID mode, when encountering a new (untagged) pc, we recursive-tag it so all newly tagged
   instructions belongs to this basic block. A translated address, if it is an entry point,
   will be dyncom-executed. Else, it will be interpreted. */
int launch_compiled_queue(cpu_t* cpu, uint32_t pc){
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	void * pfunc = NULL;
	
	/* Attempt to retrieve physical address if mmu, only in kernel mode */
	if (is_user_mode(cpu))
	{
		core->phys_pc = pc;
	}
	else 
	{
		uint32_t ret = 0;
		ret = cpu->mem_ops.effective_to_physical(cpu, pc, &core->phys_pc);
		/* targeted instruction is not in memory, let the interpreter prefetch abort do the job */
		if (ret != 0)
		{
			/* prefetch abort happened */
			return 0;
		}
	}
	
	
	/* Check if the wanted instruction is in the dyncom engine */
	fast_map hash_map = cpu->dyncom_engine->fmap;
	PFUNC(core->phys_pc);

	if (!pfunc)
	{	
		/* The instruction is not is the engine, we interpret it */
		//printf("Interpreting %p-%p with MMU %x\n", core->phys_pc, pc, (core->mmu.control));
		//compiled_queue[(++cur_compile_pos )% QUEUE_LENGTH] = core->phys_pc;
		//bb_prof
		//pthread_rwlock_wrlock(&compile_stack_rwlock);
		//if(cur_compile_pos == QUEUE_LENGTH){
		int ret;
		if((ret = pthread_rwlock_trywrlock(&compile_stack_rwlock)) == 0){ 
			compile_stack.push(core->phys_pc);
			pthread_rwlock_unlock(&compile_stack_rwlock);
		}
		else{
			printf("Warning ,can not get the wrlock ,error is %d, %s\n", ret, strerror(ret));
		}
		//}
		extern void InterpreterMainLoop(cpu_t *core);
		InterpreterMainLoop(cpu);

		return 0;
	}
	else
	{
		/* pure dyncom */
		launch_compiled_queue_dyncom(cpu, pc);
		return 0;
	}
}

static int handle_fp_insn(arm_core_t* core);
/* For PURE_DYNCON mode. This one
   is a direct copy of the old one,
   and is far less complicated */
int launch_compiled_queue_dyncom(cpu_t* cpu, uint32_t pc) {
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	void * pfunc = NULL;
	
	/* set correct pc */
	if (is_user_mode(cpu)){
		core->phys_pc = pc;
	}
	
	/* if cpu_run doesn't find the address of the block, it will return asap */
	int rc = JIT_RETURN_NOERR;
	if (is_user_mode(cpu))
		rc = um_cpu_run(cpu);
	else
		rc = cpu_run(cpu);
	
	/* General rule: return 1 if next block should be handled by Dyncom */
	switch (rc) {
	case JIT_RETURN_NOERR: /* JIT code wants us to end execution */
	case JIT_RETURN_TIMEOUT:
		//printf("Timeout - Next handling by DYNCOM %x\n", core->Reg[15]);
		return 1;
	case JIT_RETURN_SINGLESTEP:
		/* TODO */
	case JIT_RETURN_FUNC_BLANK:
	case JIT_RETURN_FUNCNOTFOUND:
	{
		//printf("pc %x is not found, phys_pc is %p\n", core->Reg[15], core->phys_pc);
		if (!is_user_mode(cpu))
		{
			//switch_mode(core, core->Cpsr & 0x1f);
			if (flush_current_page(cpu)) {
				return 1;
			}
		}
		/* keep the tflag same with the bit in CPSR */
		core->TFlag = (core->Cpsr & (1 << THUMB_BIT)) ? 1 : 0;
		//cpu->TFlag = core->TFlag;
		//cpu->user_mode = USER_MODE(core);
		//clear_tag_page(cpu, core->phys_pc); /* do it or not ? */
		uint8_t func_attr = FUNC_ATTR_NONE;
		if(core->TFlag)
                	func_attr |= FUNC_ATTR_THUMB;
		if(core->Reg[15] < 0xc0000000)
                	func_attr |= FUNC_ATTR_USERMODE;
		push_compiled_work(cpu, core->phys_pc, func_attr); // in usermode, it might be more accurate to translate reg[15] instead
		return 0;
	}
	case JIT_RETURN_TRAP:
	{
		/* user mode handling */
		if (is_user_mode(cpu))
		{
			core->phys_pc = core->Reg[15];
			if(handle_fp_insn(core) == 0){
		#if OPT_LOCAL_REGISTERS
			uint32_t instr;
			bus_read(32, core->Reg[15], &instr);
			ARMul_OSHandleSWI(core, BITS(0,19));
		#endif
			}
			core->Reg[15] += get_instr_size(cpu);
			//if (get_skyeye_pref()->start_logging)
			//	printf("Trap - Handled %x\n", core->phys_pc);
			return 0;
		}

		if((core->CP15[CP15(CP15_TLB_FAULT_STATUS)] & 0xf0)){
			//printf("\n\n###############In %s, fsr=0x%x, fault_addr=0x%x, pc=0x%x\n\n", __FUNCTION__, core->CP15[CP15(CP15_FAULT_STATUS)], core->CP15[CP15(CP15_FAULT_ADDRESS)], core->Reg[15]);
			//core->Reg[15] -= get_instr_size(cpu_dyncom);
			if(fill_tlb(core) == 0)
				return 0;
		}
		
		if (core->syscallSig) {
			return 1;
		}
		if (core->abortSig) {
			return 1;
		}
		if (!core->NirqSig) {
			if (!(core->Cpsr & 0x80)) {
				return 1;
			}
		}
			
		/* if regular trap */
		core->Reg[15] += get_instr_size(cpu);
		/*uint32_t mode = core->Cpsr & 0x1f;
		if ((mode != core->Mode) && (!is_user_mode(cpu))) {
			switch_mode(core, mode);
			return 1;
		}*/
		/* handle float point instruction */
		handle_fp_insn(core);
		return 1;
	}
	default: 
	{
		fprintf(stderr, "unknown return code: %d\n", rc);
		skyeye_exit(-1);
	}
	}//switch
	return 1;
}

static int handle_fp_insn(arm_core_t* core){
	uint32 instr = 0xdeadc0de;
	bus_read(32, core->phys_pc, &instr);
	if((instr & 0x0FB80e50)== 0x0eb80a40){ /* some instruction need to implemented here */
		/* VCVTBFI */
		printf("\n\nVCVTBFI executed:\n");
		extern int vcvtbfi_instr_impl(arm_core_t* cpu, uint32 instr);
		vcvtbfi_instr_impl(core, instr);
	}
	if((instr & 0x0FBF0ed0)== 0x0eb70ac0){ /* some instruction need to implemented here */
		/* VCVTBDS */
		printf("\n\nVCVTBDS executed:\n");
		extern int vcvtbfi_instr_impl(arm_core_t* cpu, uint32 instr);
		vcvtbfi_instr_impl(core, instr);
	}
	else if((instr & 0x0fb00e50) == 0x0e300a00){ /* some instruction need to implemented here */
		/* VADD */
		printf("VADD executed:\n");
		extern int vcvtbfi_instr_impl(arm_core_t* cpu, uint32 instr);
		vcvtbfi_instr_impl(core, instr);
	}	
	else if((instr & 0x0FBF0E50)== 0x0eb40a40){ /* some instruction need to implemented here */
		/* VCMP */
		printf("VCMP executed:\n");
		extern int vcvtbfi_instr_impl(arm_core_t* cpu, uint32 instr);
		vcvtbfi_instr_impl(core, instr);
	}	
	else if((instr & 0x0FBF0E50)== 0x0eb50a40){ /* some instruction need to implemented here */
		/* VCMP2 */
		printf("VCMP2 executed:\n");
		extern int vcvtbfi_instr_impl(arm_core_t* cpu, uint32 instr);
		vcvtbfi_instr_impl(core, instr);
	}	
	else if((instr & 0xFF800F10)== 0xF3000800){ /* some instruction need to implemented here */
		/* VSUB */
		printf("\n\nVSUB executed:\n\n");
		extern int vcvtbfi_instr_impl(arm_core_t* cpu, uint32 instr);
		vcvtbfi_instr_impl(core, instr);
	}	
	else if((instr & 0x0FB00E50)== 0x0E300a40){ /* some instruction need to implemented here */
		/* VSUB */
		printf("\n\nVSUB(floating-point) executed at 0x%x:\n\n", core->Reg[15]);
		extern int vcvtbfi_instr_impl(arm_core_t* cpu, uint32 instr);
		vcvtbfi_instr_impl(core, instr);
	}	
	else if((instr & 0x0Fb00e50)== 0x0E200a00){ /* some instruction need to implemented here */
		/* VFMUL */
		printf("\n\nVMUL executed:\n\n");
		extern int vcvtbfi_instr_impl(arm_core_t* cpu, uint32 instr);
		vcvtbfi_instr_impl(core, instr);
	}	
	else if((instr & 0x0Fb00e50)== 0x0E800a00){ /* some instruction need to implemented here */
		/* VFMUL */
		printf("\n\nVDIV executed:\n\n");
		extern int vcvtbfi_instr_impl(arm_core_t* cpu, uint32 instr);
		vcvtbfi_instr_impl(core, instr);
	}	
	else if((instr & 0x0Fb00e10)== 0x0E000a00){ /* some instruction need to implemented here */
		/* VMLA */
		printf("\n\nVMLA executed:\n\n");
		extern int vcvtbfi_instr_impl(arm_core_t* cpu, uint32 instr);
		vcvtbfi_instr_impl(core, instr);
	}
	else if((instr & 0x0FbF0ed0)== 0x0Eb00ac0){ /* some instruction need to implemented here */
		/* VABS */
		printf("\n\nVABS executed:\n\n");
		extern int vcvtbfi_instr_impl(arm_core_t* cpu, uint32 instr);
		vcvtbfi_instr_impl(core, instr);
	}	
	else if((instr & 0x0Fb00e10)== 0x0E100a00){ /* some instruction need to implemented here */
		/* VNMLA */
		printf("\n\nVNMLA executed:\n\n");
		extern int vcvtbfi_instr_impl(arm_core_t* cpu, uint32 instr);
		vcvtbfi_instr_impl(core, instr);
	}	
	else if((instr & 0x0Fb00e50)== 0x0E200a40){ /* some instruction need to implemented here */
		/* VMLS */
		printf("\n\nVNMLA executed:\n\n");
		extern int vcvtbfi_instr_impl(arm_core_t* cpu, uint32 instr);
		vcvtbfi_instr_impl(core, instr);
	}	
	else{
		//printf("Unknown instruction 0x%x\n", instr);
		return 0;
	}
	return 1;
}

#define PROF_FUNC_SIZE 0
/* This function handles tagging */
static inline void push_compiled_work(cpu_t* cpu, uint32_t pc, uint8_t func_attr){
	//printf("In %s, pc=0x%x\n", __FUNCTION__, pc);
	cpu->dyncom_engine->func_attr[cpu->dyncom_engine->functions] = func_attr;
	cpu->dyncom_engine->func_size[cpu->dyncom_engine->functions] = 0;
#if CHECK_IN_WRITE
	inc_jit_num(pc);
#else
	protect_code_page(pc);
#endif
	cpu_tag(cpu, pc);
#if PROF_FUNC_SIZE
	printf("-------------------------------------\n");
	printf("current func_size[%d] = %d, pc=0x%x, %s, %s\n", cpu->dyncom_engine->functions, cpu->dyncom_engine->func_size[cpu->dyncom_engine->functions], pc, is_usermode_func(cpu)?"usermode":"kernelmode", is_thumb_func(cpu)?"thumb":"arm");
	/* if the size of a JIT function is below 10, we print it out to see what is wrong. */
	if(cpu->dyncom_engine->func_size[cpu->dyncom_engine->functions] < 10){
		uint32_t instr = 0xdeadc0de;
		addr_t addr = pc;
		for(int i = 0; i < cpu->dyncom_engine->func_size[cpu->dyncom_engine->functions]; i++)
			if(bus_read(32, addr + i * 4, &instr) == 0)
				printf("0x%x:\t0x%x\n", addr + i * 4, instr);
		printf("-------------------------------------\n");
		
	}
#endif
	cpu->dyncom_engine->cur_tagging_pos ++;
	cpu_translate(cpu, pc);
	return;
}
/* Compiled the target to the host address , and try to free some unused translation block */
static void* compiled_worker(void* argp){
	cpu_t* cpu = (cpu_t*) argp;
	//uint32_t pos_to_translate;
	for(;;){
		while(1){
			uint32_t compiled_addr = 0xFFFFFFFF;
			uint8_t func_attr;
			pthread_rwlock_wrlock(&compile_stack_rwlock);
			if (!compile_stack.empty()) {
				compiled_addr = compile_stack.top();
				compile_stack.pop();

				func_attr = func_attr_stack.top();
				func_attr_stack.pop();
			}
			else{
				pthread_rwlock_unlock(&compile_stack_rwlock);
				/* All the code is translated, we need to out for a rest */
				break;
			}
			pthread_rwlock_unlock(&compile_stack_rwlock);
			/* begin translation */
			pthread_rwlock_wrlock(&translation_rwlock);
			fast_map hash_map = cpu->dyncom_engine->fmap;
			void* pfunc;

			PFUNC(compiled_addr);
			if(pfunc == NULL){
				push_compiled_work(cpu, compiled_addr, func_attr);
			}
			pthread_rwlock_unlock(&translation_rwlock);
		}
		//usleep(2);
	}
	return NULL;
}

void push_to_compiled(cpu_t* cpu, addr_t addr){
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	//cpu->user_mode = USER_MODE(core);
	/* we need TFlag to judge the thumb or arm, during translation time */
	core->Cpsr = (core->Cpsr & 0xffffffdf) | (core->TFlag << 5);
	//cpu->TFlag = core->TFlag;
	//assert((addr & 0x1) == 0);
	uint8_t func_attr = FUNC_ATTR_NONE;
	if(core->TFlag)
		func_attr |= FUNC_ATTR_THUMB;
	if(core->Reg[15] < 0xc0000000)
		func_attr |= FUNC_ATTR_USERMODE;
        #if MULTI_THREAD
        int ret;
        if((ret = pthread_rwlock_trywrlock(&compile_stack_rwlock)) == 0){
                compile_stack.push(addr);
		func_attr_stack.push (func_attr);
                pthread_rwlock_unlock(&compile_stack_rwlock);
        }
        else{
                printf("Warning ,can not get the wrlock ,error is %d, %s\n", ret, strerror(ret));
        }
        #else
        push_compiled_work(cpu, addr, func_attr);
        #endif
}
