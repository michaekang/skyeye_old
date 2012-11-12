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
* @file arm_dyncom_run.cpp
* @brief The dyncom run implementation for arm
* @author Michael.Kang blackfin.kang@gmail.com
* @version 78.77
* @date 2011-11-20
*/

#include <llvm/LLVMContext.h>
#include <llvm/Type.h>
#include <llvm/Function.h>
#include <llvm/Module.h>
#include <llvm/Constant.h>
#include <llvm/Constants.h>
#include "llvm/ExecutionEngine/JIT.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Instructions.h"

#include <skyeye_dyncom.h>
#include <skyeye_types.h>
#include <skyeye_obj.h>
#include <skyeye_sched.h>
#include <skyeye.h>
#include <bank_defs.h>
#include <skyeye_pref.h>
#include <skyeye_symbol.h>
#include <dyncom/dyncom_llvm.h>
#include <skyeye_log.h>
#include <dyncom/tag.h>

#include <vector>

#include "arm_regformat.h"
#include <skyeye_ram.h>

#include "armdefs.h"
#include "memory.h"
#include "dyncom/memory.h"
#include "dyncom/frontend.h"
#include "arm_dyncom_translate.h"
#include "arm_dyncom_parallel.h"
#include "dyncom/tlb.h"
#include "dyncom/defines.h"
#include "common/mmu/arm1176jzf_s_mmu.h"
#include "armmmu.h"
#include "arm_dyncom_dec.h"
#include "arm_dyncom_mmu.h"

#define LOG_IN_CLR	skyeye_printf_in_color

void arm_switch_mode(cpu_t *cpu);
//#define MAX_REGNUM 16
extern const char* arm_regstr[MAX_REG_NUM];

enum{
	ARM_DYNCOM_CALLOUT_UNDEF = MAX_DYNCOM_CALLOUT,
	ARM_DYNCOM_CALLOUT_INV_MVA,
	ARM_DYNCOM_CALLOUT_INV_ASID,
	ARM_DYNCOM_CALLOUT_INV_ALL,
	ARM_DYNCOM_CALLOUT_DEBUG_PRINT,
	ARM_DYNCOM_CALLOUT_EXIT,
	ARM_DYNCOM_MAX_CALLOUT
};

extern "C" {
extern void io_do_cycle (void * state);
}

uint32_t get_end_of_page(uint32 phys_addr){
	const uint32 page_size = 4 * 1024;
	return (phys_addr + page_size) & (~(page_size - 1));
}

void cpu_set_flags_codegen(cpu_t *cpu, uint32_t f)
{
        cpu->dyncom_engine->flags_codegen = f;
}


static int flush_current_page(cpu_t *cpu);
static int last_idx = 0;

#ifdef TIMER_PROFILE
static cpu_t* gcpu;
#include <signal.h>
#include "skyeye_thread.h"
void printinfo(int signum)
{
	cpu_print_statistics(gcpu);
}
#endif

static cpu_flags_layout_t arm_flags_layout[5] ={{NULL, 4, 'T', "TFLAG"}, {NULL, 3,'N',"NFLAG"},{NULL, 2,'Z',"ZFLAG"},{NULL, 1,'C',"CFLAG"},{NULL, 0,'V',"VFLAG"}} ;
/* physical register for arm archtecture */
static void arch_arm_init(cpu_t *cpu, cpu_archinfo_t *info, cpu_archrf_t *rf)
{
	arm_opc_func_init();
	// Basic Information
	info->name = "arm"; info->full_name = "arm_dyncom";

	// This architecture is biendian, accept whatever the
	// client wants, override other flags.
	info->common_flags &= CPU_FLAG_ENDIAN_MASK;
	/* set the flag of save pc */
	cpu->info.common_flags |= CPU_FLAG_SAVE_PC;

	info->delay_slots = 0;
	// The byte size is 8bits.
	// The word size is 32bits.
	// The float size is 64bits.
	// The address size is 32bits.
	info->byte_size = 8;
	info->word_size = 32;
	info->float_size = 32;
	info->address_size = 32;
	// There are 16 32-bit GPRs
	info->register_count[CPU_REG_GPR] = 19;
	info->register_size[CPU_REG_GPR] = info->word_size;
	// There is also 1 extra register to handle PSR.
	//info->register_count[CPU_REG_XR] = PPC_XR_SIZE;
	info->register_count[CPU_REG_XR] = MAX_REG_NUM - 19;
	//info->register_count[CPU_REG_XR] = 0;
	info->register_size[CPU_REG_XR] = 32;
	//info->register_count[CPU_REG_SPR] = MAX_REG_NUM - PHYS_PC;
	info->register_count[CPU_REG_SPR] = VFP_REG_NUM;
	info->register_size[CPU_REG_SPR] = 32;

	info->register_count[CPU_REG_FPR] = 64;
	info->register_size[CPU_REG_FPR] = 32;

	info->psr_size = 32;
	/* The flag count */
	info->flags_count = sizeof(arm_flags_layout)/sizeof(cpu_flags_layout_t);
	info->flags_layout = arm_flags_layout;
	/* Indicate the pc index for OPT_LOCAL_REGISTERS */
	info->pc_index_in_gpr = 15;

	cpu->redirection = false;
	
#ifdef TIMER_PROFILE
	gcpu = cpu;
	signal(SIGUSR1, printinfo);
	extern void *clock_thread(void*);
	pthread_t thread;
	int ret = pthread_create(&thread, NULL, clock_thread, NULL);
#endif

	//debug
	cpu_set_flags_debug(cpu, 0
	//	| CPU_DEBUG_PRINT_IR
	//	| CPU_DEBUG_LOG
	//	| CPU_DEBUG_PROFILE
               );
        cpu_set_flags_codegen(cpu, CPU_CODEGEN_TAG_LIMIT 
				| CPU_CODEGEN_OPTIMIZE
	//			| CPU_CODEGEN_VERIFY
			      );
	/* Initilize different register set for different core */

//	set_memory_operator(arch_arm_read_memory, arch_arm_write_memory);
//	arm_dyncom_mcr_init(cpu);
}

static void
arch_arm_done(cpu_t *cpu)
{
	//free(cpu->rf.grf);
}

static addr_t
arch_arm_get_pc(cpu_t *, void *reg)
{
	unsigned int *grf =(unsigned int *) reg;
	return grf[15];
}

static uint64_t
arch_arm_get_psr(cpu_t *, void *reg)
{
	return 0;
}

static int
arch_arm_get_reg(cpu_t *cpu, void *reg, unsigned reg_no, uint64_t *value)
{
	return (0);
}

static int arch_arm_disasm_instr(cpu_t *cpu, addr_t pc, char* line, unsigned int max_line){
	return 0;
}
static int arch_arm_translate_loop_helper(cpu_t *cpu, addr_t pc, BasicBlock *bb_ret, BasicBlock *bb_next, BasicBlock *bb, BasicBlock *bb_zol_cond){
	return 0;
}

static void arch_arm_emit_decode_reg(cpu_t *cpu, BasicBlock *bb)
{
	Value *nzcv = LSHR(AND(LOAD(cpu->ptr_gpr[16]), CONST(0xf0000000)), CONST(28));
	Value *n = TRUNC1(AND(LSHR(nzcv, CONST(3)), CONST(1)));
	Value *z = TRUNC1(AND(LSHR(nzcv, CONST(2)), CONST(1)));
	Value *c = TRUNC1(AND(LSHR(nzcv, CONST(1)), CONST(1)));
	Value *v = TRUNC1(AND(LSHR(nzcv, CONST(0)), CONST(1)));
	Value *t = TRUNC1(LSHR(AND(LOAD(cpu->ptr_gpr[16]), CONST(1 << THUMB_BIT)), CONST(THUMB_BIT)));
	Value *mode = AND(LOAD(cpu->ptr_gpr[16]),CONST(0x1f));
	new StoreInst(n, ptr_N, false, bb);
	new StoreInst(z, ptr_Z, false, bb);
	new StoreInst(c, ptr_C, false, bb);
	new StoreInst(v, ptr_V, false, bb);
	new StoreInst(t, ptr_T, false, bb);
	LET(MODE_REG,mode);
}

static void arch_arm_spill_reg_state(cpu_t *cpu, BasicBlock *bb)
{
		/* Save N Z C V T */
	Value *z = SHL(ZEXT32(LOAD(ptr_Z)), CONST(30));
	Value *n = SHL(ZEXT32(LOAD(ptr_N)), CONST(31));
	Value *c = SHL(ZEXT32(LOAD(ptr_C)), CONST(29));
	Value *v = SHL(ZEXT32(LOAD(ptr_V)), CONST(28));
	Value *t = SHL(ZEXT32(LOAD(ptr_T)), CONST(THUMB_BIT));
	Value *nzcv = OR(OR(OR(z, n), c), v);
	Value *mode = R(MODE_REG);
	Value *cpsr = OR(AND(LOAD(cpu->ptr_gpr[16]), CONST(0x0fffffe0)), nzcv);
	cpsr = OR(cpsr,mode);
	/* restore the T bit for arm */
	/* restore the T bit for arm */

	cpsr = OR(AND(cpsr, CONST(~(1 <<THUMB_BIT))), t);
	new StoreInst(cpsr, cpu->ptr_gpr[16], false, bb);
}

/**
* @brief Return the instruction length
*
* @param cpu
*
* @return 
*/
static uint32 arch_arm_get_instr_length(cpu_t *cpu){
	return INSTR_SIZE;
}

static arch_func_t arm_arch_func = {
	arch_arm_init,
	arch_arm_done,
	arch_arm_get_pc,
	arch_arm_get_instr_length,
	arch_arm_emit_decode_reg,
	arch_arm_spill_reg_state,
	arch_arm_tag_instr,
	arch_arm_disasm_instr,
	arch_arm_translate_cond,
	arch_arm_translate_instr,
        arch_arm_translate_loop_helper,
	// idbg support
	arch_arm_get_psr,
	arch_arm_get_reg,
	NULL /* get_fp_reg */
};

static uint32_t arm_debug_func(cpu_t* cpu){
	int idx = 0;
	arm_core_t* core = (arm_core_t*)get_cast_conf_obj(cpu->cpu_data, "arm_core_t");
	core->icounter++;
        extern int diff_single_step(cpu_t *cpu);
        return diff_single_step(cpu);
}

extern "C" unsigned arm_dyncom_SWI (ARMul_State * state, ARMword number);
extern "C" void arm_dyncom_Abort(ARMul_State * state, ARMword vector);

static void arm_dyncom_syscall(cpu_t* cpu, uint32_t num){

	//arm_core_t* core = (arm_core_t*)get_cast_conf_obj(cpu->cpu_data, "arm_core_t");
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	sky_pref_t* pref = get_skyeye_pref();
	//printf("in %s user_mode_sim %d", __FUNCTION__, pref->user_mode_sim);
	if(pref->user_mode_sim)
		arm_dyncom_SWI(core, num);
	else
		//ARMul_Abort(core,ARMul_SWIV);
		core->syscallSig = 1;
}

/* Undefined instruction handler, set necessary flags */
void 
arm_undef_instr(cpu_t *cpu){
	arm_core_t* core = (arm_core_t*)get_cast_conf_obj(cpu->cpu_data, "arm_core_t");
	printf("\t\tLet us set a flag, signaling an undefined instruction!\n");
	core->Aborted = ARMul_UndefinedInstrV;
	core->abortSig = HIGH;
}
typedef llvm::ArrayRef<llvm::Type*> TypeArray;
typedef llvm::ArrayRef<llvm::Value*> ValueArray;

/**
 * @brief Generate the invoke undef instr exception llvm IR
 *
 * @param cpu CPU core structure
 * @param bb basic block to store llvm IR 
 * @param instr undefined instruction (unused)
 */
void
arch_arm_undef(cpu_t *cpu, BasicBlock *bb, uint32_t instr)
{
	if (cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_UNDEF] == NULL) {
		printf("in %s Could not find callout\n", __FUNCTION__);
		return;
	}
	IntegerType *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
	Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
	std::vector<Value *> params;
	params.push_back(v_cpu_ptr);
	/* When using a custom callout, must put the callout index as argument for dyncom_callout */
	params.push_back(CONST(ARM_DYNCOM_CALLOUT_UNDEF));
	//params.push_back(CONST(instr)); // no need for now, the callout func takes no argument
	CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_UNDEF], ValueArray(params), "", bb);
}

/* Undefined instruction handler initialization. Should be called once at init */
static void 
arch_arm_undef_init(cpu_t *cpu){
	//types
	std::vector<llvm::Type*> type_func_undef_args;
	PointerType *type_intptr = PointerType::get(cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX()), 0);
	IntegerType *type_i32 = IntegerType::get(_CTX(), 32);
	type_func_undef_args.push_back(type_intptr);	/* intptr *cpu */
	type_func_undef_args.push_back(type_i32);	/* unsinged int */
	FunctionType *type_func_undef_callout = FunctionType::get(
		Type::getInt32Ty(cpu->dyncom_engine->mod->getContext()),	//return
		TypeArray(type_func_undef_args),	/* Params */
		false);		      	/* isVarArg */
	/* For a custom callout, the dyncom_calloutX functions should be used */
	Constant *undef_const = cpu->dyncom_engine->mod->getOrInsertFunction("dyncom_callout",	//function name
		type_func_undef_callout);	//return
	if(undef_const == NULL)
		fprintf(stderr, "Error:cannot insert function:undefined_instr_callout.\n");
	Function *undef_func = cast<Function>(undef_const);
	undef_func->setCallingConv(CallingConv::C);
	cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_UNDEF] = undef_func;
	cpu->dyncom_engine->arch_func[ARM_DYNCOM_CALLOUT_UNDEF] = (void*)arm_undef_instr;
}

void
arch_arm_invalidate_by_mva(cpu_t *cpu, BasicBlock *bb, Value* mva, tlb_type_t access_type)
{
	if (cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_MVA] == NULL) {
		printf("in %s Could not find callout\n", __FUNCTION__);
		return;
	}
	//Type const *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	IntegerType *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
	Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
	std::vector<Value *> params;
	params.push_back(v_cpu_ptr);
	params.push_back(CONST(ARM_DYNCOM_CALLOUT_INV_MVA));
	params.push_back(mva);
	params.push_back(CONST(access_type));
	/* When using a custom callout, must put the callout index as argument for dyncom_callout */
	//params.push_back(CONST(instr)); // no need for now, the callout func takes no argument
	CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_MVA], ValueArray(params), "", bb);
}

static void 
arch_arm_invalidate_by_mva_init(cpu_t *cpu){
	//types
	std::vector<llvm::Type*> type_func_args;
	PointerType *type_intptr = PointerType::get(cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX()), 0);
	IntegerType *type_i32 = IntegerType::get(_CTX(), 32);
	type_func_args.push_back(type_intptr);	/* intptr *cpu */
	type_func_args.push_back(type_i32);	/* unsinged int */
	type_func_args.push_back(type_i32);	/* mva */
	type_func_args.push_back(type_i32);	/* access type */
	FunctionType *type_func_callout = FunctionType::get(
		Type::getInt32Ty(cpu->dyncom_engine->mod->getContext()),	//return
		TypeArray(type_func_args),	/* Params */
		false);		      	/* isVarArg */
	/* For a custom callout, the dyncom_calloutX functions should be used */
	Constant *func_const = cpu->dyncom_engine->mod->getOrInsertFunction("dyncom_callout2",	//function name
		type_func_callout);	//return
	if(func_const == NULL)
		fprintf(stderr, "Error:cannot insert function:undefined_instr_callout.\n");
	Function *func = cast<Function>(func_const);
	func->setCallingConv(CallingConv::C);
	cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_MVA] = func;
	cpu->dyncom_engine->arch_func[ARM_DYNCOM_CALLOUT_INV_MVA] = (void*)erase_by_mva;
}

void
arch_arm_invalidate_by_asid(cpu_t *cpu, BasicBlock *bb, Value* asid, tlb_type_t access_type)
{
	if (cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_ASID] == NULL) {
		printf("in %s Could not find callout\n", __FUNCTION__);
		return;
	}
	//Type const *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	IntegerType *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
	Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
	std::vector<Value *> params;
	params.push_back(v_cpu_ptr);
	/* When using a custom callout, must put the callout index as argument for dyncom_callout */
	params.push_back(CONST(ARM_DYNCOM_CALLOUT_INV_ASID));
	params.push_back(asid);
	params.push_back(CONST(access_type));
	//params.push_back(CONST(instr)); // no need for now, the callout func takes no argument
	CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_ASID], ValueArray(params), "", bb);
}
static void 
arch_arm_invalidate_by_asid_init(cpu_t *cpu){
	//types
	std::vector<llvm::Type*> type_func_args;
	PointerType *type_intptr = PointerType::get(cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX()), 0);
	IntegerType *type_i32 = IntegerType::get(_CTX(), 32);
	type_func_args.push_back(type_intptr);	/* intptr *cpu */
	type_func_args.push_back(type_i32);	/* unsinged int */
	type_func_args.push_back(type_i32);	/* asid */
	type_func_args.push_back(type_i32);	/* access type */
	FunctionType *type_func_callout = FunctionType::get(
		Type::getInt32Ty(cpu->dyncom_engine->mod->getContext()),	//return
		TypeArray(type_func_args),	/* Params */
		false);		      	/* isVarArg */
	/* For a custom callout, the dyncom_calloutX functions should be used */
	Constant *func_const = cpu->dyncom_engine->mod->getOrInsertFunction("dyncom_callout2",	//function name
		type_func_callout);	//return
	if(func_const == NULL)
		fprintf(stderr, "Error:cannot insert function:undefined_instr_callout.\n");
	Function *func = cast<Function>(func_const);
	func->setCallingConv(CallingConv::C);
	cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_ASID] = func;
	cpu->dyncom_engine->arch_func[ARM_DYNCOM_CALLOUT_INV_ASID] = (void*)erase_by_asid;
}

void
arch_arm_invalidate_by_all(cpu_t *cpu, BasicBlock *bb, tlb_type_t access_type)
{
	if (cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_ALL] == NULL) {
		printf("in %s Could not find callout\n", __FUNCTION__);
		return;
	}
	//Type const *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	IntegerType *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
	Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
	std::vector<Value *> params;
	params.push_back(v_cpu_ptr);
	/* When using a custom callout, must put the callout index as argument for dyncom_callout */
	params.push_back(CONST(ARM_DYNCOM_CALLOUT_INV_ALL));
	params.push_back(CONST(access_type));
	//params.push_back(CONST(instr)); // no need for now, the callout func takes no argument
	CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_ALL], ValueArray(params), "", bb);
}

void debug_print(cpu_t* cpu, unsigned long arg0, int arg1, int arg2){
#if 1
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	//if(core->Reg[15] == 0xc00081e4){
	if(arg2 == 0)
		printf("\n---------------\n");
	int context_id = (*(uint32_t *)(cpu->rf.context_id)) & 0xFF;
	printf("In %s, arg0=0x%llx, arg1=0x%llx, arg2=0x%llx, context_id=%d\n", __FUNCTION__, arg0, arg1, arg2, context_id);
	if(arg2 == 0xd)
		printf("------------------\n");
	//}
#endif
	return;
}
void
arch_arm_debug_print(cpu_t *cpu, BasicBlock *bb, Value* arg0, Value* arg1, Value* arg2)
{
	if (cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_DEBUG_PRINT] == NULL) {
		printf("in %s Could not find callout\n", __FUNCTION__);
		return;
	}
	//Type const *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	IntegerType *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
	Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
	std::vector<Value *> params;
	params.push_back(v_cpu_ptr);
	/* When using a custom callout, must put the callout index as argument for dyncom_callout */
	params.push_back(CONST(ARM_DYNCOM_CALLOUT_DEBUG_PRINT));
	params.push_back(arg0);
	params.push_back(arg1);
	params.push_back(arg2);
	//params.push_back(CONST(instr)); // no need for now, the callout func takes no argument
	CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_DEBUG_PRINT], ValueArray(params), "", bb);
}

static void 
arch_arm_debug_print_init(cpu_t *cpu){
	//types
	std::vector<llvm::Type*> type_func_args;
	PointerType *type_intptr = PointerType::get(cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX()), 0);
	IntegerType *type_i32 = IntegerType::get(_CTX(), 32);
	IntegerType *type_i64 = IntegerType::get(_CTX(), 64);
	type_func_args.push_back(type_intptr);	/* intptr *cpu */
	type_func_args.push_back(type_i32);	/* unsinged int */
	type_func_args.push_back(type_i64);	/* arg0 unsigned int */
	type_func_args.push_back(type_i32);	/* arg1  */
	type_func_args.push_back(type_i32);	/* arg2  */
	FunctionType *type_func_callout = FunctionType::get(
		Type::getInt32Ty(cpu->dyncom_engine->mod->getContext()),	//return
		TypeArray(type_func_args),	/* Params */
		false);		      	/* isVarArg */
	/* For a custom callout, the dyncom_calloutX functions should be used */
	Constant *func_const = cpu->dyncom_engine->mod->getOrInsertFunction("dyncom_callout3",	//function name
		type_func_callout);	//return
	if(func_const == NULL)
		fprintf(stderr, "Error:cannot insert function:undefined_instr_callout.\n");
	Function *func = cast<Function>(func_const);
	func->setCallingConv(CallingConv::C);
	cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_DEBUG_PRINT] = func;
	cpu->dyncom_engine->arch_func[ARM_DYNCOM_CALLOUT_DEBUG_PRINT] = (void*)debug_print;
}
static void go_exit(cpu_t *cpu,uint32_t num){
	printf("skyeye exit,exit_code:0x%x",num);
	skyeye_exit(num);
}
void arch_arm_exit(cpu_t *cpu, BasicBlock *bb, Value* arg0){
	if (cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_EXIT] == NULL) {
		printf("in %s Could not find callout\n", __FUNCTION__);
		return;
	}
	//Type const *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	IntegerType *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
	Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
	std::vector<Value *> params;
	params.push_back(v_cpu_ptr);
	/* When using a custom callout, must put the callout index as argument for dyncom_callout */
	params.push_back(CONST(ARM_DYNCOM_CALLOUT_EXIT));
	params.push_back(arg0);
	//params.push_back(CONST(instr)); // no need for now, the callout func takes no argument
	CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_EXIT], ValueArray(params), "", bb);
	BranchInst::Create(cpu->dyncom_engine->bb_trap,bb);
	//generate exit code
}
static void arch_arm_exit_init(cpu_t *cpu){
	std::vector<llvm::Type*> type_func_args;
	PointerType *type_intptr = PointerType::get(cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX()), 0);
	IntegerType *type_i32 = IntegerType::get(_CTX(), 32);
	type_func_args.push_back(type_intptr);	/* intptr *cpu */
	type_func_args.push_back(type_i32);	/* unsinged int */
	type_func_args.push_back(type_i32);	/* unsinged int */
	FunctionType *type_func_callout = FunctionType::get(
		Type::getInt32Ty(cpu->dyncom_engine->mod->getContext()),	//return
		TypeArray(type_func_args),	/* Params */
		false);		      	/* isVarArg */
	/* For a custom callout, the dyncom_calloutX functions should be used */
	Constant *func_const = cpu->dyncom_engine->mod->getOrInsertFunction("dyncom_callout1",	//function name
		type_func_callout);	//return
	if(func_const == NULL)
		fprintf(stderr, "Error:cannot insert function:undefined_instr_callout.\n");
	Function *func = cast<Function>(func_const);
	func->setCallingConv(CallingConv::C);
	cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_EXIT] = func;
	cpu->dyncom_engine->arch_func[ARM_DYNCOM_CALLOUT_EXIT] = (void*)go_exit;
}
static BasicBlock* create_mmu_fault_bb(cpu_t* cpu, Value* result, int fault, Value* fault_addr){
	BasicBlock *bb = BasicBlock::Create(_CTX(), "mmu_fault", cpu->dyncom_engine->cur_func, 0);		
	LET(CP15_TLB_FAULT_STATUS, SELECT(result, CONST(fault), R(CP15_TLB_FAULT_STATUS)));
	LET(CP15_TLB_FAULT_ADDR, SELECT(result, fault_addr, R(CP15_TLB_FAULT_ADDR)));
	BranchInst::Create(cpu->dyncom_engine->bb_trap, bb);
	return bb;
}
#if 0
Value *
get_phys_addr(cpu_t *cpu, BasicBlock *bb, Value* addr, int read)
{
	if(is_user_mode(cpu)){
		cpu->dyncom_engine->bb = bb;
		return addr;
	}
	BasicBlock* exit_bb = cpu->dyncom_engine->bb_trap;
	Value* result = CONST1(0);
	int fault = read? TLB_READ_MISS:TLB_WRITE_MISS;
	Value* fault_addr = CONST(0xdeadc0de);

	/* va = (addr & 0xfffff000) | (CP15REG(CP15_CONTEXT_ID) & 0xff)*/
	Value* va =  AND(addr, CONST(0xFFFFF000));
	/* get index , index = tlb_cache[access_type][va & 0xff][(va >> 12) % TLB_SIZE]; */
	Value* a = ADD(cpu->dyncom_engine->ptr_TLB, ZEXT64(MUL(UREM(LSHR(addr, CONST(12)), CONST(TLB_SIZE)), CONST(sizeof(uint64_t)))));
	//arch_arm_debug_print(cpu, bb, CONST64(cpu->dyncom_engine->TLB), R(15), CONST(10));
	//arch_arm_debug_print(cpu, bb, a, R(15), CONST(11));
	a = new IntToPtrInst(a, PointerType::get(XgetType(Int64Ty), 0), "", bb);
	Value* tlb_entry = new LoadInst(a, "", false, bb);
	result = OR(result, ICMP_NE(TRUNC32(LSHR(AND(tlb_entry, CONST64(0xFFFFFFFF00000000)), CONST64(32))), va));
#if 1
		//fault_status = SELECT(result, CONST(1), CONST(0));

		/*  
		 *       AP        Priv Permissions       User Permissions
		 *	--------------------------------------------------
		 *       00        No access              No Access
		 *       01        Read/Write             No Access
		 *       10        Read/Write             Read only
		 *       11        Read/Write             Read/Write
		 */
	Value* user_mode;
	//if(cpu->user_mode){
	//if(is_usermode_func(cpu)){
		//user_mode = CONST1(1);
	user_mode = TRUNC1(cpu->dyncom_engine->ptr_user_mode);
	//else
	//	user_mode = CONST1(0);
	Value* ap = TRUNC32(AND(tlb_entry, CONST64(0x3)));
		/*
		 * if (result == 0)
		 *	if(ap == 0)
		 *		result = 1 ; permission fault happened
		 *	else
		 *		result = 0
		 * else
		 *	result = result 
		 */
	//result = SELECT(result, result, SELECT(ICMP_EQ(ap, CONST(0)), CONST1(1), result));
	result = SELECT(result, result, SELECT(AND(ICMP_EQ(ap, CONST(1)), ICMP_EQ(user_mode, CONST1(1))),
				CONST1(1), result));
	result = SELECT(result, result, SELECT(AND(AND(ICMP_EQ(ap, CONST(2)), ICMP_EQ(user_mode, CONST1(1))), ICMP_EQ(CONST(read), CONST(0))), CONST1(1), result));
	result = SELECT(ICMP_EQ(ap, CONST(3)), result, result);
	//}
#endif
		/*
		 * if(fault_addr == 0xdeadc0de && result)
		 *	fault_addr = addr;
		 */
	fault_addr = SELECT(result, SELECT(ICMP_EQ(fault_addr, CONST(0xdeadc0de)), addr, fault_addr), CONST(0xDEADC0DE));


	/* Get physical address */	
	Value* phys_page = SELECT(result, CONST(0xdead0000), TRUNC32(AND(tlb_entry, CONST64(0xFFFFF000))));
	Value* phys_addr = SELECT(result, CONST(0xdeadc0de), OR(phys_page, AND(CONST(0xFFF), addr)));

	Value *cond = ICMP_NE(result, CONST1(1));
	BasicBlock *load_store_bb = BasicBlock::Create(_CTX(), "load_store_begin", cpu->dyncom_engine->cur_func, 0);
	BasicBlock* mmu_fault_bb = create_mmu_fault_bb(cpu, result, fault, fault_addr);

	cpu->dyncom_engine->bb = load_store_bb;
	//arch_arm_debug_print(cpu, bb, ZEXT64(phys_addr), R(15), CONST(15));
	arch_branch(1, load_store_bb, mmu_fault_bb, cond, bb);
	bb = load_store_bb;
	cpu->dyncom_engine->io_flag = TRUNC32(AND(tlb_entry, CONST64(IO_FLAG_MASK)));
	return phys_addr;
}
#endif
static void 
arch_arm_invalidate_by_all_init(cpu_t *cpu){
	//types
	std::vector<llvm::Type*> type_func_args;
	PointerType *type_intptr = PointerType::get(cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX()), 0);
	IntegerType *type_i32 = IntegerType::get(_CTX(), 32);
	type_func_args.push_back(type_intptr);	/* intptr *cpu */
	type_func_args.push_back(type_i32);	/* unsinged int */
	type_func_args.push_back(type_i32);	/* access type */
	FunctionType *type_func_callout = FunctionType::get(
		Type::getInt32Ty(cpu->dyncom_engine->mod->getContext()),	//return
		TypeArray(type_func_args),	/* Params */
		false);		      	/* isVarArg */
	/* For a custom callout, the dyncom_calloutX functions should be used */
	Constant *func_const = cpu->dyncom_engine->mod->getOrInsertFunction("dyncom_callout1",	//function name
		type_func_callout);	//return
	if(func_const == NULL)
		fprintf(stderr, "Error:cannot insert function:undefined_instr_callout.\n");
	Function *func = cast<Function>(func_const);
	func->setCallingConv(CallingConv::C);
	cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_ALL] = func;
	cpu->dyncom_engine->arch_func[ARM_DYNCOM_CALLOUT_INV_ALL] = (void*)erase_all;
}


/**
* @brief Scan the bb usage and recycle some bb to free memory
*
* @param cpu
*/
const uint32_t passed_sec = 5;
const uint32_t MILL = 1000 * 1000;
const uint32_t period_in_usec = MILL * passed_sec;
static void print_statistics(void* priv_data){
	static uint64_t last_icounter = 0;
	cpu_t* cpu = (cpu_t*)priv_data;
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	uint64_t total_icounter = core->icounter + cpu->icounter;	
        skyeye_printf_in_color(BLUE, "\ncurrent translated funcions is %d, current sec=%d, MIPS=%f\n", cpu->dyncom_engine->functions, get_clock_sec(), (float)(total_icounter - last_icounter)/(MILL * passed_sec));
	printf(" total_icounter=%f Mill fast_interp icounter=%f(%f), dyncom icounter=%f(%f), fast_interp kernel_icouner=%lld(%f), \n", (float)total_icounter / MILL, (float)core->icounter/MILL, (float)core->icounter/total_icounter, (float)cpu->icounter / MILL, (float)cpu->icounter/total_icounter, core->kernel_icounter, (float)core->kernel_icounter/MILL);
        last_icounter = core->icounter + cpu->icounter;
}
void arm_dyncom_init(arm_core_t* core){
	cpu_t* cpu = cpu_new(0, 0, arm_arch_func);

	/* set user mode or not */
	sky_pref_t *pref = get_skyeye_pref();
	if(pref->user_mode_sim)
                cpu->is_user_mode = 1;
        else
                cpu->is_user_mode = 0;
	
	core->NirqSig = HIGH;
	cpu->dyncom_engine->code_entry = 0x80d0;
	if (!pref->user_mode_sim) {
		cpu->dyncom_engine->code_start = 0;
		cpu->dyncom_engine->code_end = 0xffffffff;
	} else {
		cpu->dyncom_engine->code_end = 0x100000;
		cpu->dyncom_engine->code_entry = 0x80d0;
		/* The user mode code section from android kernel */
		cpu->dyncom_engine->code1_start = 0xFFFF0000;
		cpu->dyncom_engine->code1_end = 0xFFFF0000 + 0x1000;

	}

	cpu->switch_mode = arm_switch_mode;
		
	cpu->mem_ops = arm_dyncom_mem_ops;
	//cpu->cpu_edata = (conf_object_t*)core;
	cpu->cpu_data = get_conf_obj_by_cast(core, "arm_core_t");
	
	/* init the reg structure */
	cpu->rf.pc = &core->Reg[15];
	/* Under user mode sim, both phys_pc and pc are pointed to Reg 15 */
	if(is_user_mode(cpu))
		cpu->rf.phys_pc = &core->Reg[15];
	else
		cpu->rf.phys_pc = &core->phys_pc;
	cpu->rf.context_id = &(core->CP15[CP15(CP15_CONTEXT_ID)]);
	cpu->rf.cpsr = &core->Cpsr;
	cpu->rf.grf = core->Reg;
	//cpu->rf.srf = core->Spsr;
	//cpu->rf.srf = &core->phys_pc;
	cpu->rf.srf = core->Reg_usr;
	//cpu->rf.srf = core->ExtReg;
	cpu->rf.frf = core->ExtReg;
       /* The flag address  */
	cpu->info.flags_layout[0].flag_address = &core->TFlag;
	cpu->info.flags_layout[1].flag_address = &core->NFlag;
	cpu->info.flags_layout[2].flag_address = &core->ZFlag;
	cpu->info.flags_layout[3].flag_address = &core->CFlag;
	cpu->info.flags_layout[4].flag_address = &core->VFlag;
	
	cpu->debug_func = arm_debug_func;
	
	if(pref->user_mode_sim){
		cpu->syscall_func = arm_dyncom_syscall;
	}
	else
//		cpu->syscall_func = NULL;
		cpu->syscall_func = arm_dyncom_syscall;
	core->dyncom_cpu = get_conf_obj_by_cast(cpu, "cpu_t");
	
	cpu->dyncom_engine->flags &= ~CPU_FLAG_SWAPMEM;
	cpu->dyncom_engine->need_exclusive = 0;
	cpu->dyncom_engine->wb_flag = 0;
#if FAST_MEMORY
	if (pref->user_mode_sim){
		cpu->dyncom_engine->RAM = (uint8_t*)get_dma_addr(0);
	}
	else{
		cpu->dyncom_engine->RAM = (uint8_t*)get_dma_addr(BANK0_START);
		//cpu->dyncom_engine->TLB = (unsigned long)new_tlb();
	}
#endif

	//core->CP15[CP15(CP15_MAIN_ID)] = 0x410FB760;
	core->CP15[CP15(CP15_MAIN_ID)] = 0x7b000;
	//core->CP15[CP15_MAIN_ID + 1] = 0x410FB760;
	//core->CP15[CP15_MAIN_ID - 1] = 0x410FB760;
	core->CP15[CP15(CP15_CONTROL)] = 0x00050078;
//	core->CP15[CP15(CP15_CONTROL)] = 0x00000078;
	core->CP15[CP15(CP15_CACHE_TYPE)] = 0xd172172;
	core->Cpsr = 0xd3;
	core->Mode = SVC32MODE;

//	load_symbol_from_sysmap();
	//new_tlb(TLB_SIZE, ASID_SIZE);
	/* undefined instr handler init */
	arch_arm_undef_init(cpu);
	arch_arm_invalidate_by_asid_init(cpu);
	arch_arm_invalidate_by_mva_init(cpu);
	arch_arm_invalidate_by_all_init(cpu);
	arch_arm_debug_print_init(cpu);
	arch_arm_exit_init(cpu);
        /* normal irq flag */
	IntegerType *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
        Constant *v_Nirq = ConstantInt::get(intptr_type, (uintptr_t)&(core->NirqSig));
        cpu->ptr_Nirq = ConstantExpr::getIntToPtr(v_Nirq, PointerType::getUnqual(getIntegerType(cpu->info.address_size)));
        cpu->ptr_Nirq->setName("Nirq");
	
	init_compiled_queue(cpu);
	//if(running_mode == HYBRID || running_mode == PURE_DYNCOM){
		int timer_id;
		//create_thread_scheduler(period_in_usec, Periodic_sched, print_statistics, (void *)cpu, &timer_id);
	//}
	return;
}

void switch_mode(arm_core_t *core, uint32_t mode)
{
	uint32_t tmp1, tmp2;
	if (core->Mode == mode) {
		//Mode not changed.
		//printf("mode not changed\n");
		return;
	}
	//printf("%d --->>> %d\n", core->Mode, mode);
	//printf("In %s, Cpsr=0x%x, R15=0x%x, last_pc=0x%x, cpsr=0x%x, spsr_copy=0x%x, icounter=%lld\n", __FUNCTION__, core->Cpsr, core->Reg[15], core->last_pc, core->Cpsr, core->Spsr_copy, core->icounter);
	if (mode != USERBANK) {
		switch (core->Mode) {
		case USER32MODE:
			core->Reg_usr[0] = core->Reg[13];
			core->Reg_usr[1] = core->Reg[14];
			break;
		case IRQ32MODE:
			core->Reg_irq[0] = core->Reg[13];
			core->Reg_irq[1] = core->Reg[14];
			core->Spsr[IRQBANK] = core->Spsr_copy;
			break;
		case SVC32MODE:
			core->Reg_svc[0] = core->Reg[13];
			core->Reg_svc[1] = core->Reg[14];
			core->Spsr[SVCBANK] = core->Spsr_copy;
			break;
		case ABORT32MODE:
			core->Reg_abort[0] = core->Reg[13];
			core->Reg_abort[1] = core->Reg[14];
			core->Spsr[ABORTBANK] = core->Spsr_copy;
			break;
		case UNDEF32MODE:
			core->Reg_undef[0] = core->Reg[13];
			core->Reg_undef[1] = core->Reg[14];
			core->Spsr[UNDEFBANK] = core->Spsr_copy;
			break;
		case FIQ32MODE:
			core->Reg_firq[0] = core->Reg[13];
			core->Reg_firq[1] = core->Reg[14];
			core->Spsr[FIQBANK] = core->Spsr_copy;
			break;

		}

		switch (mode) {
		case USER32MODE:
			core->Reg[13] = core->Reg_usr[0];
			core->Reg[14] = core->Reg_usr[1];
			core->Bank = USERBANK;
			break;
		case IRQ32MODE:
			core->Reg[13] = core->Reg_irq[0];
			core->Reg[14] = core->Reg_irq[1];
			core->Spsr_copy = core->Spsr[IRQBANK];
			core->Bank = IRQBANK;
			break;
		case SVC32MODE:
			core->Reg[13] = core->Reg_svc[0];
			core->Reg[14] = core->Reg_svc[1];
			core->Spsr_copy = core->Spsr[SVCBANK];
			core->Bank = SVCBANK;
			break;
		case ABORT32MODE:
			core->Reg[13] = core->Reg_abort[0];
			core->Reg[14] = core->Reg_abort[1];
			core->Spsr_copy = core->Spsr[ABORTBANK];
			core->Bank = ABORTBANK;
			break;
		case UNDEF32MODE:
			core->Reg[13] = core->Reg_undef[0];
			core->Reg[14] = core->Reg_undef[1];
			core->Spsr_copy = core->Spsr[UNDEFBANK];
			core->Bank = UNDEFBANK;
			break;
		case FIQ32MODE:
			core->Reg[13] = core->Reg_firq[0];
			core->Reg[14] = core->Reg_firq[1];
			core->Spsr_copy = core->Spsr[FIQBANK];
			core->Bank = FIQBANK;
			break;

		}
		core->Mode = mode;
		//printf("In %si end, Cpsr=0x%x, R15=0x%x, last_pc=0x%x, cpsr=0x%x, spsr_copy=0x%x, icounter=%lld\n", __FUNCTION__, core->Cpsr, core->Reg[15], core->last_pc, core->Cpsr, core->Spsr_copy, core->icounter);
		//printf("\n--------------------------------------\n");
	} else {
		printf("user mode\n");
		exit(-2);
	}
}

void switch_mode_IR(cpu_t *cpu,Value* mode,BasicBlock *bb_cur){
	BasicBlock* bb = bb_cur;
	Value* mode_diff = new ICmpInst(*bb_cur,ICmpInst::ICMP_NE,R(MODE_REG),mode,"");
	BasicBlock* bb_switch_mode = BasicBlock::Create(_CTX(),"switch_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_normal = BasicBlock::Create(_CTX(),"normal_not_switch_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_old_user32 = BasicBlock::Create(_CTX(),"old_user32_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_old_irq32 = BasicBlock::Create(_CTX(),"old_irq32_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_old_svc32 = BasicBlock::Create(_CTX(),"old_svc32_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_old_abort32 = BasicBlock::Create(_CTX(),"old_abort32_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_old_undef32 = BasicBlock::Create(_CTX(),"old_undef32_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_old_fiq32 = BasicBlock::Create(_CTX(),"old_fiq32_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_new_user32 = BasicBlock::Create(_CTX(),"new_user32_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_new_irq32 = BasicBlock::Create(_CTX(),"new_irq32_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_new_svc32 = BasicBlock::Create(_CTX(),"new_svc32_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_new_abort32 = BasicBlock::Create(_CTX(),"new_abort32_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_new_undef32 = BasicBlock::Create(_CTX(),"new_undef32_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_new_fiq32 = BasicBlock::Create(_CTX(),"new_fiq32_mode",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_label1 = BasicBlock::Create(_CTX(),"label1",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_label2 = BasicBlock::Create(_CTX(),"label2",cpu->dyncom_engine->cur_func,0);
	BasicBlock* bb_exit = BasicBlock::Create(_CTX(),"exit",cpu->dyncom_engine->cur_func,0);
	cpu->dyncom_engine->bb = bb_normal;
	BasicBlock* bb_switch_mode_real = BasicBlock::Create(_CTX(),"switch_mode_real",cpu->dyncom_engine->cur_func,0);
	bb = bb_switch_mode_real;
	Value* cpu_mode = R(MODE_REG);
	SwitchInst* sw_old = SwitchInst::Create(cpu_mode,bb_exit,6,bb_switch_mode_real);
	sw_old->addCase(CONST(USER32MODE),bb_old_user32);
	sw_old->addCase(CONST(IRQ32MODE),bb_old_irq32);
	sw_old->addCase(CONST(SVC32MODE),bb_old_svc32);
	sw_old->addCase(CONST(ABORT32MODE),bb_old_abort32);
	sw_old->addCase(CONST(UNDEF32MODE),bb_old_undef32);
	sw_old->addCase(CONST(FIQ32MODE),bb_old_fiq32);
	SwitchInst* sw_new = SwitchInst::Create(mode,bb_exit,6,bb_label1);
	sw_new->addCase(CONST(USER32MODE),bb_new_user32);
	sw_new->addCase(CONST(IRQ32MODE),bb_new_irq32);
	sw_new->addCase(CONST(SVC32MODE),bb_new_svc32);
	sw_new->addCase(CONST(ABORT32MODE),bb_new_abort32);
	sw_new->addCase(CONST(UNDEF32MODE),bb_new_undef32);
	sw_new->addCase(CONST(FIQ32MODE),bb_new_fiq32);
	BranchInst::Create(bb_switch_mode,bb_normal,mode_diff,bb_cur);
	mode_diff = new ICmpInst(*bb_switch_mode,ICmpInst::ICMP_NE,mode,CONST(USERBANK),"");
	BranchInst::Create(bb_switch_mode_real,bb_exit,mode_diff,bb_switch_mode);
	bb = bb_old_user32;
	LET(R13_USR,R(R13));
	LET(R14_USR,R(LR));	
	BranchInst::Create(bb_label1,bb);
	bb = bb_old_irq32;
	LET(R13_IRQ,R(R13));
	LET(R14_IRQ,R(LR));
	LET(SPSR_IRQ,R(SPSR_REG));
	BranchInst::Create(bb_label1,bb);
	bb = bb_old_svc32;
	LET(R13_SVC,R(R13));
	LET(R14_SVC,R(LR));
	LET(SPSR_SVC,R(SPSR_REG));
	BranchInst::Create(bb_label1,bb);
	bb = bb_old_abort32;
	LET(R13_ABORT,R(R13));
	LET(R14_ABORT,R(LR));
	LET(SPSR_ABORT,R(SPSR_REG));
	BranchInst::Create(bb_label1,bb);
	bb = bb_old_undef32;
	LET(R13_UNDEF,R(R13));
	LET(R14_UNDEF,R(LR));
	LET(SPSR_UNDEF,R(SPSR_REG));
	BranchInst::Create(bb_label1,bb);
	bb = bb_old_fiq32;
	LET(R13_FIRQ,R(R13));
	LET(R14_FIRQ,R(LR));
	LET(SPSR_FIRQ,R(SPSR_REG));
	BranchInst::Create(bb_label1,bb);
	bb = bb_new_user32;
	LET(R13,R(R13_USR));
	LET(LR,R(R14_USR));	
	LET(BANK_REG,CONST(USERBANK));
	BranchInst::Create(bb_label2,bb);
	bb = bb_new_irq32;
	LET(R13,R(R13_IRQ));
	LET(LR,R(R14_IRQ));
	LET(SPSR_REG,R(SPSR_IRQ));	
	LET(BANK_REG,CONST(IRQBANK));
	BranchInst::Create(bb_label2,bb);
	bb = bb_new_svc32;
	LET(R13,R(R13_SVC));
	LET(LR,R(R14_SVC));
	LET(SPSR_REG,R(SPSR_SVC));	
	LET(BANK_REG,CONST(SVCBANK));
	BranchInst::Create(bb_label2,bb);
	bb = bb_new_abort32;
	LET(R13,R(R13_ABORT));
	LET(LR,R(R14_ABORT));
	LET(SPSR_REG,R(SPSR_ABORT));	
	LET(BANK_REG,CONST(ABORTBANK));
	BranchInst::Create(bb_label2,bb);
	bb = bb_new_undef32;
	LET(R13,R(R13_UNDEF));
	LET(LR,R(R14_UNDEF));
	LET(SPSR_REG,R(SPSR_UNDEF));	
	LET(BANK_REG,CONST(UNDEFBANK));
	BranchInst::Create(bb_label2,bb);
	bb = bb_new_fiq32;
	LET(R13,R(R13_FIRQ));
	LET(LR,R(R14_FIRQ));
	LET(SPSR_REG,R(SPSR_FIRQ));	
	LET(BANK_REG,CONST(FIQBANK));
	BranchInst::Create(bb_label2,bb);
	bb = bb_label2;
	LET(MODE_REG,mode);
	BranchInst::Create(bb_normal,bb);
	//generate exit code
	arch_arm_exit(cpu,bb_exit,CONST(-2));
}
void arm_switch_mode(cpu_t *cpu)
{
	//arm_core_t* core = (arm_core_t*)get_cast_conf_obj(cpu->cpu_data, "arm_core_t");
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	switch_mode(core, core->Cpsr & 0x1f);
}

static int flush_current_page(cpu_t *cpu){
	//arm_core_t* core = (arm_core_t*)(cpu->cpu_data);
	arm_core_t* core = (arm_core_t*)get_cast_conf_obj(cpu->cpu_data, "arm_core_t");
	addr_t effec_pc = *(addr_t*)cpu->rf.pc;
//	printf("effec_pc is %x\n", effec_pc);
//	printf("in %s\n", __FUNCTION__);
	int ret = cpu->mem_ops.effective_to_physical(cpu, effec_pc, (uint32_t*)cpu->rf.phys_pc);
	cpu->current_page_phys = core->phys_pc & 0xfffff000;
	cpu->current_page_effec = core->pc & 0xfffff000;
	return ret;
}

void arm_dyncom_run(cpu_t* cpu){
	//arm_core_t* core = (arm_core_t*)get_cast_conf_obj(cpu->cpu_data, "arm_core_t");
	arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	uint32_t mode;

	addr_t phys_pc;
	if(is_user_mode(cpu)){
		addr_t phys_pc = core->Reg[15];
	}

	int rc = cpu_run(cpu);
	switch (rc) {
	case JIT_RETURN_NOERR: /* JIT code wants us to end execution */
	case JIT_RETURN_TIMEOUT:
                        break;
	case JIT_RETURN_SINGLESTEP:
	case JIT_RETURN_FUNCNOTFOUND:
//			printf("pc %x is not found\n", core->Reg[15]);
//			printf("phys_pc is %x\n", core->phys_pc);
//			printf("out of jit\n");
			if(!is_user_mode(cpu)){
				//switch_mode(core, core->Cpsr & 0x1f);
				if (flush_current_page(cpu)) {
					return;
				}
				clear_tag_page(cpu, core->phys_pc);
				cpu_tag(cpu, core->phys_pc);
				cpu->dyncom_engine->cur_tagging_pos ++;
				//cpu_translate(cpu, core->Reg[15]);
				cpu_translate(cpu, core->phys_pc);
			}
			else{
				cpu_tag(cpu, core->Reg[15]);
				cpu->dyncom_engine->cur_tagging_pos ++;
				cpu_translate(cpu, core->Reg[15]);
			}

		 /*
                  *If singlestep,we run it here,otherwise,break.
                  */
                        if (cpu->dyncom_engine->flags_debug & CPU_DEBUG_SINGLESTEP){
                                rc = cpu_run(cpu);
                                if(rc != JIT_RETURN_TRAP)
                                        break;
                        }
                        else
                                break;
	case JIT_RETURN_TRAP:
		if (core->syscallSig) {
			return;
		}
		if (core->abortSig) {
			return;
		}
//		printf("cpu maybe changed mode.\n");
//		printf("pc is %x\n", core->Reg[15]);
		//printf("icounter is %lld\n", cpu->icounter);
		//exit(-1);
		//core->Reg[15] += 4;
		/*mode = core->Cpsr & 0x1f;
		if ( (mode != core->Mode) && (!is_user_mode(cpu)) ) {
			switch_mode(core, mode);
			//exit(-1);
		}*/
		core->Reg[15] += 4;
		return;
			break;
		default:
                        fprintf(stderr, "unknown return code: %d\n", rc);
			skyeye_exit(-1);
        }// switch (rc)
	return;
}
/**
* @brief Debug function that will be called in every instruction execution, print the cpu state
*
* @param cpu the cpu_t instance
*/

void arm_dyncom_stop(){
}

void arm_dyncom_fini(){
	//cpu_free(cpu);
}
