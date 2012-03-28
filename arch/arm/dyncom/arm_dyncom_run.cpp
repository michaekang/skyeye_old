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
#include "arm_dyncom_tlb.h"
#include "dyncom/defines.h"
#include "common/mmu/arm1176jzf_s_mmu.h"
#include "armmmu.h"

#include "dyncom/arm_dyncom_mmu.h"

#define LOG_IN_CLR	skyeye_printf_in_color

void arm_switch_mode(cpu_t *cpu);
//#define MAX_REGNUM 16
extern const char* arm_regstr[MAX_REG_NUM];

enum{
	ARM_DYNCOM_CALLOUT_UNDEF = MAX_DYNCOM_CALLOUT,
	ARM_DYNCOM_CALLOUT_INV_MVA,
	ARM_DYNCOM_CALLOUT_INV_ASID,
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

struct symbolInfo *symbol_info_head = NULL;

void add_list_tail(struct symbolInfo *list)
{
        static struct symbolInfo *symbol_info_tail = NULL;
        if(!symbol_info_head) {
                symbol_info_head = symbol_info_tail = list;
                return;
        }
        symbol_info_tail->next = list;
        symbol_info_tail = list;
}

uint8_t *store_string_info(char *info, size_t size)
{
        static uint32_t max_vol = 0x1000;
        static uint32_t offset = 0;
        static uint32_t remain = 0x1000;
        static uint8_t *repo = NULL;

        uint8_t *str = NULL;
        uint8_t *new_repo = NULL;
        struct symbolInfo *item = NULL;

        //printf("%s, %d, %d\n", info, size, remain);
        if (repo == NULL) {
                repo = (uint8_t *)malloc(max_vol);
                printf("allocate %d bytes.\n", max_vol);
        }
        if (remain < size) {
                new_repo = (uint8_t *)malloc(max_vol * 2);
                printf("allocate %d bytes.\n", max_vol * 2);
                memcpy(new_repo, repo, offset);
                for (item = symbol_info_head; item; item = item->next) {
                        //printf("symbol : %s\taddress : %x\n", item->name, item->address);
                        item->name = new_repo + ((uint8_t *)item->name - (uint8_t *)repo);
                }
                free(repo);
                repo = new_repo;
                new_repo = NULL;
                remain += max_vol;
                max_vol *= 2;
        }
        str = repo + offset;
        memcpy(repo + offset, info, size);
        repo[offset + size] = '\0';
        offset += size;
        remain -= size;
        return str;
}
struct symbolInfo *alloc_symbol_info(uint8_t *str, uint32_t address)
{
        struct symbolInfo *item = (struct symbolInfo *)malloc(sizeof(struct symbolInfo));
        if (item == NULL) {
                printf("Can't allocate more memory in %s\n", __FUNCTION__);
                exit(-1);
        }
        item->next = NULL;
        item->name = str;
        item->address = address;
        return item;
}

struct symbolInfo *search_symbol_info_by_addr(uint32_t address)
{
        struct symbolInfo *prev = NULL, *item = NULL;
        for (item = symbol_info_head; item; item = item->next) {
                if(address == item->address) {
                        return item;
                } else if(address > item->address){
                        prev = item;
                        continue;
                } else {
                        return prev;
                }
        }
        printf("Can not found the address 0x%x in System.map.\n", address);
        //exit(-1);
        return NULL;
}

void print_func_name(uint32_t address)
{
        static struct symbolInfo *last_found = NULL;
        static uint32_t last_address = 0;
        struct symbolInfo *new_found = NULL;
        new_found = search_symbol_info_by_addr(address);
        if (new_found == NULL) {
                return;
        }
        if (last_found != new_found) {
                if (last_found) {
                        LOG_IN_CLR(LIGHT_RED, "exit function %s 0x%x\n", last_found->name, last_address);
                }
                printf("%s\n", new_found->name);
                last_found = new_found;
                last_address = address;
        } else {
		last_address = address;
	}
}

void load_symbol_from_sysmap()
{
        char symbol_address[100];
        char symbol_name[100];
        char type = 0;
        char *str = NULL;
        struct symbolInfo *item = NULL;
        int i = 0;

        uint32_t address = 0;
        FILE *sysmap = fopen("/home/myesis/linux-2.6.35.y/System.map", "r");

        do {
                    if (3 != fscanf(sysmap, "%s %c %s", symbol_address, &type, symbol_name)) break;
                    address = strtol(symbol_address, NULL, 16);
                    while (symbol_name[i] != '\0') {
                            //printf("%c\n", symbol_name[i]);
                            i++;
                    }
                    //printf("symbol:%s\taddress:%x\tsize:%d\n", symbol_name, address, i);
                    str = (char *)store_string_info(symbol_name, i + 1);
                    item = alloc_symbol_info((uint8_t *)str, address);
                    add_list_tail(item);
        } while (1);
        for (item = symbol_info_head; item; item = item->next) {
                printf("symbol : %s\taddress : %x\n", item->name, item->address);
        }
}

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
	info->float_size = 64;
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

	info->register_count[CPU_REG_FPR] = 0;
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
				| CPU_CODEGEN_VERIFY
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
	new StoreInst(n, cpu->ptr_N, false, bb);
	new StoreInst(z, cpu->ptr_Z, false, bb);
	new StoreInst(c, cpu->ptr_C, false, bb);
	new StoreInst(v, cpu->ptr_V, false, bb);
	new StoreInst(t, cpu->ptr_T, false, bb);
}

static void arch_arm_spill_reg_state(cpu_t *cpu, BasicBlock *bb)
{
		/* Save N Z C V T */
	Value *z = SHL(ZEXT32(LOAD(cpu->ptr_Z)), CONST(30));
	Value *n = SHL(ZEXT32(LOAD(cpu->ptr_N)), CONST(31));
	Value *c = SHL(ZEXT32(LOAD(cpu->ptr_C)), CONST(29));
	Value *v = SHL(ZEXT32(LOAD(cpu->ptr_V)), CONST(28));
	Value *t = SHL(ZEXT32(LOAD(cpu->ptr_T)), CONST(THUMB_BIT));
	Value *nzcv = OR(OR(OR(z, n), c), v);
	Value *cpsr = OR(AND(LOAD(cpu->ptr_gpr[16]), CONST(0x0fffffff)), nzcv);
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

	arm_core_t* core = (arm_core_t*)get_cast_conf_obj(cpu->cpu_data, "arm_core_t");
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
	Type const *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
	Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
	std::vector<Value *> params;
	params.push_back(v_cpu_ptr);
	/* When using a custom callout, must put the callout index as argument for dyncom_callout */
	params.push_back(CONST(ARM_DYNCOM_CALLOUT_UNDEF));
	//params.push_back(CONST(instr)); // no need for now, the callout func takes no argument
	CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_UNDEF], params.begin(), params.end(), "", bb);
}

/* Undefined instruction handler initialization. Should be called once at init */
static void 
arch_arm_undef_init(cpu_t *cpu){
	//types
	std::vector<const Type*> type_func_undef_args;
	PointerType *type_intptr = PointerType::get(cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX()), 0);
	const IntegerType *type_i32 = IntegerType::get(_CTX(), 32);
	type_func_undef_args.push_back(type_intptr);	/* intptr *cpu */
	type_func_undef_args.push_back(type_i32);	/* unsinged int */
	FunctionType *type_func_undef_callout = FunctionType::get(
		Type::getInt32Ty(cpu->dyncom_engine->mod->getContext()),	//return
		type_func_undef_args,	/* Params */
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
arch_arm_invalidate_by_mva(cpu_t *cpu, BasicBlock *bb, Value* mva)
{
	if (cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_MVA] == NULL) {
		printf("in %s Could not find callout\n", __FUNCTION__);
		return;
	}
	Type const *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
	Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
	std::vector<Value *> params;
	params.push_back(v_cpu_ptr);
	params.push_back(CONST(ARM_DYNCOM_CALLOUT_INV_MVA));
	params.push_back(mva);
	/* When using a custom callout, must put the callout index as argument for dyncom_callout */
	//params.push_back(CONST(instr)); // no need for now, the callout func takes no argument
	CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_MVA], params.begin(), params.end(), "", bb);
}

static void 
arch_arm_invalidate_by_mva_init(cpu_t *cpu){
	//types
	std::vector<const Type*> type_func_args;
	PointerType *type_intptr = PointerType::get(cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX()), 0);
	const IntegerType *type_i32 = IntegerType::get(_CTX(), 32);
	type_func_args.push_back(type_intptr);	/* intptr *cpu */
	type_func_args.push_back(type_i32);	/* unsinged int */
	type_func_args.push_back(type_i32);	/* mva */
	FunctionType *type_func_callout = FunctionType::get(
		Type::getInt32Ty(cpu->dyncom_engine->mod->getContext()),	//return
		type_func_args,	/* Params */
		false);		      	/* isVarArg */
	/* For a custom callout, the dyncom_calloutX functions should be used */
	Constant *func_const = cpu->dyncom_engine->mod->getOrInsertFunction("dyncom_callout1",	//function name
		type_func_callout);	//return
	if(func_const == NULL)
		fprintf(stderr, "Error:cannot insert function:undefined_instr_callout.\n");
	Function *func = cast<Function>(func_const);
	func->setCallingConv(CallingConv::C);
	cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_MVA] = func;
	cpu->dyncom_engine->arch_func[ARM_DYNCOM_CALLOUT_INV_MVA] = (void*)invalidate_by_mva;
}

void
arch_arm_invalidate_by_asid(cpu_t *cpu, BasicBlock *bb, Value* asid)
{
	if (cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_ASID] == NULL) {
		printf("in %s Could not find callout\n", __FUNCTION__);
		return;
	}
	Type const *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
	Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
	std::vector<Value *> params;
	params.push_back(v_cpu_ptr);
	/* When using a custom callout, must put the callout index as argument for dyncom_callout */
	params.push_back(CONST(ARM_DYNCOM_CALLOUT_INV_ASID));
	params.push_back(asid);
	//params.push_back(CONST(instr)); // no need for now, the callout func takes no argument
	CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_ASID], params.begin(), params.end(), "", bb);
}
static void 
arch_arm_invalidate_by_asid_init(cpu_t *cpu){
	//types
	std::vector<const Type*> type_func_args;
	PointerType *type_intptr = PointerType::get(cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX()), 0);
	const IntegerType *type_i32 = IntegerType::get(_CTX(), 32);
	type_func_args.push_back(type_intptr);	/* intptr *cpu */
	type_func_args.push_back(type_i32);	/* unsinged int */
	type_func_args.push_back(type_i32);	/* asid */
	FunctionType *type_func_callout = FunctionType::get(
		Type::getInt32Ty(cpu->dyncom_engine->mod->getContext()),	//return
		type_func_args,	/* Params */
		false);		      	/* isVarArg */
	/* For a custom callout, the dyncom_calloutX functions should be used */
	Constant *func_const = cpu->dyncom_engine->mod->getOrInsertFunction("dyncom_callout1",	//function name
		type_func_callout);	//return
	if(func_const == NULL)
		fprintf(stderr, "Error:cannot insert function:undefined_instr_callout.\n");
	Function *func = cast<Function>(func_const);
	func->setCallingConv(CallingConv::C);
	cpu->dyncom_engine->ptr_arch_func[ARM_DYNCOM_CALLOUT_INV_ASID] = func;
	cpu->dyncom_engine->arch_func[ARM_DYNCOM_CALLOUT_INV_ASID] = (void*)invalidate_by_asid;
}



/**
* @brief Scan the bb usage and recycle some bb to free memory
*
* @param cpu
*/
static void recycle_bb(void* priv_data){
	cpu_t* cpu = (cpu_t*)priv_data;
	printf("\ncurrent funcions is %d\n", cpu->dyncom_engine->functions);
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
	//cpu->cpu_data = (conf_object_t*)core;
	cpu->cpu_data = get_conf_obj_by_cast(core, "arm_core_t");
	
	/* init the reg structure */
	cpu->rf.pc = &core->Reg[15];
	/* Under user mode sim, both phys_pc and pc are pointed to Reg 15 */
	if(is_user_mode(cpu))
		cpu->rf.phys_pc = &core->Reg[15];
	else
		cpu->rf.phys_pc = &core->phys_pc;
	cpu->rf.grf = core->Reg;
	//cpu->rf.srf = core->Spsr;
	//cpu->rf.srf = &core->phys_pc;
	cpu->rf.srf = core->Reg_usr;
	cpu->rf.srf = core->ExtReg;
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

#if FAST_MEMORY
	if (pref->user_mode_sim){
		cpu->dyncom_engine->RAM = (uint8_t*)get_dma_addr(0);
	}
	else{
		cpu->dyncom_engine->RAM = (uint8_t*)get_dma_addr(BANK0_START);
		//cpu->dyncom_engine->TLB = (uint32_t*)new_tlb(TLB_SIZE, ASID_SIZE);
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
	new_tlb(TLB_SIZE, ASID_SIZE);
	/* undefined instr handler init */
	arch_arm_undef_init(cpu);
	arch_arm_invalidate_by_asid_init(cpu);
	arch_arm_invalidate_by_mva_init(cpu);
	
	init_compiled_queue(cpu);
	if(running_mode == HYBRID || running_mode == PURE_DYNCOM){
		int timer_id;
		//create_thread_scheduler(1000000, Periodic_sched, recycle_bb, (void *)cpu, &timer_id);
	}
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

void arm_switch_mode(cpu_t *cpu)
{
	arm_core_t* core = (arm_core_t*)get_cast_conf_obj(cpu->cpu_data, "arm_core_t");
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
				switch_mode(core, core->Cpsr & 0x1f);
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
		mode = core->Cpsr & 0x1f;
		if ( (mode != core->Mode) && (!is_user_mode(cpu)) ) {
			switch_mode(core, mode);
			//exit(-1);
		}
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
