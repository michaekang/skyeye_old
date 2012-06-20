/**
 * @file interface.cpp
 * 
 * This is the interface to the client.
 * 
 * @author OS Center,TsingHua University (Ported from libcpu)
 * @date 11/11/2010
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "llvm/Analysis/Verifier.h"
#include "llvm/ExecutionEngine/JIT.h"
#include "llvm/Module.h"
#include "llvm/Target/TargetData.h"
#include "llvm/DerivedTypes.h"
#include "llvm/ExecutionEngine/ExecutionEngine.h"

#include "llvm/LLVMContext.h"
#include "llvm/PassManager.h"
#include "llvm/Analysis/Verifier.h"
#include "llvm/Analysis/Passes.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Support/IRBuilder.h"
#include "llvm/Support/TargetSelect.h"

/* project global headers */
#include "skyeye_dyncom.h"
#include "skyeye_mm.h"
#include "dyncom/dyncom_llvm.h"
#include "dyncom/tag.h"
#include "translate_all.h"
#include "translate_singlestep.h"
#include "translate_singlestep_bb.h"
#include "function.h"
#include "optimize.h"
#include "stat.h"
#include "dyncom/basicblock.h"
#include "dyncom/tlb.h"

#include "skyeye_log.h"
#include "skyeye.h"
#include "bank_defs.h"
extern "C" {
#include "skyeye_disas.h"
}

static void debug_func_init(cpu_t *cpu);
static void syscall_func_init(cpu_t *cpu);
#define IS_LITTLE_ENDIAN(cpu) (((cpu)->info.common_flags & CPU_FLAG_ENDIAN_MASK) == CPU_FLAG_ENDIAN_LITTLE)

static inline bool
is_valid_gpr_size(size_t size)
{
	switch (size) {
		case 0: case 1: case 8: case 16: case 32: case 64:
			return true;
		default:
			return false;
	}
}

static inline bool
is_valid_fpr_size(size_t size)
{
	switch (size) {
		case 0: case 32: case 64: case 80: case 128:
			return true;
		default:
			return false;
	}
}

static inline bool
is_valid_vr_size(size_t size)
{
	switch (size) {
		case 0: case 64: case 128:
			return true;
		default:
			return false;
	}
}

int func_not_found_func(uint8_t *RAM, void *grf, void *srf, void *frf, fp_read_memory_t readfp, fp_write_memory_t writefp, unsigned long TLB){
        return JIT_RETURN_FUNC_BLANK;
}

//////////////////////////////////////////////////////////////////////
// cpu_t
//////////////////////////////////////////////////////////////////////
/**
 * @brief Create a new CPU core structure and initialize the llmv Module,ExectionEngine
 *
 * @param arch the architecture type of CPU core
 * @param flags some flags,such as floating point,little/big endian 
 * @param arch_flags target machine bits 
 *
 * @return pointer of CPU core structure 
 */
cpu_t *
cpu_new(uint32_t flags, uint32_t arch_flags, arch_func_t arch_func)
{
	cpu_t *cpu;

	llvm::InitializeNativeTarget();

	cpu = new cpu_t;
	assert(cpu != NULL);
	memset(&cpu->info, 0, sizeof(cpu->info));
	memset(&cpu->rf, 0, sizeof(cpu->rf));

	cpu->info.name = "noname";
	cpu->info.common_flags = flags;
	cpu->info.arch_flags = arch_flags;
	//assert(!arch_func);
	cpu->f = arch_func;
	cpu->icounter = 0;

	cpu->dyncom_engine = new dyncom_engine_t;
	cpu->dyncom_engine->code_start = 0;
	cpu->dyncom_engine->code_end = 0;
	cpu->dyncom_engine->code_entry = 0;
	cpu->dyncom_engine->tag = NULL;

	/* init hash fast map */
#ifdef HASH_FAST_MAP
#if L3_HASHMAP
	cpu->dyncom_engine->fmap = (fast_map)malloc(sizeof(void***) * HASH_MAP_SIZE_L1);
	memset(cpu->dyncom_engine->fmap, NULL, sizeof(void***) * HASH_MAP_SIZE_L1);
#else
	cpu->dyncom_engine->fmap = (fast_map)malloc(sizeof(void *) * HASH_FAST_MAP_SIZE);
	for(int i = 0; i < HASH_FAST_MAP_SIZE; i++)
		cpu->dyncom_engine->fmap[i] = (void *)func_not_found_func;
#endif /* #if L3_HASHMAP */

#endif
	uint32_t i;
	for (i = 0; i < 4; i++) {
		cpu->dyncom_engine->tag_array[i] = NULL;
		cpu->dyncom_engine->code_size[i] = 0;
	}
	cpu->dyncom_engine->tag_table = (tag_t ***)skyeye_mm_zero(TAG_LEVEL1_TABLE_SIZE * sizeof(tag_t **));

	for (i = 0; i < sizeof(cpu->dyncom_engine->func)/sizeof(*cpu->dyncom_engine->func); i++)
		cpu->dyncom_engine->func[i] = NULL;
	for (i = 0; i < sizeof(cpu->dyncom_engine->fp)/sizeof(*cpu->dyncom_engine->fp); i++)
		cpu->dyncom_engine->fp[i] = NULL;
	cpu->dyncom_engine->functions = 0;

	cpu->dyncom_engine->flags_codegen = CPU_CODEGEN_OPTIMIZE;
	cpu->dyncom_engine->flags_debug = CPU_DEBUG_NONE;
	cpu->dyncom_engine->flags_hint = CPU_HINT_NONE;
	cpu->dyncom_engine->flags = 0;

	// init the frontend
	cpu->f.init(cpu, &cpu->info, &cpu->rf);

	assert(is_valid_gpr_size(cpu->info.register_size[CPU_REG_GPR]) &&
		"the specified GPR size is not guaranteed to work");
	assert(is_valid_fpr_size(cpu->info.register_size[CPU_REG_FPR]) &&
		"the specified FPR size is not guaranteed to work");
	assert(is_valid_vr_size(cpu->info.register_size[CPU_REG_VR]) &&
		"the specified VR size is not guaranteed to work");
	assert(is_valid_gpr_size(cpu->info.register_size[CPU_REG_XR]) &&
		"the specified XR size is not guaranteed to work");

	uint32_t count = cpu->info.register_count[CPU_REG_GPR];
	if (count != 0) {
		cpu->ptr_gpr = (Value **)calloc(count, sizeof(Value *));
		cpu->in_ptr_gpr = (Value **)calloc(count, sizeof(Value *));
	} else {
		cpu->ptr_gpr = NULL;
		cpu->in_ptr_gpr = NULL;
	}

	count = cpu->info.register_count[CPU_REG_XR];
	if (count != 0) {
		cpu->ptr_xr = (Value **)calloc(count, sizeof(Value *));
		cpu->in_ptr_xr = (Value **)calloc(count, sizeof(Value *));
	} else {
		cpu->ptr_xr = NULL;
		cpu->in_ptr_xr = NULL;
	}

	count = cpu->info.register_count[CPU_REG_SPR];
	if (count != 0) {
		cpu->ptr_spr = (Value **)calloc(count, sizeof(Value *));
		cpu->in_ptr_spr = (Value **)calloc(count, sizeof(Value *));
	} else {
		cpu->ptr_spr = NULL;
		cpu->in_ptr_spr = NULL;
	}

	count = cpu->info.register_count[CPU_REG_FPR];
	if (count != 0) {
		cpu->ptr_fpr = (Value **)calloc(count, sizeof(Value *));
		cpu->in_ptr_fpr = (Value **)calloc(count, sizeof(Value *));
	} else {
		cpu->ptr_fpr = NULL;
		cpu->in_ptr_fpr = NULL;
	}

	if (cpu->info.psr_size != 0) {
		cpu->ptr_FLAG = (Value **)calloc(cpu->info.flags_count,
				sizeof(Value*));
		assert(cpu->ptr_FLAG != NULL);
	}

	// init LLVM
	cpu->dyncom_engine->mod = new Module(cpu->info.name, _CTX());
	assert(cpu->dyncom_engine->mod != NULL);
	cpu->dyncom_engine->exec_engine = ExecutionEngine::create(cpu->dyncom_engine->mod);
	assert(cpu->dyncom_engine->exec_engine != NULL);

	// check if FP80 and FP128 are supported by this architecture.
	// XXX there is a better way to do this?
	std::string data_layout = cpu->dyncom_engine->exec_engine->getTargetData()->getStringRepresentation();
	if (data_layout.find("f80") != std::string::npos) {
		LOG("INFO: FP80 supported.\n");
		cpu->dyncom_engine->flags |= CPU_FLAG_FP80;
	}
	if (data_layout.find("f128") != std::string::npos) {
		LOG("INFO: FP128 supported.\n");
		cpu->dyncom_engine->flags |= CPU_FLAG_FP128;
	}

	// check if we need to swap guest memory.
	if (cpu->dyncom_engine->exec_engine->getTargetData()->isLittleEndian()
			^ IS_LITTLE_ENDIAN(cpu))
		cpu->dyncom_engine->flags |= CPU_FLAG_SWAPMEM;

	cpu->timer_total[TIMER_TAG] = 0;
	cpu->timer_total[TIMER_FE] = 0;
	cpu->timer_total[TIMER_BE] = 0;
	cpu->timer_total[TIMER_RUN] = 0;
	cpu->timer_total[TIMER_OPT] = 0;

	debug_func_init(cpu);
	syscall_func_init(cpu);
#ifndef __WIN32__
	if(pthread_rwlock_init(&(cpu->dyncom_engine->rwlock), NULL)){
		fprintf(stderr, "can not initilize the rwlock\n");
	}
#endif

	return cpu;
}
/**
 * @brief free CPU core structure
 *
 * @param cpu CPU core structure
 */
void
cpu_free(cpu_t *cpu)
{
	if (cpu->f.done != NULL)
		cpu->f.done(cpu);
	if (cpu->dyncom_engine->exec_engine != NULL) {
		if (cpu->dyncom_engine->cur_func != NULL) {
			cpu->dyncom_engine->exec_engine->freeMachineCodeForFunction(cpu->dyncom_engine->cur_func);
			cpu->dyncom_engine->cur_func->eraseFromParent();
		}
		delete cpu->dyncom_engine->exec_engine;
	}
	if (cpu->ptr_FLAG != NULL)
		free(cpu->ptr_FLAG);
	if (cpu->in_ptr_fpr != NULL)
		free(cpu->in_ptr_fpr);
	if (cpu->ptr_fpr != NULL)
		free(cpu->ptr_fpr);
	if (cpu->in_ptr_xr != NULL)
		free(cpu->in_ptr_xr);
	if (cpu->ptr_xr != NULL)
		free(cpu->ptr_xr);
	if (cpu->in_ptr_gpr != NULL)
		free(cpu->in_ptr_gpr);
	if (cpu->ptr_gpr != NULL)
		free(cpu->ptr_gpr);

	delete cpu;
}
/**
 * @brief Set cpu RAM
 *
 * @param cpu CPU core structure
 * @param r RAM base address
 */
void
cpu_set_ram(cpu_t*cpu, uint8_t *r)
{
	cpu->dyncom_engine->RAM = r;
}
/**
 * @brief Set cpu codegen flags
 *
 * @param cpu CPU core structure
 * @param f flag
 */
void
cpu_set_flags_codegen(cpu_t *cpu, uint32_t f)
{
	cpu->dyncom_engine->flags_codegen = f;
}
/**
 * @brief set cpu debug flags
 *
 * @param cpu CPU core structure
 * @param f flag
 */
void
cpu_set_flags_debug(cpu_t *cpu, uint32_t f)
{
	cpu->dyncom_engine->flags_debug = f;
}
/**
 * @brief Set cpu hint flags
 *
 * @param cpu CPU core structure
 * @param f flag
 */
void
cpu_set_flags_hint(cpu_t *cpu, uint32_t f)
{
	cpu->dyncom_engine->flags_hint = f;
}
/**
 * @brief tag from pc and stop if necessary
 *
 * @param cpu CPU core structure
 * @param pc start tagging address
 */
void
cpu_tag(cpu_t *cpu, addr_t pc)
{
	UPDATE_TIMING(cpu, TIMER_TAG, true);
	tag_start(cpu, pc);
	UPDATE_TIMING(cpu, TIMER_TAG, false);
}
/**
 * @brief Save the map from native code function to entry address.
 *
 * @param cpu CPU core structure
 * @param native_code_func entry of the native code function
 */
void save_addr_in_func(cpu_t *cpu, void *native_code_func)
{
#ifdef HASH_FAST_MAP
#if L3_HASHMAP
	bbaddr_map &bb_addr = cpu->dyncom_engine->func_bb[cpu->dyncom_engine->cur_func];
	bbaddr_map::iterator i = bb_addr.begin();
#ifndef __WIN32__
	pthread_rwlock_wrlock(&(cpu->dyncom_engine->rwlock));
#endif
	for (; i != bb_addr.end(); i++){
		if((get_tag(cpu, i->first) & TAG_AFTER_NEW_BB) && !is_start_of_basicblock(cpu, i->first)){
			continue;
		}
		if(cpu->dyncom_engine->fmap[HASH_MAP_INDEX_L1(i->first)] == NULL)
			init_fmap_l2(cpu->dyncom_engine->fmap, i->first);
		if(cpu->dyncom_engine->fmap[HASH_MAP_INDEX_L1(i->first)][HASH_MAP_INDEX_L2(i->first)] == NULL)
			init_fmap_l3(cpu->dyncom_engine->fmap, i->first);
		cpu->dyncom_engine->fmap[HASH_MAP_INDEX_L1(i->first)][HASH_MAP_INDEX_L2(i->first)][HASH_MAP_INDEX_L3(i->first)] =
			native_code_func;
	}
	/* shenoubang add win32 2012-6-14 */
#ifndef __WIN32__
	if(pthread_rwlock_unlock(&(cpu->dyncom_engine->rwlock))){
		fprintf(stderr, "unlock error\n");
	}
#endif
#else
	bbaddr_map &bb_addr = cpu->dyncom_engine->func_bb[cpu->dyncom_engine->cur_func];
	bbaddr_map::iterator i = bb_addr.begin();
         for (; i != bb_addr.end(); i++)
                 cpu->dyncom_engine->fmap[i->first & (HASH_FAST_MAP_SIZE - 1)] = native_code_func;
#endif /* #if L3_HASHMAP */

#else
	bbaddr_map &bb_addr = cpu->dyncom_engine->func_bb[cpu->dyncom_engine->cur_func];
	bbaddr_map::iterator i = bb_addr.begin();
	for (; i != bb_addr.end(); i++)
		cpu->dyncom_engine->fmap[i->first] = native_code_func;
#endif
}

void disas_insn_in_jit(cpu_t *cpu)
{
	vector<addr_t> addrset = cpu->dyncom_engine->insns_in_jit;

	uint32_t insn;
	uint32_t given;
	uint8_t func_attrs = cpu->dyncom_engine->func_attr[cpu->dyncom_engine->functions];
	int size = 4;
	printf("-----disas begin-----\n");
	if (func_attrs & FUNC_ATTR_THUMB) {
		set_thumb_mode(1);
		size = 2;
		printf("thumb\n");
	}

	vector<addr_t>::iterator it = addrset.begin();
	for (; it != addrset.end(); ++it) {
		insn = 0;
		given = 0;
		if (is_start_of_basicblock(cpu, *it)) {
			printf("%x:\n", *it);
		}
		if (func_attrs & FUNC_ATTR_THUMB) {
			size = 2;
			bus_read(16, *it, &insn);
			if ((insn & 0xF800) == 0xF800
			      || (insn & 0xF800) == 0xF000
			      || (insn & 0xF800) == 0xE800) {
				bus_read(16, *it + 2, &given);
				size = 4;
				/* skip next thumb insn */
		//		++it;
				insn = (insn << 16) | (given & 0xffff);
				disas(stdout, &insn, size, *it);

#if 1
				if ((it + 1) != addrset.end()) {
					if (*(it + 1) == *it + 2)
						++it;
				}
#endif
			} else
				disas(stdout, &insn, size, *it);
		}
		else {
			bus_read(32, *it, &insn);
			size = 4;
			disas(stdout, &insn, size, *it);
		}
//		printf("%x\n", insn);
	}

	if (func_attrs & FUNC_ATTR_THUMB)
		set_thumb_mode(0);
	printf("-----disas end-----\n");
}

/**
 * @brief Create llvm JIT Function and translate instructions to fill the JIT Function.
 *	Optimize the llvm IR and save the function and its entry address to map.
 *
 * @param cpu CPU core structure
 */
static void *
cpu_translate_function(cpu_t *cpu, addr_t addr)
{
	BasicBlock *bb_ret, *bb_trap, *label_entry, *bb_start, *bb_timeout;
	static int jit_num = 0;
	//addr_t start_addr = cpu->f.get_pc(cpu, cpu->rf.grf);
	addr_t start_addr = addr;
	
	//printf("In %s, addr=0x%x\n", __FUNCTION__, addr);
	/* create function and fill it with std basic blocks */
	cpu->dyncom_engine->cur_func = cpu_create_function(cpu, "jitmain", &bb_ret, &bb_trap, &bb_timeout, &label_entry);

	/* TRANSLATE! */
	UPDATE_TIMING(cpu, TIMER_FE, true);
#ifndef __WIN32__
	if (cpu->dyncom_engine->flags_debug & CPU_DEBUG_SINGLESTEP) {
		bb_start = cpu_translate_singlestep(cpu, bb_ret, bb_trap);
	} else if (cpu->dyncom_engine->flags_debug & CPU_DEBUG_SINGLESTEP_BB) {
		bb_start = cpu_translate_singlestep_bb(cpu, bb_ret, bb_trap);
	} else {
		bb_start = cpu_translate_all(cpu, bb_ret, bb_trap, bb_timeout);
	}
#else
	bb_start = cpu_translate_all(cpu, bb_ret, bb_trap, bb_timeout);
#endif

	UPDATE_TIMING(cpu, TIMER_FE, false);

	/* finish entry basicblock */
	BranchInst::Create(bb_start, label_entry);

	jit_num++;
	if (cpu->dyncom_engine->flags_debug & CPU_DEBUG_PRINT_IR)
		cpu->dyncom_engine->cur_func->dump();

	if (cpu->dyncom_engine->flags_debug & CPU_DEBUG_PRINT_DISAS)
		disas_insn_in_jit(cpu);
	/* make sure everything is OK */
	if (cpu->dyncom_engine->flags_codegen & CPU_CODEGEN_VERIFY){
		if(verifyFunction(*cpu->dyncom_engine->cur_func, PrintMessageAction) == true){
			printf("------------------ JIT Function Dump ---------------\n");
			cpu->dyncom_engine->cur_func->dump();
			exit(-1);
		}
	}

	if (cpu->dyncom_engine->flags_codegen & CPU_CODEGEN_OPTIMIZE) {
		UPDATE_TIMING(cpu, TIMER_OPT, true);
		LOG("*** Optimizing...");
		optimize(cpu);
		LOG("done.\n");
		UPDATE_TIMING(cpu, TIMER_OPT, false);
		if (cpu->dyncom_engine->flags_debug & CPU_DEBUG_PRINT_IR_OPTIMIZED)
			cpu->dyncom_engine->mod->dump();
	}

	LOG("*** Translating...");
	UPDATE_TIMING(cpu, TIMER_BE, true);
	cpu->dyncom_engine->fp[cpu->dyncom_engine->functions] = cpu->dyncom_engine->exec_engine->getPointerToFunction(cpu->dyncom_engine->cur_func);
	//cpu->dyncom_engine->fmap[start_addr] = cpu->dyncom_engine->fp[cpu->dyncom_engine->functions];
	save_addr_in_func(cpu, cpu->dyncom_engine->fp[cpu->dyncom_engine->functions]);
	LOG("Generate native code for %x\n", start_addr);
	UPDATE_TIMING(cpu, TIMER_BE, false);
	LOG("done.\n");

	cpu->dyncom_engine->functions++;/* Bug."functions" member could not be reset. */
	if(cpu->dyncom_engine->functions == JIT_NUM){
		printf("JIT function number is %d, Please set your cache bigger.\n", cpu->dyncom_engine->functions);
		skyeye_exit(0);
	}
	return cpu->dyncom_engine->fp[cpu->dyncom_engine->functions - 1];
}

/**
 * @brief forces ahead of time translation (e.g. for benchmarking the run)
 *
 * @param cpu CPU core structure
 */
void *
cpu_translate(cpu_t *cpu, addr_t addr)
{
	return cpu_translate_function(cpu, addr);
}

//typedef int (*fp_t)(uint8_t *RAM, void *grf, void *frf, read_memory_t readfp, write_memory_t writefp);
typedef int (*fp_t)(uint8_t *RAM, void *grf, void *srf, void *frf, fp_read_memory_t readfp, fp_write_memory_t writefp, unsigned long read_tlb_index, unsigned long write_tlb_index, unsigned long mixed_tlb, unsigned long io_tlb, uint32_t user_mode);
/* cpu run for user mode application */
int
um_cpu_run(cpu_t *cpu){
	int ret;
	typedef int (*um_fp_t)(uint8_t *RAM, void *grf, void *srf, void *frf);
	um_fp_t pfunc = NULL;
	generic_address_t pc;
	/* before running jit, update old_icounter first */
	cpu->old_icounter = cpu->icounter;
	while(1){
		pc = cpu->f.get_pc(cpu, cpu->rf.grf);
		*(addr_t*)cpu->rf.phys_pc = pc;
#if L3_HASHMAP
		fast_map hash_map = cpu->dyncom_engine->fmap;
		if(hash_map[HASH_MAP_INDEX_L1(pc)] == NULL)
			return JIT_RETURN_FUNCNOTFOUND;
		else if(hash_map[HASH_MAP_INDEX_L1(pc)][HASH_MAP_INDEX_L2(pc)] == NULL)
			return JIT_RETURN_FUNCNOTFOUND;
		else if(hash_map[HASH_MAP_INDEX_L1(pc)][HASH_MAP_INDEX_L2(pc)][HASH_MAP_INDEX_L3(pc)] == NULL)
			return JIT_RETURN_FUNCNOTFOUND;
		pfunc = (um_fp_t)hash_map[HASH_MAP_INDEX_L1(pc)][HASH_MAP_INDEX_L2(pc)][HASH_MAP_INDEX_L3(pc)];
#else
		fast_map hash_map = cpu->dyncom_engine->fmap;
		pfunc = (um_fp_t)hash_map[pc & (HASH_FAST_MAP_SIZE - 1)];
		if(!pfunc)
			return JIT_RETURN_FUNCNOTFOUND;
#endif
		//ret = pfunc(cpu->dyncom_engine->RAM, cpu->rf.grf, cpu->rf.srf, cpu->rf.frf, cpu->mem_ops.read_memory, cpu->mem_ops.write_memory, cpu->mem_ops.check_mm);
		ret = pfunc(cpu->dyncom_engine->RAM, cpu->rf.grf, cpu->rf.srf, cpu->rf.frf);
		if(ret != JIT_RETURN_FUNCNOTFOUND)
			return ret;
	}
}

#ifdef __GNUC__
void __attribute__((noinline))
breakpoint() {
asm("nop");
}
#else
void breakpoint() {}
#endif
/**
 * @brief search the function of current address in map.If find,run it.Other wise,return.
 *
 * @param cpu CPU core structure
 * @param debug_function debug function 
 *
 * @return the return value of JIT Function
 */
int
cpu_run(cpu_t *cpu)
{
	addr_t pc = 0, phys_pc = 0;
	uint32_t i;
	int ret = 0;
	fp_t pfunc = NULL;


	/* try to find the entry in all functions */
	while(true) {
		pc = cpu->f.get_pc(cpu, cpu->rf.grf);
		int ret = cpu->mem_ops.effective_to_physical(cpu, pc, &phys_pc);
		/* if ISI exception happened here, we mush set pc to phys_pc which is
		 * the exception handler address.*/
		if(ret) {
			pc = phys_pc;
			return JIT_RETURN_TRAP;
		}
		*(addr_t*)cpu->rf.phys_pc = phys_pc;
		cpu->current_page_phys = phys_pc & 0xfffff000;
		cpu->current_page_effec = pc & 0xfffff000; 
#ifdef HASH_FAST_MAP
#if L3_HASHMAP
		fast_map hash_map = cpu->dyncom_engine->fmap;
		if(hash_map[HASH_MAP_INDEX_L1(phys_pc)] == NULL)
			return JIT_RETURN_FUNCNOTFOUND;
		else if(hash_map[HASH_MAP_INDEX_L1(phys_pc)][HASH_MAP_INDEX_L2(phys_pc)] == NULL)
			return JIT_RETURN_FUNCNOTFOUND;
		else if(hash_map[HASH_MAP_INDEX_L1(phys_pc)][HASH_MAP_INDEX_L2(phys_pc)][HASH_MAP_INDEX_L3(phys_pc)] == NULL)
			return JIT_RETURN_FUNCNOTFOUND;
		pfunc = (fp_t)hash_map[HASH_MAP_INDEX_L1(phys_pc)][HASH_MAP_INDEX_L2(phys_pc)][HASH_MAP_INDEX_L3(phys_pc)];
#else
		fast_map hash_map = cpu->dyncom_engine->fmap;
		pfunc = (fp_t)hash_map[phys_pc & (HASH_FAST_MAP_SIZE - 1)];
#endif
#else
		fast_map &func_addr = cpu->dyncom_engine->fmap;
		fast_map::const_iterator it = func_addr.find(pc);
		if (it != func_addr.end()) {
			pfunc = (fp_t)it->second;
		} else{
			LOG("jitfunction not found:key=0x%x\n", pc);
			return JIT_RETURN_FUNCNOTFOUND;
		}
#endif
		LOG("******Run jit 0x%x\n", pc);
		int context_id = (*(uint32_t *)(cpu->rf.context_id)) & 0xFF;
		#define USER32MODE 16L
		int user_mode = (((*(uint32_t *)cpu->rf.cpsr) & 0x1F) == USER32MODE);
		unsigned long offset = context_id * TLB_SIZE * TLB_ENTRY_SIZE;
		if(user_mode)
			ret = pfunc(cpu->dyncom_engine->RAM, cpu->rf.grf, cpu->rf.srf, cpu->rf.frf, cpu->mem_ops.read_memory, cpu->mem_ops.write_memory, (get_tlb(DATA_USER_READ) + offset), (get_tlb(DATA_USER_WRITE) + offset), (get_tlb(MIXED_TLB) + offset), (get_tlb(IO_TLB) + offset), user_mode);
		else
			ret = pfunc(cpu->dyncom_engine->RAM, cpu->rf.grf, cpu->rf.srf, cpu->rf.frf, cpu->mem_ops.read_memory, cpu->mem_ops.write_memory, (get_tlb(DATA_KERNEL_READ) + offset), get_tlb(DATA_KERNEL_WRITE) + offset, (get_tlb(MIXED_TLB) + offset), get_tlb(IO_TLB) + offset, user_mode);

		if (ret != JIT_RETURN_FUNCNOTFOUND)
			return ret;
	}
}
/**
 * @brief Clear the function:address map and free the memory of all the native code function.
 *
 * @param cpu CPU core structure
 */
void
cpu_flush(cpu_t *cpu)
{
	//cpu->dyncom_engine->exec_engine->freeMachineCodeForFunction(cpu->dyncom_engine->cur_func);
	//cpu->dyncom_engine->cur_func->eraseFromParent();

	cpu->dyncom_engine->functions = 0;
	
	funcbb_map::const_reverse_iterator i = cpu->dyncom_engine->func_bb.rbegin();
	for(; i != cpu->dyncom_engine->func_bb.rend(); i++){
		cpu->dyncom_engine->exec_engine->freeMachineCodeForFunction(i->first);
		i->first->eraseFromParent();
	}
	// reset bb caching mapping
	cpu->dyncom_engine->func_bb.clear();

//	delete cpu->dyncom_engine->mod;
//	cpu->dyncom_engine->mod = NULL;
}
/**
 * @brief print statistics,will be invoked when benchmark exits.
 *
 * @param cpu CPU core structure
 */
void
cpu_print_statistics(cpu_t *cpu)
{
	//printf("icounter = %8lld\n", REG(SR(ICOUNTER)));
	printf("tag = %8lld ms\n", cpu->timer_total[TIMER_TAG]);
	printf("fe  = %8lld ms\n", cpu->timer_total[TIMER_FE]);
	printf("be  = %8lld ms\n", cpu->timer_total[TIMER_BE]);
	printf("run = %8lld ms\n", cpu->timer_total[TIMER_RUN]);
	printf("opt = %8lld ms\n", cpu->timer_total[TIMER_OPT]);
}

extern "C" void debug_output(cpu_t* cpu){
	if(cpu->debug_func != NULL)
		cpu->debug_func(cpu);
}; 

typedef llvm::ArrayRef<llvm::Type*> TypeArray;
/* 
 * init the global functions.
 * By default the first callout function is debug function
 */

static void debug_func_init(cpu_t *cpu){
	//types
	//std::vector<const Type*> type_func_debug_args;
	std::vector<llvm::Type*> type_func_debug_args;
	PointerType *type_intptr = PointerType::get(cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX()), 0);
	type_func_debug_args.push_back(type_intptr);	/* intptr *cpu */
	//llvm::ArrayRef<const Type*> func_args = llvm::ArrayRef<const Type*>(type_func_debug_args);
	FunctionType *type_func_debug_callout = FunctionType::get(
		Type::getInt32Ty(cpu->dyncom_engine->mod->getContext()),	//return
		TypeArray(type_func_debug_args),	/* Params */
		false);		      	/* isVarArg */
	Constant *debug_const = cpu->dyncom_engine->mod->getOrInsertFunction("debug_output",	//function name
		type_func_debug_callout);	//return
	if(debug_const == NULL)
		fprintf(stderr, "Error:cannot insert function:debug.\n");
	Function *debug_func = cast<Function>(debug_const);
	debug_func->setCallingConv(CallingConv::C);
	cpu->dyncom_engine->ptr_arch_func[0] = debug_func;
}

extern "C" int syscall_func(cpu_t *cpu, uint32_t num){
	if(cpu->syscall_func != NULL)
		cpu->syscall_func(cpu, num);

	return No_exp;
};
static void syscall_func_init(cpu_t *cpu){
	//types
	std::vector<llvm::Type*> type_func_syscall_args;
	PointerType *type_intptr = PointerType::get(cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX()), 0);
	IntegerType *type_i32 = IntegerType::get(_CTX(), 32);
	type_func_syscall_args.push_back(type_intptr);	/* intptr *cpu */
	type_func_syscall_args.push_back(type_i32);	/* unsinged int */
	FunctionType *type_func_syscall_callout = FunctionType::get(
		Type::getVoidTy(cpu->dyncom_engine->mod->getContext()),	//return
		TypeArray(type_func_syscall_args),	/* Params */
		false);		      	/* isVarArg */
	Constant *syscall_const = cpu->dyncom_engine->mod->getOrInsertFunction("syscall_func",	//function name
		type_func_syscall_callout);	//return
	if(syscall_const == NULL)
		fprintf(stderr, "Error:cannot insert function:syscall.\n");
	Function *syscall_func = cast<Function>(syscall_const);
	syscall_func->setCallingConv(CallingConv::C);
	cpu->dyncom_engine->ptr_arch_func[1] = syscall_func;
}

/**
 * @brief Callout functions
 *
 * @param cpu
 * @param index
 *
 * @return 
 */
extern "C" void dyncom_callout(cpu_t *cpu, uint32_t index)
{
	if(index > MAX_ARCH_FUNC_NUM || index < 0){
		skyeye_log(Error_log, __func__, "In %s Callout function %d not exsit.\n", __func__, index);
		skyeye_exit(0);
	}
	((callout)cpu->dyncom_engine->arch_func[index])(cpu);
}
extern "C" void dyncom_callout1(cpu_t *cpu, uint32_t index, uint32_t arg1)
{
	if(index > MAX_ARCH_FUNC_NUM || index < 0){
		skyeye_log(Error_log, __func__, "In %s Callout function %d not exsit.\n", __func__, index);
		skyeye_exit(0);
	}
	((callout1)cpu->dyncom_engine->arch_func[index])(cpu, arg1);
}
extern "C" void dyncom_callout2(cpu_t *cpu, uint32_t index, uint32_t arg1, uint32_t arg2)
{
	if(index > MAX_ARCH_FUNC_NUM || index < 0){
		skyeye_log(Error_log, __func__, "In %s Callout function %d not exsit.\n", __func__, index);
		skyeye_exit(0);
	}
	((callout2)cpu->dyncom_engine->arch_func[index])(cpu, arg1, arg2);
}
extern "C" void dyncom_callout3(cpu_t *cpu, uint32_t index, uint64_t arg1, uint32_t arg2, uint32_t arg3)
{
	if(index > MAX_ARCH_FUNC_NUM || index < 0){
		skyeye_log(Error_log, __func__, "In %s Callout function %d not exsit.\n", __func__, index);
		skyeye_exit(0);
	}
	((callout3)cpu->dyncom_engine->arch_func[index])(cpu, arg1, arg2, arg3);
}
extern "C" void dyncom_callout4(cpu_t *cpu, uint32_t index, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4)
{
	if(index > MAX_ARCH_FUNC_NUM || index < 0){
		skyeye_log(Error_log, __func__, "In %s Callout function %d not exsit.\n", __func__, index);
		skyeye_exit(0);
	}
	((callout4)cpu->dyncom_engine->arch_func[index])(cpu, arg1, arg2, arg3, arg4);
}
