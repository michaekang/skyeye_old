/**
 * @file tag.cpp
 *
 * Do a depth search of all reachable code and associate
 * every reachable instruction with flags that indicate
 * instruction type (branch,call,ret, ...), flags
 * (conditional, ...) and code flow information (branch
 * target, ...)
 *
 * @author OS Center,TsingHua University (Ported from libcpu)
 * @date 11/11/2010
 */
#include "skyeye_dyncom.h"
#include "skyeye_mm.h"
#include "dyncom/tag.h"
#include "dyncom/defines.h"
#include "dyncom/basicblock.h"
#include "sha1.h"
#include <vector>
using namespace std;
/*
 * TODO: on architectures with constant instruction sizes,
 * this shouldn't waste extra tag data for every byte of
 * code memory, but have one tag per instruction location.
 */

static uint32_t block_entry;

static const char *
get_temp_dir()
{
#ifdef _WIN32
	static char pathname[MAX_PATH];
	if (GetTempPathA(sizeof(pathname), pathname))
		return pathname;
#endif
	return "/tmp/";
}
static bool
is_tag_level2_table_allocated(cpu_t *cpu, addr_t addr)
{
	uint32_t level1_offset = TAG_LEVEL1_OFFSET(addr);
	return cpu->dyncom_engine->tag_table[level1_offset];
}

static bool
is_tag_level3_table_allocated(cpu_t *cpu, addr_t addr)
{
	uint32_t level1_offset = TAG_LEVEL1_OFFSET(addr);
	uint32_t level2_offset = TAG_LEVEL2_OFFSET(addr);
	return cpu->dyncom_engine->tag_table[level1_offset][level2_offset];
}

/**
 * @brief initialize tag level2 table
 *
 * @param cpu CPU core structure
 * @param addr address to tag
 */
static void
init_tag_level2_table(cpu_t *cpu, addr_t addr)
{
	tag_t **tag = (tag_t**)skyeye_mm_zero(TAG_LEVEL2_TABLE_SIZE * sizeof(tag_t *));
	memset(tag, 0, TAG_LEVEL2_TABLE_SIZE * sizeof(tag_t *));

	uint32_t level1_offset = TAG_LEVEL1_OFFSET(addr);
	cpu->dyncom_engine->tag_table[level1_offset] = tag;
}

/**
 * @brief initialize tag level3 table
 *
 * @param cpu CPU core structure
 * @param addr address to tag
 */
static void
init_tag_level3_table(cpu_t *cpu, addr_t addr)
{
	addr_t nitems, i;

	nitems = TAG_LEVEL3_TABLE_SIZE;

	tag_t* tag = (tag_t*)skyeye_mm_zero(nitems * sizeof(tag_t) * 2);
	for (i = 0; i < nitems; i++) {
		tag[i] = TAG_UNKNOWN;
		tag[i + TAG_LEVEL3_TABLE_SIZE] = 0;
	}

	uint32_t level1_offset = TAG_LEVEL1_OFFSET(addr);
	uint32_t level2_offset = TAG_LEVEL2_OFFSET(addr);
	cpu->dyncom_engine->tag_table[level1_offset][level2_offset] = tag;
}

/**
 * @brief check integrity of tag array memory. Allocate memory on demand.
 *
 * @param cpu CPU core structure
 * @param addr address to tag
 */
static void
check_tag_memory_integrity(cpu_t *cpu, addr_t addr)
{
	if (!is_tag_level2_table_allocated(cpu, addr)) {
		init_tag_level2_table(cpu, addr);
	}
	if (!is_tag_level3_table_allocated(cpu, addr)) {
		init_tag_level3_table(cpu, addr);
	}
}

/* check instruction at the address is translated or not. */
bool
is_translated_code(cpu_t *cpu, addr_t addr)
{
	if (get_tag(cpu, addr) & TAG_TRANSLATED) {
		return true;
	} else
		return false;
}

/* check instruction at the address is translated or not. */
bool
is_translated_entry(cpu_t *cpu, addr_t addr)
{
	if (get_tag(cpu, addr) & (TAG_TRANSLATED|TAG_ENTRY)) {
		return true;
	} else
		return false;
}


/* check instruction at the address is translated or not. */
bool
is_fast_interp_code(cpu_t *cpu, addr_t addr)
{
	if (get_tag(cpu, addr) & TAG_FAST_INTERP) {
		return true;
	} else
		return false;
}

#if 0
/* In os simulation ,tag depth is 1, so we can find out the first instruction of jit function
   by checking tag attribute backward in tag table. If tag has TAG_ENTRY, it is start of jit.*/
addr_t find_bb_start(cpu_t *cpu, addr_t addr)
{
	uint32_t level1_offset = TAG_LEVEL1_OFFSET(addr);
	uint32_t level2_offset = TAG_LEVEL2_OFFSET(addr);
	uint32_t level3_offset = TAG_LEVEL3_OFFSET(addr);
	tag_t tag = cpu->dyncom_engine->tag_table[level1_offset][level2_offset][level3_offset];
	int i = level3_offset;
	while (!(tag & TAG_ENTRY) && i) {
		i --;
		tag = cpu->dyncom_engine->tag_table[level1_offset][level2_offset][i];
	}
	return (level1_offset << TAG_LEVEL1_TABLE_SHIFT) |
		(level2_offset << TAG_LEVEL2_TABLE_SHIFT) |
		(i);
}
#endif
/**
 * @brief Determine an address is in code area or not
 *
 * @param cpu CPU core structure
 * @param a address
 *
 * @return true if in code area,false otherwise.
 */
bool
is_inside_code_area(cpu_t *cpu, addr_t a)
{
	//return (a >= cpu->dyncom_engine->code_start && a < cpu->dyncom_engine->code_end) | (a >= cpu->dyncom_engine->code1_start && a < a < cpu->dyncom_engine->code1_end);
	return (a >= cpu->dyncom_engine->code_start && a < cpu->dyncom_engine->code_end) | (a >= cpu->dyncom_engine->code1_start && a < cpu->dyncom_engine->code1_end);
}
/**
 * @brief Give a tag to an address
 *
 * @param cpu CPU core structure
 * @param a address to be tagged
 * @param t tag
 */
void
or_tag(cpu_t *cpu, addr_t a, tag_t t)
{
	tag_t old_tag = get_tag(cpu, a);
	set_tag(cpu, a, (old_tag|t));
	return;
}
void
xor_tag(cpu_t *cpu, addr_t a, tag_t t)
{
	/* NEW_PC_NONE is not a real address. Some branch/call address could not be known at translate-time*/
	if (a == NEW_PC_NONE) {
		return;
	}
	check_tag_memory_integrity(cpu, a);
	uint32_t level1_offset = TAG_LEVEL1_OFFSET(a);
	uint32_t level2_offset = TAG_LEVEL2_OFFSET(a);
	uint32_t level3_offset = TAG_LEVEL3_OFFSET(a);
	cpu->dyncom_engine->tag_table[level1_offset][level2_offset][level3_offset] &= ~t;
}
void clear_tag(cpu_t *cpu, addr_t a)
{
	addr_t nitems, i;
	/* NEW_PC_NONE is not a real address. Some branch/call address could not be known at translate-time*/
	if (a == NEW_PC_NONE) {
		return;
	}
	check_tag_memory_integrity(cpu, a);
	uint32_t level1_offset = TAG_LEVEL1_OFFSET(a);
	uint32_t level2_offset = TAG_LEVEL2_OFFSET(a);
	uint32_t level3_offset = TAG_LEVEL3_OFFSET(a);
	cpu->dyncom_engine->tag_table[level1_offset][level2_offset][level3_offset] = TAG_UNKNOWN;
	cpu->dyncom_engine->tag_table[level1_offset][level2_offset][level3_offset + TAG_LEVEL3_TABLE_SIZE] = 0;
}
/**
 * @brief Clear specific tags
 *
 * @param cpu CPU core structure
 * @param a address to be tagged
 * @param mask bits of tags to be cleared (a ~ will be applied)
 */
void selective_clear_tag(cpu_t *cpu, addr_t a, uint32_t mask)
{
	set_tag(cpu, a, (get_tag(cpu, a) & (~mask)));
}
void clear_tag_page(cpu_t *cpu, addr_t a)
{
	addr_t nitems, i;
	/* NEW_PC_NONE is not a real address. Some branch/call address could not be known at translate-time*/
	if (a == NEW_PC_NONE) {
		return;
	}
	check_tag_memory_integrity(cpu, a);
	uint32_t level1_offset = TAG_LEVEL1_OFFSET(a);
	uint32_t level2_offset = TAG_LEVEL2_OFFSET(a);
	for (i = 0; i < TAG_LEVEL3_TABLE_SIZE; i++) {
		//cpu->dyncom_engine->tag_table[level1_offset][level2_offset][i] &= TAG_TRANSLATED | TAG_ENTRY;
		cpu->dyncom_engine->tag_table[level1_offset][level2_offset][i] = TAG_UNKNOWN;
	}
}
void clear_tag_table(cpu_t *cpu)
{
	addr_t i, j, k;
	for (i = 0; i < TAG_LEVEL1_TABLE_SIZE; i++)
		if (cpu->dyncom_engine->tag_table[i])
			for (j = 0; j < TAG_LEVEL2_TABLE_SIZE; j++)
				if (cpu->dyncom_engine->tag_table[i][j])
					for (k = 0; k < TAG_LEVEL3_TABLE_SIZE; k++) {
						cpu->dyncom_engine->tag_table[i][j][k] = TAG_UNKNOWN;
						cpu->dyncom_engine->tag_table[i][j][k + TAG_LEVEL3_TABLE_SIZE] = 0;
					}
}


/**
* @brief Profiling function for BB
*
* @param cpu
* @param a
* @param inc the increasement for bb profiling counter
*
* @return 
*/
int
get_bb_prof(cpu_t *cpu, addr_t a, int inc)
{
        int counter;
        /* NEW_PC_NONE is not a real address. Some branch/call address could not be known at translate-time*/
        if (a == NEW_PC_NONE) {
                return TAG_UNKNOWN;
        }
        check_tag_memory_integrity(cpu, a);
        uint32_t level1_offset = TAG_LEVEL1_OFFSET(a);
        uint32_t level2_offset = TAG_LEVEL2_OFFSET(a);
        uint32_t level3_offset = TAG_LEVEL3_OFFSET(a);
        counter = cpu->dyncom_engine->tag_table[level1_offset][level2_offset][level3_offset + TAG_LEVEL3_TABLE_SIZE] + inc;
        cpu->dyncom_engine->tag_table[level1_offset][level2_offset][level3_offset + TAG_LEVEL3_TABLE_SIZE] = counter;
        return counter;
}

/* access functions */
/**
 * @brief Get the tag of an address 
 *
 * @param cpu CPU core structure
 * @param a address
 *
 * @return tag of the address
 */
tag_t
get_tag(cpu_t *cpu, addr_t a)
{
	/* NEW_PC_NONE is not a real address. Some branch/call address could not be known at translate-time*/
	if (a == NEW_PC_NONE) {
		return TAG_UNKNOWN;
	}
	check_tag_memory_integrity(cpu, a);
	uint32_t level1_offset = TAG_LEVEL1_OFFSET(a);
	uint32_t level2_offset = TAG_LEVEL2_OFFSET(a);
	uint32_t level3_offset = TAG_LEVEL3_OFFSET(a);
	return cpu->dyncom_engine->tag_table[level1_offset][level2_offset][level3_offset];
}

void
set_tag(cpu_t *cpu, addr_t a, tag_t tag)
{
	/* NEW_PC_NONE is not a real address. Some branch/call address could not be known at translate-time*/
	if (a == NEW_PC_NONE) {
		return;
	}
	check_tag_memory_integrity(cpu, a);
	uint32_t level1_offset = TAG_LEVEL1_OFFSET(a);
	uint32_t level2_offset = TAG_LEVEL2_OFFSET(a);
	uint32_t level3_offset = TAG_LEVEL3_OFFSET(a);
	cpu->dyncom_engine->tag_table[level1_offset][level2_offset][level3_offset] = tag;
	return;
}

#if 0
/**
 * @brief Get the tag of an address and add one to its execution 
 *        count
 *
 * @param cpu CPU core structure
 * @param a address 
 * @param counter pointer to counter 
 *
 * @return tag of the address
 */
tag_t
check_tag_execution(cpu_t *cpu, addr_t a, uint32_t *counter, uint32_t *entry)
{
	/* NEW_PC_NONE is not a real address. Some branch/call address could not be known at translate-time*/
	if (a == NEW_PC_NONE) {
		return TAG_UNKNOWN;
	}
	check_tag_memory_integrity(cpu, a);
	uint32_t level1_offset = TAG_LEVEL1_OFFSET(a);
	uint32_t level2_offset = TAG_LEVEL2_OFFSET(a);
	uint32_t level3_offset = TAG_LEVEL3_OFFSET(a);
	uint32_t tag = cpu->dyncom_engine->tag_table[level1_offset][level2_offset][level3_offset];
	if (tag & TAG_ENTRY) {
		*counter = cpu->dyncom_engine->tag_table[level1_offset][level2_offset][level3_offset + TAG_LEVEL3_TABLE_SIZE]++;
		return tag;
	} else if ((tag & TAG_CODE) && (entry == 0)) { /* condition on entry is to avoid infinite recursion */
		// uncomment if execution counter raises for each basic block instruction executed
		#if 0
		*entry = cpu->dyncom_engine->tag_table[level1_offset][level2_offset][level3_offset + TAG_LEVEL3_TABLE_SIZE];
		check_tag_execution(cpu, *entry, counter, entry);
		#endif
	}
	return tag;
}
#endif
/**
 * @brief Determine an address is code or not
 *
 * @param cpu CPU core structure
 * @param a address
 *
 * @return true if is code,false otherwise
 */
bool
is_code(cpu_t *cpu, addr_t a)
{
	return !!(get_tag(cpu, a) & TAG_CODE);
}
/**
 * @brief Determine an address is translated or not
 *
 * @param cpu CPU core structure
 * @param a address
 *
 * @return true if is translated,false otherwise
 */
bool
is_translated(cpu_t *cpu, addr_t a)
{
	return (get_tag(cpu, a) & TAG_TRANSLATED);
}

extern void disasm_instr(cpu_t *cpu, addr_t pc);

static void save_startbb_addr(cpu_t *cpu, addr_t pc){
	if (is_start_of_basicblock(cpu, pc) || (get_tag(cpu, pc) & TAG_AFTER_NEW_BB)){
		int cur_pos;
		cur_pos = cpu->dyncom_engine->cur_tagging_pos;
		vector<addr_t>::iterator i = cpu->dyncom_engine->startbb[cur_pos].begin();
		for(; i < cpu->dyncom_engine->startbb[cur_pos].end(); i++){

			if(*i == pc)
				break;
		}
		if(i == cpu->dyncom_engine->startbb[cur_pos].end())
			cpu->dyncom_engine->startbb[cur_pos].push_back(pc);

	}
}

#define LIMIT_TAGGING_DFS 4
static void
tag_recursive(cpu_t *cpu, addr_t pc, int level)
{
	int bytes;
	tag_t tag;
	addr_t new_pc, next_pc;

	if ((cpu->dyncom_engine->flags_codegen & CPU_CODEGEN_TAG_LIMIT)
	    && (level == LIMIT_TAGGING_DFS))
		return;
	if ((cpu->dyncom_engine->flags_codegen & CPU_CODEGEN_TAG_LIMIT)
	    && (level == 0))
	{
		LOG("tag start at %x\n", pc);
		/* save tag start address */
		cpu->dyncom_engine->tag_start = pc;
	}
	for(;;) {
		if(!cpu->mem_ops.is_inside_page(cpu, pc) && !is_user_mode(cpu))
			return;
		/* the area never be executed */
		if(!is_fast_interp_code(cpu, pc))
			return;

		if (!is_inside_code_area(cpu, pc)){
			LOG("In %s pc = %x start = %x end = %x\n",
					__FUNCTION__, pc, cpu->dyncom_engine->code_start, cpu->dyncom_engine->code_end);
			return;
		}
		if (LOGGING) {
			LOG("%*s", level, "");
//			disasm_instr(cpu, pc);
		}

		/* clear tag when instruction re-transalted. */
		tag = get_tag(cpu, pc);
		cpu->dyncom_engine->func_size[cpu->dyncom_engine->functions]++;

		bytes = cpu->f.tag_instr(cpu, pc, &tag, &new_pc, &next_pc);
		/* temporary fix: in case the previous instr at pc had changed,
		   we remove instr dependant tags. They will be set again anyway */
		selective_clear_tag(cpu, pc, TAG_BRANCH | TAG_CONDITIONAL | TAG_RET | TAG_STOP | TAG_CONTINUE | TAG_TRAP | TAG_NEW_BB | TAG_END_PAGE);
		or_tag(cpu, pc, tag | TAG_CODE);
#if OPT_LOCAL_REGISTERS
#if 0
		if (is_inside_code_area(cpu, next_pc)){
			tag_t tmp_tag;
			addr_t tmp_newpc, tmp_nextpc;
			cpu->f.tag_instr(cpu, next_pc, &tmp_tag, &tmp_newpc, &tmp_nextpc);
			if(tmp_tag & TAG_SYSCALL){
				or_tag(cpu, pc, TAG_BEFORE_SYSCALL);
			}
			if(tag & TAG_SYSCALL){
				or_tag(cpu, next_pc, TAG_AFTER_SYSCALL);
			}
		}
#endif
#endif
		LOG("In %s, pc=0x%x, tag=0x%x\n", __FUNCTION__, pc, tag);
		if ((tag & TAG_NEW_BB) && !is_user_mode(cpu)) {
			or_tag(cpu, next_pc, TAG_AFTER_NEW_BB);
		}
		if (tag & (TAG_CONDITIONAL))
			or_tag(cpu, next_pc, TAG_AFTER_COND);

		if (tag & TAG_TRAP)	{
			/* regular trap - no code after it */
			if (!(cpu->dyncom_engine->flags_hint & (CPU_HINT_TRAP_RETURNS | CPU_HINT_TRAP_RETURNS_TWICE)))
				//return;
				break;
			/*
			 * client hints that a trap will likely return,
			 * so tag code after it (optimization for usermode
			 * code that makes syscalls)
			 */
			or_tag(cpu, next_pc, TAG_AFTER_TRAP);
			/*
			 * client hints that a trap will likely return
			 * - to the next instruction AND
			 * - to the instruction after that
			 * OpenBSD on M88K skips an instruction on a trap
			 * return if there was an error.
			 */
			if (cpu->dyncom_engine->flags_hint & CPU_HINT_TRAP_RETURNS_TWICE) {
				tag_t dummy1;
				addr_t next_pc2, dummy2;
				next_pc2 = next_pc + cpu->f.tag_instr(cpu, next_pc, &dummy1, &dummy2, &dummy2);
				or_tag(cpu, next_pc2, TAG_AFTER_TRAP);
				tag_recursive(cpu, next_pc2, level+1);
			}
		}

		if (tag & TAG_CALL) {
			/* tag subroutine, then continue with next instruction */
			or_tag(cpu, new_pc, TAG_SUBROUTINE);
			or_tag(cpu, next_pc, TAG_AFTER_CALL);
			tag_recursive(cpu, new_pc, level+1);
		}

		if (tag & (TAG_BRANCH)) {
			or_tag(cpu, new_pc, TAG_BRANCH_TARGET);
//			printf("new_pc : %x pc : %x\n", new_pc, pc);
			if (new_pc != NEW_PC_NONE) {
				new_pc = (pc & 0xfffff000) + (new_pc & 0xfff);
				tag_recursive(cpu, new_pc, level+1);
			}
			if (!(tag & (TAG_CONDITIONAL)))
				//return;
				break;
		}

		if (is_translated(cpu, next_pc)) {
			or_tag(cpu, pc, tag | TAG_STOP | TAG_LAST_INST);
			//return;
		}
		if(cpu->mem_ops.is_page_start(cpu, pc) && !is_user_mode(cpu))
			or_tag(cpu, pc, tag | TAG_START_PAGE);
		if(cpu->mem_ops.is_page_end(cpu, pc) && !is_user_mode(cpu)){
			LOG("In %s. TAG_END_PAGE for pc=0x%x\n", __FUNCTION__, pc);
			or_tag(cpu, pc, tag | TAG_STOP | TAG_END_PAGE);
			xor_tag(cpu, pc, TAG_CONTINUE);
			/* if the memory related insn is located at the end of page,
				check_mm needs PC to parse the instruction */
			if(tag & TAG_NEW_BB){
				LOG("In %s. TAG_NEED_PC for pc=0x%x\n", __FUNCTION__, pc);
				or_tag(cpu, pc, tag | TAG_NEED_PC);
			}
			break;
		}
		if ((tag & TAG_EXCEPTION) && !is_user_mode(cpu)) {
			or_tag(cpu, next_pc, TAG_AFTER_EXCEPTION);
			xor_tag(cpu, pc, TAG_CONTINUE);
			break;
		}

		if (tag & (TAG_RET | TAG_STOP))	/* execution ends here, the follwing location is not reached */
			//return;
			break;
		save_startbb_addr(cpu, pc);
		pc = next_pc;
		/* save tag end address */
	}
	save_startbb_addr(cpu, pc);
	cpu->dyncom_engine->tag_end = pc;
	LOG("tag end at %x\n", pc);
    LOG("next pc is %x\n", next_pc);
}
/**
 * @brief Start tag from current pc.
 *
 * @param cpu CPU core structure
 * @param pc current address start tagging(physics pc)
 */
void
tag_start(cpu_t *cpu, addr_t pc)
{
	cpu->dyncom_engine->tags_dirty = true;

	/* for singlestep, we don't need this */
	if (cpu->dyncom_engine->flags_debug & (CPU_DEBUG_SINGLESTEP | CPU_DEBUG_SINGLESTEP_BB))
		return;

	/* initialize data structure on demand */
	check_tag_memory_integrity(cpu, pc);

	LOG("starting tagging at $%02llx\n", (unsigned long long)pc);

	if (!(cpu->dyncom_engine->flags_codegen & CPU_CODEGEN_TAG_LIMIT)) {
		int i;
		if (cpu->dyncom_engine->file_entries) {
			for (i = 0; i < 4; i++)
				fputc((pc >> (i*8))&0xFF, cpu->dyncom_engine->file_entries);
			fflush(cpu->dyncom_engine->file_entries);
		}
	}

	block_entry = pc;
	
	or_tag(cpu, pc, TAG_ENTRY); /* client wants to enter the guest code here */

	tag_recursive(cpu, pc, 0);
}

static void
tag_iterative(cpu_t *cpu, vector<uint32_t> &trace, vector<uint32_t> &start_addr)
{
	int bytes;
	tag_t tag;
	addr_t new_pc, next_pc, pc;

	vector<uint32_t>::iterator it;
	for(it = trace.begin(); it != trace.end(); ++it) {
		pc = *it;
//		printf("in tag, pc : %x\n", pc);
		if(!cpu->mem_ops.is_inside_page(cpu, pc) && !is_user_mode(cpu))
			return;
		if (!is_inside_code_area(cpu, pc)){
			LOG("In %s pc = %x start = %x end = %x\n",
					__FUNCTION__, pc, cpu->dyncom_engine->code_start, cpu->dyncom_engine->code_end);
			return;
		}

		/* clear tag when instruction re-transalted. */
		tag = get_tag(cpu, pc);

		bytes = cpu->f.tag_instr(cpu, pc, &tag, &new_pc, &next_pc);
		/* temporary fix: in case the previous instr at pc had changed,
		   we remove instr dependant tags. They will be set again anyway */
		selective_clear_tag(cpu, pc, TAG_BRANCH | TAG_CONDITIONAL | TAG_RET | TAG_STOP | TAG_CONTINUE | TAG_TRAP | TAG_NEW_BB | TAG_END_PAGE);
		or_tag(cpu, pc, tag | TAG_CODE);
#ifdef OPT_LOCAL_REGISTERS
#if 0
		if (is_inside_code_area(cpu, next_pc)){
			tag_t tmp_tag;
			addr_t tmp_newpc, tmp_nextpc;
			cpu->f.tag_instr(cpu, next_pc, &tmp_tag, &tmp_newpc, &tmp_nextpc);
			if(tmp_tag & TAG_SYSCALL){
				or_tag(cpu, pc, TAG_BEFORE_SYSCALL);
			}
			if(tag & TAG_SYSCALL){
				or_tag(cpu, next_pc, TAG_AFTER_SYSCALL);
			}
		}
#endif
#endif
		LOG("In %s, pc=0x%x, tag=0x%x\n", __FUNCTION__, pc, tag);
		if ((tag & TAG_NEW_BB) && !is_user_mode(cpu)) {
//			or_tag(cpu, next_pc, TAG_AFTER_COND);
			or_tag(cpu, next_pc, TAG_AFTER_NEW_BB);
		}
		if (tag & (TAG_CONDITIONAL))
			or_tag(cpu, next_pc, TAG_AFTER_COND);

		if (tag & TAG_TRAP)	{
			/* regular trap - no code after it */
			if (!(cpu->dyncom_engine->flags_hint & (CPU_HINT_TRAP_RETURNS | CPU_HINT_TRAP_RETURNS_TWICE)))
				//return;
				continue;
			/*
			 * client hints that a trap will likely return,
			 * so tag code after it (optimization for usermode
			 * code that makes syscalls)
			 */
			or_tag(cpu, next_pc, TAG_AFTER_TRAP);
			/*
			 * client hints that a trap will likely return
			 * - to the next instruction AND
			 * - to the instruction after that
			 * OpenBSD on M88K skips an instruction on a trap
			 * return if there was an error.
			 */
		}

		if (tag & TAG_CALL) {
			/* tag subroutine, then continue with next instruction */
			or_tag(cpu, new_pc, TAG_SUBROUTINE);
			or_tag(cpu, next_pc, TAG_AFTER_CALL);
//			tag_recursive(cpu, new_pc, level+1);
		}

		if (tag & (TAG_BRANCH)) {
			or_tag(cpu, new_pc, TAG_BRANCH_TARGET);
//			printf("new_pc : %x pc : %x\n", new_pc, pc);
			if (new_pc != NEW_PC_NONE) {
				new_pc = (pc & 0xfffff000) + (new_pc & 0xfff);
//				tag_recursive(cpu, new_pc, level+1);
			}
			#if 0
			if (!(tag & (TAG_CONDITIONAL))) {
//				return;
				//break;
				printf("!cond branch\n");
				continue;
			}
			#endif
		}

		if (is_translated(cpu, next_pc)) {
			or_tag(cpu, pc, tag | TAG_STOP | TAG_LAST_INST);
			continue;
		}
		if(cpu->mem_ops.is_page_start(cpu, pc) && !is_user_mode(cpu))
			or_tag(cpu, pc, tag | TAG_START_PAGE);
		if(cpu->mem_ops.is_page_end(cpu, pc) && !is_user_mode(cpu)){
			LOG("In %s. TAG_END_PAGE for pc=0x%x\n", __FUNCTION__, pc);
			or_tag(cpu, pc, tag | TAG_STOP | TAG_END_PAGE);
			xor_tag(cpu, pc, TAG_CONTINUE);
			/* if the memory related insn is located at the end of page,
				check_mm needs PC to parse the instruction */
			if(tag & TAG_NEW_BB){
				LOG("In %s. TAG_NEED_PC for pc=0x%x\n", __FUNCTION__, pc);
				or_tag(cpu, pc, tag | TAG_NEED_PC);
			}
			continue;
		}
		if ((tag & TAG_EXCEPTION) && !is_user_mode(cpu)) {
			or_tag(cpu, next_pc, TAG_AFTER_EXCEPTION);
			xor_tag(cpu, pc, TAG_CONTINUE);
			continue;
		}
#if 1
		if (tag & (TAG_RET | TAG_STOP))	/* execution ends here, the follwing location is not reached */
//			return;
//			break;
			continue;
#endif
		save_startbb_addr(cpu, pc);
		pc = next_pc;
		/* save tag end address */
	}
#if 0
	if (it != trace.end()) {
		printf("NONO\n");
		exit(-1);
	}
#endif
	//save_startbb_addr(cpu, pc);
	#if 0
	for(it = start_addr.begin(); it != start_addr.end(); ++it) {
		pc = *it;
		or_tag(cpu, pc, TAG_ENTRY);
		save_startbb_addr(cpu, pc);
	}
	#endif
	#if 0
	it = start_addr.begin();
	pc = *it;
	or_tag(cpu, pc, TAG_ENTRY);
	save_startbb_addr(cpu, pc);
	#endif
	cpu->dyncom_engine->tag_end = pc;
	LOG("tag end at %x\n", pc);
	LOG("next pc is %x\n", next_pc);
}

void
tag_by_trace(cpu_t *cpu, vector<uint32_t> &trace, vector<uint32_t> &start_addr)
{
	uint32_t pc;
	pc = trace.front();

	cpu->dyncom_engine->tags_dirty = true;

	/* for singlestep, we don't need this */
	if (cpu->dyncom_engine->flags_debug & (CPU_DEBUG_SINGLESTEP | CPU_DEBUG_SINGLESTEP_BB))
		return;

	/* initialize data structure on demand */
	check_tag_memory_integrity(cpu, pc);

	LOG("starting tagging at $%02llx\n", (unsigned long long)pc);

	if (!(cpu->dyncom_engine->flags_codegen & CPU_CODEGEN_TAG_LIMIT)) {
		int i;
		if (cpu->dyncom_engine->file_entries) {
			for (i = 0; i < 4; i++)
				fputc((pc >> (i*8))&0xFF, cpu->dyncom_engine->file_entries);
			fflush(cpu->dyncom_engine->file_entries);
		}
	}

	block_entry = pc;
	
	or_tag(cpu, pc, TAG_ENTRY); /* client wants to enter the guest code here */
	vector<uint32_t>::iterator it;
	tag_iterative(cpu, trace, start_addr);
	#if 0
	if (start_addr.size()) {
		for(it = start_addr.begin(); it != start_addr.end(); ++it) {
			tag_recursive(cpu, *it, 0);
		}
	}
	#endif
}

