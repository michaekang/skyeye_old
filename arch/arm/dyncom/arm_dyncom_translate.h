#ifndef __ARM_TRANS__
#define __ARM_TRANS__
#include "skyeye_dyncom.h"
#include "skyeye_obj.h"
#include "armdefs.h"
//#include "armemu.h"

#include "arm_regformat.h"
int arch_arm_tag_instr(cpu_t *cpu, addr_t pc, tag_t *tag, addr_t *new_pc, addr_t *next_pc);
Value *
arch_arm_translate_cond(cpu_t *cpu, addr_t pc, BasicBlock *bb);
int arch_arm_translate_instr(cpu_t *cpu, addr_t pc, BasicBlock *bb);
void arm_opc_func_init();
void arch_arm_flus_instr_category();

typedef std::map<addr_t, int> decoder_cache;
extern decoder_cache dc_map;


/**
* @brief The thumb bit location at the cpsr
*/
const int THUMB_BIT = 5;
static int get_instr_size(cpu_t* cpu){
	//arm_core_t* core = (arm_core_t*)(cpu->cpu_data->obj);
	//return (core->TFlag)?2:4;
	return is_thumb_func(cpu)? 2 : 4;
}
#ifndef INSTR_SIZE
#define INSTR_SIZE get_instr_size(cpu)
#endif

#ifdef __cplusplus
 extern "C" {
#endif

void update_cond_from_fpscr(cpu_t *cpu, uint32_t instr, BasicBlock *bb, addr_t pc);
#ifdef __cplusplus
}
#endif

#endif
