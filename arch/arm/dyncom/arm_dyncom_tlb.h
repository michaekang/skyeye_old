#ifndef __ARM_DYNCOM_TLB_H__
#define __ARM_DYNCOM_TLB_H__
#include <skyeye_dyncom.h>
#define TLB_SIZE 1024 * 1024
#define ASID_SIZE 255

uint32_t get_phys_page(ARMul_State* state, ARMword va);
inline void insert_tlb(ARMul_State* state, ARMword va, ARMword pa);
uint32_t** new_tlb(int line, int way);
void fini_tlb();
void invalidate_by_asid(ARMul_State *state, ARMword asid);
void invalidate_by_mva(cpu_t* cpu, ARMword va);
#endif
