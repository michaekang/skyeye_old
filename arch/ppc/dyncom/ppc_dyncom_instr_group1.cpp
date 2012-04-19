/* Copyright (C) 
* 2012 - Michael.Kang blackfin.kang@gmail.com
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
* @file ppc_dyncom_instr_group1.cpp
* @brief The instruction translation for powerpc group1
* @author Michael.Kang blackfin.kang@gmail.com
* @version 78.77
* @date 2012-02-10
*/

#include "debug.h"
#include "tracers.h"
#include "ppc_dyncom_dec.h"
#include "ppc_exc.h"
#include "ppc_cpu.h"
#include "ppc_dyncom_alu.h"
#include "ppc_dyncom_run.h"
#include "ppc_tools.h"
#include "ppc_mmu.h"

#include "llvm/Instructions.h"
#include <dyncom/dyncom_llvm.h>
#include <dyncom/frontend.h>
#include "dyncom/basicblock.h"
#include "skyeye.h"
#include "skyeye_types.h"

#include "ppc_dyncom_debug.h"
static uint32 ppc_cmp_and_mask[8] = {
	0xfffffff0,
	0xffffff0f,
	0xfffff0ff,
	0xffff0fff,
	0xfff0ffff,
	0xff0fffff,
	0xf0ffffff,
	0x0fffffff,
};
/*
 *	bcctrx		Branch Conditional to Count Register
 *	.438
 */
int opc_bcctrx_tag(cpu_t *cpu, uint32_t instr, addr_t phys_pc, tag_t *tag, addr_t *new_pc, addr_t *next_pc){
	*tag = TAG_COND_BRANCH;
	*new_pc = NEW_PC_NONE;
	*tag |= TAG_STOP;
	return PPC_INSN_SIZE;
}
Value* opc_bcctrx_translate_cond(cpu_t *cpu, uint32_t instr, BasicBlock *bb){
	uint32 BO, BI, BD;
	PPC_OPC_TEMPL_XL(instr, BO, BI, BD);
	PPC_OPC_ASSERT(BD==0);
	PPC_OPC_ASSERT(!(BO & 2));     
	bool_t bo8 = (bool_t)((BO & 8)?1:0);
	Value* cr_value = SELECT(ICMP_NE(AND(RSPR(CR_REGNUM), CONST(1<<(31-BI))), CONST(0)), CONST(1), CONST(0));
	return OR(ICMP_NE(AND(CONST(BO), CONST(16)), CONST(0)), LOG_NOT(XOR(cr_value, CONST(bo8))));
}
static int opc_bcctrx_translate(cpu_t *cpu, uint32_t instr, BasicBlock *bb){
	if (instr & PPC_OPC_LK) {
		if(is_user_mode(cpu))
			LETS(LR_REGNUM, ADD(RSPR(PHYS_PC_REGNUM), CONST(4)));
		else
			LETS(LR_REGNUM, ADD(RSPR(PC_REGNUM), CONST(4)));
	}
	//arch_store(AND(RSPR(CTR_REGNUM), CONST(0xfffffffc)), cpu->ptr_PHYS_PC, bb);
	LETS(PHYS_PC_REGNUM, AND(RSPR(CTR_REGNUM), CONST(0xfffffffc)));
	if(!is_user_mode(cpu))
		//arch_store(AND(RSPR(CTR_REGNUM), CONST(0xfffffffc)), cpu->ptr_PC, bb);
		LETS(PC_REGNUM, AND(RSPR(CTR_REGNUM), CONST(0xfffffffc)));

	return No_exp;
}
/*
 *	bclrx		Branch Conditional to Link Register
 *	.440
 */
int opc_bclrx_tag(cpu_t *cpu, uint32_t instr, addr_t phys_pc, tag_t *tag, addr_t *new_pc, addr_t *next_pc){
	uint32 BO, BI, BD;
	PPC_OPC_TEMPL_XL(instr, BO, BI, BD);
	/* if BO is 1z1zz, then branch always */
	if((BO & 0x14) == 0x14 & is_user_mode(cpu))
		*tag = TAG_RET;
	else{
		*tag = TAG_COND_BRANCH;
		*new_pc = NEW_PC_NONE;
		*tag |= TAG_STOP;
	}
	return PPC_INSN_SIZE;
}
Value* opc_bclrx_translate_cond(cpu_t *cpu, uint32_t instr, BasicBlock *bb){
	uint32 BO, BI, BD;
	PPC_OPC_TEMPL_XL(instr, BO, BI, BD);
	PPC_OPC_ASSERT(BD==0);
	if (!(BO & 4)) {
		LETS(CTR_REGNUM, SUB(RSPR(CTR_REGNUM), CONST(1)));
	}
	bool_t bo2 = ((BO & 2)?True:False);
	bool_t bo8 = ((BO & 8)?True:False);
	Value* cr_value = SELECT(ICMP_NE(AND(RSPR(CR_REGNUM), CONST(1<<(31-BI))), CONST(0)), CONST(1), CONST(0));
	Value * tmp1 = ICMP_NE(AND(CONST(BO), CONST(4)), CONST(0));
	Value * tmp2 = XOR(ICMP_NE(RSPR(CTR_REGNUM), CONST(0)), ICMP_NE(CONST(bo2), CONST(0)));
	Value * tmp3 = ICMP_NE(AND(CONST(BO), CONST(16)), CONST(0));
	Value * tmp4 = ICMP_EQ(XOR(cr_value, CONST(bo8)), CONST(0));
	return AND(OR(tmp1, tmp2), OR(tmp3, tmp4));
}
static int opc_bclrx_translate(cpu_t *cpu, uint32_t instr, BasicBlock *bb){
	Value *tmp = AND(RSPR(LR_REGNUM), CONST(0xfffffffc));
	if (instr & PPC_OPC_LK) {
		if(is_user_mode(cpu))
			LETS(LR_REGNUM, ADD(RSPR(PHYS_PC_REGNUM), CONST(4)));
		else
			LETS(LR_REGNUM, ADD(RSPR(PC_REGNUM), CONST(4)));
	}
	//arch_store(tmp, cpu->ptr_PHYS_PC, bb);
	LETS(PHYS_PC_REGNUM, tmp);
	if(!is_user_mode(cpu))
		//arch_store(tmp, cpu->ptr_PC, bb);
		LETS(PC_REGNUM, tmp);

	return No_exp;
}
/*
 *	isync		Instruction Synchronize
 *	.520
 */
static int opc_isync_translate(cpu_t *cpu, uint32_t instr, BasicBlock *bb){
	// NO-OP
	return No_exp;
}
/*
 *	rfi		Return from Interrupt
 *	.607
 */
int opc_rfi_tag(cpu_t *cpu, uint32_t instr, addr_t phys_pc, tag_t *tag, addr_t *new_pc, addr_t *next_pc){
	*tag = TAG_BRANCH;
	*tag |= TAG_EXCEPTION;
	*new_pc = NEW_PC_NONE;
	return PPC_INSN_SIZE;
}
static int opc_rfi_translate(cpu_t *cpu, uint32_t instr, BasicBlock *bb){
	Value *old_pc = RSPR(PC_REGNUM);
	Value *cond = ICMP_NE(AND(RSPR(MSR_REGNUM), CONST(MSR_PR)), CONST(0));
	arch_ppc_dyncom_exception(cpu, bb, cond, PPC_EXC_PROGRAM, PPC_EXC_PROGRAM_PRIV, 0);
	ppc_dyncom_set_msr(cpu, bb, AND(RSPR(SRR_REGNUM + 1), CONST(MSR_RFI_SAVE_MASK)), cond);
	LETS(PC_REGNUM, SELECT(cond, old_pc, AND(RSPR(SRR_REGNUM), CONST(0xfffffffc))));
//	LETS(TMP_EFFECTIVE_ADDR_REGNUM, AND(RSPR(SRR_REGNUM), CONST(0xfffffffc)));
//	arch_ppc_dyncom_effective_to_physical(cpu, bb, PPC_MMU_CODE);
//	LETS(PC_REGNUM, SELECT(cond, RSPR(PC_REGNUM), RSPR(TMP_PHYSICAL_ADDR_REGNUM)));
	//should update page base
	return No_exp;
}
/*
 *	mcrf		Move Condition Register Field
 *	.561
 */
static int opc_mcrf_translate(cpu_t *cpu, uint32_t instr, BasicBlock *bb){
	uint32 crD, crS, bla;
	PPC_OPC_TEMPL_X(instr, crD, crS, bla);
	// FIXME: bla == 0
	crD >>= 2;
	crS >>= 2;
	crD = 7-crD;
	crS = 7-crS;
	Value *c_v = AND(LSHR(RSPR(CR_REGNUM), CONST(crS * 4)), CONST(0xf)); 
	LETS(CR_REGNUM, AND(RSPR(CR_REGNUM), CONST(ppc_cmp_and_mask[crD])));
	LETS(CR_REGNUM, OR(RSPR(CR_REGNUM), SHL(c_v, CONST(crD * 4))));

	return No_exp;
}
/*
 *	cror		Condition Register OR
 *	.453
 */
static int opc_cror_translate(cpu_t *cpu, uint32_t instr, BasicBlock *bb){
	int crD, crA, crB;
	PPC_OPC_TEMPL_X(instr, crD, crA, crB);
	uint32 t = (1<<(31-crA)) | (1<<(31-crB));
	LETS(CR_REGNUM, SELECT(
				ICMP_NE(AND(RSPR(CR_REGNUM), CONST(t)), CONST(0)),
				OR(RSPR(CR_REGNUM), CONST(1<<(31-crD))),
				AND(RSPR(CR_REGNUM), CONST(~(1<<(31-crD))))
				));

	return No_exp;
}
/*
 *	crnor		Condition Register NOR
 *	.452
 */
static int opc_crnor_translate(cpu_t *cpu, uint32_t instr, BasicBlock *bb)
{
	int crD, crA, crB;
	PPC_OPC_TEMPL_X(instr, crD, crA, crB);
	uint32 t = (1<<(31-crA)) | (1<<(31-crB));
	Value *cond = ICMP_NE(AND(RSPR(CR_REGNUM), CONST(t)), CONST(0));
	LETS(CR_REGNUM,SELECT(cond,
				AND(RSPR(CR_REGNUM), CONST(~(1<<(31-crD)))),
				OR(RSPR(CR_REGNUM), CONST(1<<(31-crD)))
				));

	return No_exp;
}
/*
 *	crxor		Condition Register XOR
 *	.448
 */
static int opc_crxor_translate(cpu_t *cpu, uint32_t instr, BasicBlock *bb)
{
	int crD, crA, crB;
	PPC_OPC_TEMPL_X(instr, crD, crA, crB);
	Value *cr_v = RSPR(CR_REGNUM);
	Value *tmp_a = CONST(1<<(31-crA));
	Value *tmp_b = CONST(1<<(31-crB));
	Value *tmp_c = CONST(1<<(31-crD));
	Value *tmp_not_c = CONST(~(1<<(31-crD)));
	Value *cond = OR(
			AND(ICMP_EQ(AND(cr_v, tmp_a), CONST(0)), ICMP_NE(AND(cr_v, tmp_b), CONST(0))),
			AND(ICMP_NE(AND(cr_v, tmp_a), CONST(0)), ICMP_EQ(AND(cr_v, tmp_b), CONST(0)))
			);
	LETS(CR_REGNUM, SELECT(cond, OR(cr_v, tmp_c), AND(cr_v, tmp_not_c)));

	return No_exp;
}
/* Interfaces */
ppc_opc_func_t ppc_opc_crnor_func = {
	opc_default_tag,
	opc_crnor_translate,
	opc_invalid_translate_cond,
};
ppc_opc_func_t ppc_opc_crandc_func;
ppc_opc_func_t ppc_opc_crxor_func = {
	opc_default_tag,
	opc_crxor_translate,
	opc_invalid_translate_cond,
};
ppc_opc_func_t ppc_opc_crnand_func;
ppc_opc_func_t ppc_opc_crand_func;
ppc_opc_func_t ppc_opc_creqv_func;
ppc_opc_func_t ppc_opc_crorc_func;
ppc_opc_func_t ppc_opc_cror_func = {
	opc_default_tag,
	opc_cror_translate,
	opc_invalid_translate_cond,
};

ppc_opc_func_t ppc_opc_bcctrx_func = {
	opc_bcctrx_tag,
	opc_bcctrx_translate,
	opc_bcctrx_translate_cond,
};
ppc_opc_func_t ppc_opc_bclrx_func = {
	opc_bclrx_tag,
	opc_bclrx_translate,
	opc_bclrx_translate_cond,
};
ppc_opc_func_t ppc_opc_mcrf_func = {
	opc_default_tag,
	opc_mcrf_translate,
	opc_invalid_translate_cond,
};
ppc_opc_func_t ppc_opc_rfi_func = {
	opc_rfi_tag,
	opc_rfi_translate,
	opc_invalid_translate_cond,
};
ppc_opc_func_t ppc_opc_isync_func = {
	opc_default_tag,
	opc_isync_translate,
	opc_invalid_translate_cond,
};
