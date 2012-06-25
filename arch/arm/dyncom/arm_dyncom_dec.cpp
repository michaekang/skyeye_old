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
* @file arm_dyncom_dec.cpp
* @brief Some common utility for arm decoder
* @author Michael.Kang blackfin.kang@gmail.com
* @version 7849
* @date 2012-03-15
*/

#include "llvm/Instructions.h"
#include "llvm/Constants.h"
#include "arm_regformat.h"
#include "skyeye_dyncom.h"
#include "dyncom/dyncom_llvm.h"
#include "dyncom/frontend.h"
#include "arm_internal.h"
#include "arm_types.h"
#include "dyncom/tag.h"

#include "armdefs.h"
#include "arm_dyncom_run.h"
#include "arm_dyncom_dec.h"
#include "arm_dyncom_memory.h"
#include "arm_dyncom_translate.h"

using namespace llvm;
#if 0
#define BITS(a,b) ((instr >> (a)) & ((1 << (1+(b)-(a)))-1))
#define BIT(n) ((instr >> (n)) & 1)
#define BAD	do{printf("meet BAD at %s, instr is %x\n", __FUNCTION__, instr ); /*exit(0);*/}while(0);
#define ptr_N	cpu->ptr_N
#define ptr_Z	cpu->ptr_Z
#define ptr_C	cpu->ptr_C
#define ptr_V	cpu->ptr_V
#define ptr_I 	cpu->ptr_I
#define	ptr_CPSR cpu->ptr_gpr[16]

/* for MUL instructions */
/*xxxx xxxx xxxx 1111 xxxx xxxx xxxx xxxx */
#define RDHi ((instr >> 16) & 0xF)
/*xxxx xxxx xxxx xxxx 1111 xxxx xxxx xxxx */
#define RDLo ((instr >> 12) & 0xF)
/*xxxx xxxx xxxx 1111 xxxx xxxx xxxx xxxx */
#define MUL_RD ((instr >> 16) & 0xF)
/*xxxx xxxx xxxx xxxx 1111 xxxx xxxx xxxx */
#define MUL_RN ((instr >> 12) & 0xF)
/*xxxx xxxx xxxx xxxx xxxx 1111 xxxx xxxx */
#define RS ((instr >> 8) & 0xF)

/*xxxx xxxx xxxx xxxx 1111 xxxx xxxx xxxx */
#define RD ((instr >> 12) & 0xF)
/*xxxx xxxx xxxx 1111 xxxx xxxx xxxx xxxx */
#define RN ((instr >> 16) & 0xF)
/*xxxx xxxx xxxx xxxx xxxx xxxx xxxx 1111 */
#define RM (instr & 0xF)
#define BIT(n) ((instr >> (n)) & 1)
#define BITS(a,b) ((instr >> (a)) & ((1 << (1+(b)-(a)))-1))
/*xxxx xx1x xxxx xxxx xxxx xxxx xxxx xxxx */
#define I BIT(25)
/*xxxx xxxx xxx1 xxxx xxxx xxxx xxxx xxxx */
#define S BIT(20)

#define SHIFT BITS(5,6)
#define SHIFT_IMM BITS(7,11)
#define IMMH BITS(8,11)
#define IMML BITS(0,3)

#define LSPBIT  BIT(24)
#define LSUBIT  BIT(23)
#define LSBBIT  BIT(22)
#define LSWBIT  BIT(21)
#define LSLBIT  BIT(20)
#define LSSHBITS BITS(5,6)
#define OFFSET12 BITS(0,11)
#define SBIT  BIT(20)
#define DESTReg (BITS (12, 15))

/* they are in unused state, give a corrent value when using */
#define IS_V5E 0
#define IS_V5  0
#define IS_V6  0
#define LHSReg 0

/* temp define the using the pc reg need implement a flow */
#define STORE_CHECK_RD_PC	ADD(R(RD), CONST(8))
#endif
/*
*		LoadStore operations funcs relationship
* 			LoadStore
*          |		    |		    |
* WOrUBLoadStore     MisLoadStore	 LoadStoreM
*/

/* store a word to memory */
void StoreWord(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	#if 0
	Value* phys_addr = get_phys_addr(cpu, bb, addr, 0);
	if(RD == 15)
		arch_write_memory(cpu, bb, phys_addr, STORE_CHECK_RD_PC, 32);
	else
		arch_write_memory(cpu, bb, phys_addr, R(RD), 32);
	#endif
	if(RD == 15)
		memory_write(cpu, bb, addr, STORE_CHECK_RD_PC, 32);
	else
		memory_write(cpu, bb, addr, R(RD), 32);

	
	bb = cpu->dyncom_engine->bb;
}

/* store a half word to memory */
void StoreHWord(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	#if 0
	Value* phys_addr = get_phys_addr(cpu, bb, addr, 0);
	arch_write_memory(cpu, bb, phys_addr, R(RD), 16);
	#endif
	memory_write(cpu, bb, addr, R(RD), 16);
}

/* store a byte word to memory */
void StoreByte(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	#if 0
	Value* phys_addr = get_phys_addr(cpu, bb, addr, 0);
	arch_write_memory(cpu, bb, phys_addr, R(RD), 8);
	#endif
	memory_write(cpu, bb, addr, R(RD), 8);
}

/* store a double word to memory */
void StoreDWord(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	#if 0
	Value* phys_addr = get_phys_addr(cpu, bb, addr, 0);
	bb = cpu->dyncom_engine->bb;
	arch_write_memory(cpu, bb, phys_addr, R(RD), 32);
	#endif
	memory_write(cpu, bb, addr, R(RD), 32);
	bb = cpu->dyncom_engine->bb;

	#if 0
	phys_addr = get_phys_addr(cpu, bb, ADD(addr, CONST(4)), 0);
	bb = cpu->dyncom_engine->bb;
	arch_write_memory(cpu, bb, phys_addr, R(RD + 1),32);
	#endif
	memory_write(cpu, bb, ADD(addr, CONST(4)), R(RD + 1),32);
}

/* load a word from memory */
void LoadWord(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	#if 0
	//arch_arm_debug_print(cpu, bb, ZEXT64(addr), R(15), CONST(23));
	Value* phys_addr = get_phys_addr(cpu, bb, addr, 1);
	bb = cpu->dyncom_engine->bb;
	//arch_arm_debug_print(cpu, bb, ZEXT64(phys_addr), R(15), CONST(23));
	arch_read_memory(cpu, bb, phys_addr, 0, 32);
	#endif
	memory_read(cpu, bb, addr, 0, 32);
	bb = cpu->dyncom_engine->bb;
	Value *ret = new LoadInst(cpu->dyncom_engine->read_value, "", false, bb);
	//arch_arm_debug_print(cpu, bb, ZEXT64(phys_addr), R(15), CONST(24));
	if(RD == 15){
		STORE(TRUNC1(AND(ret, CONST(1))), ptr_T);
		LET(RD,AND(ret, CONST(0xFFFFFFFE)));
		/* SET_NEW_PAGE here */
		if (!cpu->is_user_mode) {
			Value *new_page_effec = AND(R(15), CONST(0xfffff000));
			new StoreInst(new_page_effec, cpu->ptr_CURRENT_PAGE_EFFEC, bb);	
		};
		LET(PHYS_PC, R(15));
	}
	else
		LET(RD, ret);
}

/* load a half word from memory */
void LoadHWord(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	#if 0
	Value* phys_addr = get_phys_addr(cpu, bb, addr, 1);
	bb = cpu->dyncom_engine->bb;
	arch_read_memory(cpu, bb, phys_addr, 0, 16);
	#endif
	memory_read(cpu, bb, addr, 0, 16);
	bb = cpu->dyncom_engine->bb;
	Value *ret = new LoadInst(cpu->dyncom_engine->read_value, "", false, bb);
	LET(RD,ret);
}

/* load a signed half word from memory */
void LoadSHWord(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	#if 0
	Value* phys_addr = get_phys_addr(cpu, bb, addr, 1);
	bb = cpu->dyncom_engine->bb;
	arch_read_memory(cpu, bb, phys_addr, 1, 16);
	#endif
	memory_read(cpu, bb, addr, 1, 16);
	bb = cpu->dyncom_engine->bb;
	Value *ret = new LoadInst(cpu->dyncom_engine->read_value, "", false, bb);
	LET(RD,ret);
}

/* load a byte from memory */
void LoadByte(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	#if 0
	Value* phys_addr = get_phys_addr(cpu, bb, addr, 1);
	bb = cpu->dyncom_engine->bb;
	arch_read_memory(cpu, bb, phys_addr, 0, 8);
	#endif
	memory_read(cpu, bb, addr, 0, 8);
	bb = cpu->dyncom_engine->bb;
	Value *ret = new LoadInst(cpu->dyncom_engine->read_value, "", false, bb);
	LET(RD,ret);
}

/* load a signed byte from memory */
void LoadSByte(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	#if 0
	Value* phys_addr = get_phys_addr(cpu, bb, addr, 1);
	bb = cpu->dyncom_engine->bb;
	arch_read_memory(cpu, bb, phys_addr, 1, 8);
	#endif
	memory_read(cpu, bb, addr, 1, 8);
	bb = cpu->dyncom_engine->bb;
	Value *ret = new LoadInst(cpu->dyncom_engine->read_value, "", false, bb);
	LET(RD,ret);
}

/* load a double word from memory */
void LoadDWord(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	#if 0
	Value* phys_addr = get_phys_addr(cpu, bb, addr, 1);
	bb = cpu->dyncom_engine->bb;
	arch_read_memory(cpu, bb, phys_addr, 0, 32);
	#endif
	memory_read(cpu, bb, addr, 0, 32);
	bb = cpu->dyncom_engine->bb;
	Value *ret = new LoadInst(cpu->dyncom_engine->read_value, "", false, bb);
	LET(RD,ret);
	#if 0
	phys_addr = get_phys_addr(cpu, bb, ADD(addr, CONST(4)), 1);
	bb = cpu->dyncom_engine->bb;
	arch_read_memory(cpu, bb, phys_addr, 0, 32);
	#endif
	memory_read(cpu, bb, ADD(addr, CONST(4)), 0, 32);
	bb = cpu->dyncom_engine->bb;
	ret = new LoadInst(cpu->dyncom_engine->read_value, "", false, bb);
	LET(RD+1,ret);
}

/* word or unsigned byte load operation, following arm doc */
void WOrUBLoad(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	if(LSBBIT)
		LoadByte(cpu, instr, bb, addr); // alex-ykl fix 2011-07-26 : was loading a signed byte
	else
		LoadWord(cpu, instr, bb, addr);
}

/* word or unsigned byte store operation, following arm doc */
void WOrUBStore(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	if(LSBBIT)
		StoreByte(cpu, instr, bb, addr);
	else
		StoreWord(cpu, instr, bb, addr);
}

/* word or unsigned byte load operation, following arm doc */
void WOrUBLoadStore(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	if(LSLBIT)
		WOrUBLoad(cpu, instr, bb, addr);
	else
		WOrUBStore(cpu, instr, bb, addr);
}

/* Miscellaneous load operations, following arm doc */
void MisLoad(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	switch (LSSHBITS){
		case 0:
			LoadByte(cpu,instr,bb,addr);
			break;
		case 1:
			LoadHWord(cpu,instr,bb,addr);
			break;
		case 2:
		{
			if (LSLBIT)
				LoadSByte(cpu,instr,bb,addr);
			else
				LoadDWord(cpu,instr,bb,addr);
		}
			break;
		case 3:
			LoadHWord(cpu,instr,bb,addr);
			break;
	}
}

/* Miscellaneous store operations, following arm doc */
void MisStore(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	switch (LSSHBITS){
		case 0:
			StoreByte(cpu,instr,bb,addr);
			break;
		case 1:
			StoreHWord(cpu,instr,bb,addr);
			break;
		case 2:
			LoadDWord(cpu,instr,bb,addr);
			break;
		case 3:
			StoreDWord(cpu,instr,bb,addr);
			break;
	}
}

/* Miscellaneous store load operation collecton, following arm doc */
void MisLoadStore(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr)
{
	if ((LSLBIT) || ((LSLBIT==0) && (LSSHBITS==0x2)))
		MisLoad(cpu,instr,bb,addr);
	else
		MisStore(cpu,instr,bb,addr);
}

/* Load multiple operation, following arm doc */
void LoadM(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr, Value* Rn)
{
	int i;
	Value *ret;
	Value *Addr = addr;
	int count = get_reg_count(instr);
	assert(count > 0);	
	count -= 1;
	#if 0
	Value* start_phys_page = get_phys_addr(cpu, bb, addr, 1);
	bb = cpu->dyncom_engine->bb;
	start_phys_page = AND(start_phys_page, CONST(0xFFFFF000));
	/* possible maximum address for memory access */
	Value* end_phys_page = get_phys_addr(cpu, bb, ADD(addr, CONST(count * 4)), 1);
	bb = cpu->dyncom_engine->bb;
	end_phys_page = AND(end_phys_page, CONST(0xFFFFF000));

	Value* start_virt_page = AND(addr, CONST(0xFFFFF000));
	#endif
	if (BITS(25, 27) == 4 && BIT(22) && BIT(20) && !BIT(15)) {
		/* LDM (2) user */
		for (i = 0; i < 13; i++) {
			if(BIT(i)){
				#if 0
				phys_addr1 = OR(start_phys_page, AND(Addr, CONST(0xFFF)));
				phys_addr2 = OR(end_phys_page, AND(Addr, CONST(0xFFF)));
				phys_addr = SELECT(ICMP_EQ(AND(Addr, CONST(0xFFFFF000)), start_virt_page), phys_addr1, phys_addr2);
				arch_read_memory(cpu, bb, phys_addr, 0, 32);
				#endif
				memory_read(cpu, bb, Addr, 0, 32);
				bb = cpu->dyncom_engine->bb;
				ret = new LoadInst(cpu->dyncom_engine->read_value, "", false, bb);
				//LOG("In %s, i=0x%x\n", __FUNCTION__, i);
				LET(i, ret);
				Addr = ADD(Addr, CONST(4));
			}
		}
		if (BIT(13)) {
			#if 0
			phys_addr1 = OR(start_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr2 = OR(end_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr = SELECT(ICMP_EQ(AND(Addr, CONST(0xFFFFF000)), start_virt_page), phys_addr1, phys_addr2);

			arch_read_memory(cpu, bb, phys_addr, 0, 32);
			#endif
			memory_read(cpu, bb, Addr, 0, 32);
			bb = cpu->dyncom_engine->bb;
			ret = new LoadInst(cpu->dyncom_engine->read_value, "", false, bb);
			LET(R13_USR, ret);
			Addr = ADD(Addr, CONST(4));
		}
		if (BIT(14)) {
			#if 0
			phys_addr1 = OR(start_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr2 = OR(end_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr = SELECT(ICMP_EQ(AND(Addr, CONST(0xFFFFF000)), start_virt_page), phys_addr1, phys_addr2);

			arch_read_memory(cpu, bb, phys_addr, 0, 32);
			#endif
			memory_read(cpu, bb, Addr, 0, 32);
			bb = cpu->dyncom_engine->bb;
			ret = new LoadInst(cpu->dyncom_engine->read_value, "", false, bb);
			LET(R14_USR, ret);
			Addr = ADD(Addr, CONST(4));
		}
		return;
	}
	for( i = 0; i < 16; i ++ ){
		if(BIT(i)){
			#if 0
			phys_addr1 = OR(start_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr2 = OR(end_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr = SELECT(ICMP_EQ(AND(Addr, CONST(0xFFFFF000)), start_virt_page), phys_addr1, phys_addr2);
			
			arch_read_memory(cpu, bb, phys_addr, 0, 32);
			#endif
			memory_read(cpu, bb, Addr, 0, 32);
			bb = cpu->dyncom_engine->bb;
			ret = new LoadInst(cpu->dyncom_engine->read_value, "", false, bb);
			//LOG("In %s, i=0x%x\n", __FUNCTION__, i);
			if(i == R15){
				STORE(TRUNC1(AND(ret, CONST(1))), ptr_T);
				LET(i, AND(ret, CONST(0xFFFFFFFE)));
			}
			else
				LET(i, ret);

			Addr = ADD(Addr, CONST(4));
		}
	}
}

/* temp define the using the pc reg need implement a flow */
#define STOREM_CHECK_PC ADD(R(15), CONST(8))
/* store multiple operation, following arm doc */
void StoreM(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr, Value* Rn)
{
	int i;
	Value *Addr = addr;

	int count = get_reg_count(instr);
	assert(count > 0);
	count -= 1;
	#if 0	
	//arch_arm_debug_print(cpu, bb, ZEXT64(addr), R(15), CONST(50));
	Value* start_phys_page = get_phys_addr(cpu, bb, addr, 0);
	bb = cpu->dyncom_engine->bb;
	start_phys_page = AND(start_phys_page, CONST(0xFFFFF000));
	//arch_arm_debug_print(cpu, bb, ZEXT64(start_phys_page), R(15), CONST(50));
	/* possible maximum address for memory access */
	Value* end_phys_page = get_phys_addr(cpu, bb, ADD(addr, CONST(count * 4)), 0);
	bb = cpu->dyncom_engine->bb;
	end_phys_page = AND(end_phys_page, CONST(0xFFFFF000));

	//arch_arm_debug_print(cpu, bb, ZEXT64(end_phys_page), R(15), CONST(50));
	Value* start_virt_page = AND(addr, CONST(0xFFFFF000));
	#endif
	/* Check if base register is in register list */
	if (BITS(25, 27) == 4 && BITS(20, 22) == 4) {
		for (i = 0; i < 13; i++) {
			if(BIT(i)){
				#if 0
				phys_addr1 = OR(start_phys_page, AND(Addr, CONST(0xFFF)));
				phys_addr2 = OR(end_phys_page, AND(Addr, CONST(0xFFF)));
				phys_addr = SELECT(ICMP_EQ(AND(Addr, CONST(0xFFFFF000)), start_virt_page), phys_addr1, phys_addr2);
				arch_write_memory(cpu, bb, phys_addr, R(i), 32);
				#endif
				memory_write(cpu, bb, Addr, R(i), 32);
				bb = cpu->dyncom_engine->bb;
				Addr = ADD(Addr, CONST(4));
			}
		}
		if (BIT(13)) {
			#if 0
			phys_addr1 = OR(start_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr2 = OR(end_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr = SELECT(ICMP_EQ(AND(Addr, CONST(0xFFFFF000)), start_virt_page), phys_addr1, phys_addr2);
			if(RN == 13)
				arch_write_memory(cpu, bb, phys_addr, Rn, 32);
			else
				arch_write_memory(cpu, bb, phys_addr, R(R13_USR), 32);
			#endif
			if(RN == 13)
				memory_write(cpu, bb, Addr, Rn, 32);
			else
				memory_write(cpu, bb, Addr, R(R13_USR), 32);
			
			bb = cpu->dyncom_engine->bb;
			Addr = ADD(Addr, CONST(4));
		}
		if (BIT(14)) {
			#if 0
			phys_addr1 = OR(start_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr2 = OR(end_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr = SELECT(ICMP_EQ(AND(Addr, CONST(0xFFFFF000)), start_virt_page), phys_addr1, phys_addr2);
			arch_write_memory(cpu, bb, phys_addr, R(R14_USR), 32);
			#endif
			memory_write(cpu, bb, Addr, R(R14_USR), 32);
			bb = cpu->dyncom_engine->bb;
			Addr = ADD(Addr, CONST(4));
		}
		if(BIT(15)){
			#if 0
			phys_addr1 = OR(start_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr2 = OR(end_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr = SELECT(ICMP_EQ(AND(Addr, CONST(0xFFFFF000)), start_virt_page), phys_addr1, phys_addr2);
			arch_write_memory(cpu, bb, phys_addr, STOREM_CHECK_PC, 32);
			#endif
			memory_write(cpu, bb, Addr, STOREM_CHECK_PC, 32);
			bb = cpu->dyncom_engine->bb;
		}
		return;
	}
	for( i = 0; i < 15; i ++ ){
		if(BIT(i)){
			#if 0
			phys_addr1 = OR(start_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr2 = OR(end_phys_page, AND(Addr, CONST(0xFFF)));
			phys_addr = SELECT(ICMP_EQ(AND(Addr, CONST(0xFFFFF000)), start_virt_page), phys_addr1, phys_addr2);

			if(i == RN)
				arch_write_memory(cpu, bb, phys_addr, Rn, 32);
			else
				arch_write_memory(cpu, bb, phys_addr, R(i), 32);
			#endif
			if(i == RN)
				memory_write(cpu, bb, Addr, Rn, 32);
			else
				memory_write(cpu, bb, Addr, R(i), 32);
	
			bb = cpu->dyncom_engine->bb;
			Addr = ADD(Addr, CONST(4));
		}
	}

	/* check pc reg*/
	if(BIT(i)){
		#if 0
		phys_addr1 = OR(start_phys_page, AND(Addr, CONST(0xFFF)));
		phys_addr2 = OR(end_phys_page, AND(Addr, CONST(0xFFF)));
		phys_addr = SELECT(ICMP_EQ(AND(Addr, CONST(0xFFFFF000)), start_virt_page), phys_addr1, phys_addr2);
		
		arch_write_memory(cpu, bb, phys_addr, STOREM_CHECK_PC, 32);
		#endif
		memory_write(cpu, bb, Addr, STOREM_CHECK_PC, 32);
		bb = cpu->dyncom_engine->bb;
	}
}

/* load store multiple operations collection, following arm doc */
void LoadStoreM(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr, Value* Rn)
{
	if(LSLBIT)
		LoadM(cpu,instr,bb,addr, Rn);
	else
		StoreM(cpu,instr,bb,addr, Rn);
}

/* load store operations collection */
void LoadStore(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value *addr, Value* Rn)
{
	if (BITS(20, 27) == 0x19 && BITS(0, 11) == 0xf9f) {
		/* LDREX */
		LoadWord(cpu, instr, bb, addr);
		return;
	}
	if(BITS(24,27) == 0x4 || BITS(24,27) == 0x5 || BITS(24,27) == 0x6 || BITS(24,27) == 0x7){
		WOrUBLoadStore(cpu, instr, bb, addr);
	}else if(BITS(24,27) == 0x0 || BITS(24,27) == 0x1){
		MisLoadStore(cpu, instr, bb, addr);
	}else if(BITS(24,27) == 0x8 || BITS(24,27) == 0x9){
		LoadStoreM(cpu, instr, bb, addr, Rn);
	}else{
		printf("Not a Load Store operation \n");
	}
}

#define CHECK_READ_REG15_WA(RN) ((RN == 15)? (ADD(AND(R(RN), CONST(0xFFFFFFFC)), CONST(INSTR_SIZE * 2))):R(RN))

// FIXME set_condition added by yukewei
// if S = 1 set CPSR zncv bit
int set_condition(cpu_t *cpu, Value *ret, BasicBlock *bb, Value *op1, Value *op2)
{
	/* N */ new StoreInst(ICMP_SLT(ret, CONST(0)), ptr_N, bb);
	/* Z */ new StoreInst(ICMP_EQ(ret, CONST(0)), ptr_Z, bb);
	/* C */ new StoreInst(ICMP_ULT(ret, op1), ptr_C, false, bb);
	/* V */ new StoreInst(ICMP_SLT(AND((XOR(op1, op2)), XOR(op1,ret)), CONST(0)), ptr_V, bb);
	return 0;
}

/*	Getting Address from a LoadStore instruction
*			GetAddr
*		|	   |		|
*	    MisGetAddr		    LSMGetAddr
*		      WOrUBGetAddr
*
*/
/* Addr Mode 1 */

/* Addr Mode 2, following arm operand doc */
/* Getting Word or Unsigned Byte Address Immediate offset operand.in arm doc */
Value *WOrUBGetAddrImmOffset(cpu_t *cpu, uint32_t instr, BasicBlock *bb)
{
	Value *Addr;
	if(LSUBIT)
		Addr =  ADD(CHECK_READ_REG15_WA(RN), CONST(OFFSET12));
	else
		Addr =  SUB(CHECK_READ_REG15_WA(RN), CONST(OFFSET12));

	//bb = arch_check_mm(cpu, bb, Addr, read, cpu->dyncom_engine->bb_trap);
	//CHECK_REG15();
	return Addr;
}

/* Getting Word or Unsigned Byte Address register offset operand.in arm doc */
Value *WOrUBGetAddrRegOffset(cpu_t *cpu, uint32_t instr, BasicBlock *bb)
{
	Value *Addr;
	if(LSUBIT)
		Addr =  ADD(CHECK_READ_REG15_WA(RN), R(RM));
	else
		Addr =  SUB(CHECK_READ_REG15_WA(RN), R(RM));

	//bb = arch_check_mm(cpu, bb, Addr, read, cpu->dyncom_engine->bb_trap);
	return Addr;
}

/* Getting Word or Unsigned Byte Address scaled register offset operand.in arm doc */
Value *WOrUBGetAddrScaledRegOffset(cpu_t *cpu, uint32_t instr, BasicBlock *bb)
{
	Value *Addr;
	int shift = SHIFT;
	Value *index;
	switch(shift) {
	case 0:	/* LSL */
		index = SHL(R(RM), CONST(SHIFT_IMM));
		break;
	case 1: /* LSR */
		if(SHIFT_IMM == 0)
			index = CONST(0);
		else
			index = LSHR(R(RM), CONST(SHIFT_IMM));
		break;
	case 2:	/* ASR */
		if(SHIFT_IMM == 0)
			index = ADD(XOR(LSHR(R(RM), CONST(31)), CONST(-1)), CONST(1));
		else
			index = ASHR(R(RM), CONST(SHIFT_IMM));
		break;
	case 3:	/* ROR or RRX */
		if(SHIFT_IMM == 0)
			;/* CFLAG? */
		else
			index = ROTL(R(RM), CONST(SHIFT_IMM));
		break;
	}

	if(LSUBIT)
		Addr = ADD(CHECK_READ_REG15_WA(RN), index);
	else
		Addr = SUB(CHECK_READ_REG15_WA(RN), index);

	//bb = arch_check_mm(cpu, bb, Addr, read, cpu->dyncom_engine->bb_trap);
	return Addr;
}

/* Getting Word or Unsigned Byte Address Immediate Preload operand.in arm doc */
Value *WOrUBGetAddrImmPre(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	Value *Addr = WOrUBGetAddrImmOffset(cpu, instr, bb);
	//bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
	//LET(RN, Addr);
	RECORD_WB(Addr, 1);
	return Addr;
}

/* Getting Word or Unsigned Byte Address Register Preload operand.in arm doc */
Value *WOrUBGetAddrRegPre(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	Value *Addr = WOrUBGetAddrRegOffset(cpu, instr, bb);
	//bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
	//LET(RN, Addr);
	RECORD_WB(Addr, 1);
	
	return Addr;
}

/* Getting Word or Unsigned Byte Address scaled Register Pre-indexed operand.in arm doc */
Value *WOrUBGetAddrScaledRegPre(cpu_t *cpu, uint32_t instr, BasicBlock *bb,int read)
{
	Value *Addr = WOrUBGetAddrScaledRegOffset(cpu, instr, bb);
	//bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
	//LET(RN, Addr);
	RECORD_WB(Addr, 1);
	return Addr;
}

/* Getting Word or Unsigned Byte Immediate Post-indexed operand.in arm doc */
Value *WOrUBGetAddrImmPost(cpu_t *cpu, uint32_t instr, BasicBlock *bb,int read)
{
	Value *Addr = R(RN);
	//bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
	//LET(RN,WOrUBGetAddrImmOffset(cpu, instr, bb));
	RECORD_WB(WOrUBGetAddrImmOffset(cpu, instr, bb), 1);
	return Addr;
}

/* Getting Word or Unsigned Byte Address register Post-indexed operand.in arm doc */
Value *WOrUBGetAddrRegPost(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	Value *Addr = R(RN);
	//bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
	//LET(RN,WOrUBGetAddrRegOffset(cpu, instr, bb));
	RECORD_WB(WOrUBGetAddrRegOffset(cpu, instr, bb), 1);
	return Addr;
}

/* Getting Word or Unsigned Byte Address scaled register Post-indexed operand.in arm doc */
Value *WOrUBGetAddrScaledRegPost(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	Value *Addr = R(RN);
	//bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
	//LET(RN,WOrUBGetAddrScaledRegOffset(cpu, instr, bb));
	RECORD_WB(WOrUBGetAddrScaledRegOffset(cpu, instr, bb), 1);
	return Addr;
}

/* Getting Word or Unsigned Byte Address Immediate operand operations collection */
Value *WOrUBGetAddrImm(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	if(BITS(24,27) == 0x5){
		if(!BIT(21)){
		/* ImmOff */
			Value* Addr= WOrUBGetAddrImmOffset(cpu, instr, bb);
			//bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
			return Addr;
		}else{
		/* ImmPre */
			return WOrUBGetAddrImmPre(cpu, instr, bb, read);
		}
	}else if(BITS(24,27) == 0x4){
		/* ImmPost */
		if(!BIT(21) || BIT(21)){
			return WOrUBGetAddrImmPost(cpu, instr, bb, read);
		}
	}
	printf(" Error in WOrUB Get Imm Addr instr is %x \n", instr);
	return NULL;
}

/* Getting Word or Unsigned Byte Address reg operand operations collection */
Value *WOrUBGetAddrReg(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	if(BITS(24,27) == 0x7){
		if(!BIT(21)){
		/* Reg off */
			if(!BITS(4,11)){
				Value* Addr = WOrUBGetAddrRegOffset(cpu, instr, bb);
				//bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
				return Addr;
			}else{
			/* scaled reg */
				Value* Addr = WOrUBGetAddrScaledRegOffset(cpu, instr, bb);
				//bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
				return Addr;
			}
		} else {
		/* Reg pre */
			if(!BITS(4,11)){
				return WOrUBGetAddrRegPre(cpu, instr, bb, read);
			}else{
			/* scaled reg */
				return WOrUBGetAddrScaledRegPre(cpu, instr, bb, read);
			}
		}
	}else if(BITS(24,27) == 0x6){
		if(!BIT(21)){
		/* Reg post */
			if(!BITS(4,11)){
				return WOrUBGetAddrRegPost(cpu, instr, bb, read);
			}else{
			/* scaled reg */
				return WOrUBGetAddrScaledRegPost(cpu, instr, bb, read);
			}
		}
	} else if (BITS(24, 27) == 0x5 && BIT(21) == 0) {
		Value* Addr = WOrUBGetAddrImmOffset(cpu, instr, bb);
		//arch_check_mm(cpu, bb, Addr, read, 4, cpu->dyncom_engine->bb_trap);
		return Addr;
	}
	printf(" Error in WOrUB Get Reg Addr inst is %x\n", instr);
	return NULL;
}

/* Getting Word or Unsigned Byte Address operand operations collection */
Value *WOrUBGetAddr(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	if(!BIT(25))
		return WOrUBGetAddrImm(cpu, instr, bb, read);
	else
		return WOrUBGetAddrReg(cpu, instr, bb, read);
}

/* Addr Mode 3, following arm operand doc */
/* Getting Miscellaneous Address Immidiate offset operand.in arm doc */
Value *MisGetAddrImmOffset(cpu_t *cpu, uint32_t instr, BasicBlock *bb)
{

	Value *Addr;
	Value *Offset_8;

	Offset_8 = CONST(IMMH << 4 | IMML);
	if(LSUBIT)
		Addr =  ADD(CHECK_READ_REG15_WA(RN), Offset_8);
	else
		Addr =  SUB(CHECK_READ_REG15_WA(RN), Offset_8);

	return Addr;
}

/* Getting Miscellaneous Address register offset operand.in arm doc */
Value *MisGetAddrRegOffset(cpu_t *cpu, uint32_t instr, BasicBlock *bb)
{
	Value *Addr;

	if(LSUBIT)
		Addr =  ADD(CHECK_READ_REG15_WA(RN), R(RM));
	else
		Addr =  SUB(CHECK_READ_REG15_WA(RN), R(RM));

	return Addr;
}

/* Getting Miscellaneous Address immdiate pre-indexed operand.in arm doc */
Value *MisGetAddrImmPre(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	Value *Addr = MisGetAddrImmOffset(cpu, instr, bb);
	
	//if((BIT(20) == 0 && (BITS(4, 7) == 0xF)) || (BIT(20) == 0 && (BITS(4, 7) == 0xd)))
	//	bb = arch_check_mm(cpu, bb, Addr, 8, read, cpu->dyncom_engine->bb_trap);
	//else
	//	bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
	//LET(RN, Addr);
	RECORD_WB(Addr, 1);

	return Addr;
}

/* Getting Miscellaneous Address registers pre-indexed operand.in arm doc */
Value *MisGetAddrRegPre(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	Value *Addr = MisGetAddrRegOffset(cpu, instr, bb);
	//if((BIT(20) == 0 && (BITS(4, 7) == 0xF)) || (BIT(20) == 0 && (BITS(4, 7) == 0xd)))
	//	bb = arch_check_mm(cpu, bb, Addr, 8, read, cpu->dyncom_engine->bb_trap);
	//else
	//	bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
	//LET(RN, Addr);
	RECORD_WB(Addr, 1);
	return Addr;
}

/* Getting Miscellaneous Address immdiate post-indexed operand.in arm doc */
Value *MisGetAddrImmPost(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	Value *Addr = CHECK_READ_REG15_WA(RN);
	//if((BIT(20) == 0 && (BITS(4, 7) == 0xF)) || (BIT(20) == 0 && (BITS(4, 7) == 0xd)))
	//	bb = arch_check_mm(cpu, bb, Addr, 8, read, cpu->dyncom_engine->bb_trap);
	//else
	//	bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
	//LET(RN, MisGetAddrImmOffset(cpu, instr, bb));
	RECORD_WB(MisGetAddrImmOffset(cpu, instr, bb), 1);
	return Addr;
}

/* Getting Miscellaneous Address register post-indexed operand.in arm doc */
Value *MisGetAddrRegPost(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	Value *Addr = CHECK_READ_REG15_WA(RN);
	//if((BIT(20) == 0 && (BITS(4, 7) == 0xF)) || (BIT(20) == 0 && (BITS(4, 7) == 0xd)))
	//	bb = arch_check_mm(cpu, bb, Addr, 8, read, cpu->dyncom_engine->bb_trap);
	//else
	//	bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
	//LET(RN, MisGetAddrRegOffset(cpu, instr, bb));
	RECORD_WB(MisGetAddrRegOffset(cpu, instr, bb), 1);
	return Addr;
}

/* Getting Miscellaneous Address immdiate operand operation collection. */
Value *MisGetAddrImm(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	if(BITS(24,27) == 0x0){
		if(BITS(21,22) == 0x2){
		/* Imm Post */
			return MisGetAddrImmPost(cpu, instr, bb, read);
		}
	}else if(BITS(24,27) == 0x1){
		if(BITS(21,22) == 0x2){
		/* Imm Offset */
			Value* Addr = MisGetAddrImmOffset(cpu, instr, bb);
			//if((BIT(20) == 0 && (BITS(4, 7) == 0xF)) || (BIT(20) == 0 && (BITS(4, 7) == 0xd)))
			//	bb = arch_check_mm(cpu, bb, Addr, 8, read, cpu->dyncom_engine->bb_trap);
			//else
			//	bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
			return Addr;
		}else if(BITS(21,22) == 0x3){
		/* Imm pre */
			return MisGetAddrImmPre(cpu, instr, bb, read);
		}
	}
	printf(" Error in Mis Get Imm Addr \n");
	return NULL;
}

/* Getting Miscellaneous Address register operand operation collection. */
Value *MisGetAddrReg(cpu_t *cpu, uint32_t instr, BasicBlock *bb,int read)
{
	if(BITS(24,27) == 0x0){
		if(BITS(21,22) == 0x0){
		/* Reg Post */
			return MisGetAddrRegPost(cpu, instr, bb, read);
		}
	}else if(BITS(24,27) == 0x1){
		if(BITS(21,22) == 0x0){
		/* Reg offset */
			Value* Addr = MisGetAddrRegOffset(cpu, instr, bb);
			/* ldrd and strd */
			//if((BIT(20) == 0 && (BITS(4, 7) == 0xF)) || (BIT(20) == 0 && (BITS(4, 7) == 0xd)))
			//	bb = arch_check_mm(cpu, bb, Addr, 8, read, cpu->dyncom_engine->bb_trap);
			//else
			//	bb = arch_check_mm(cpu, bb, Addr, 4, read, cpu->dyncom_engine->bb_trap);
			return Addr;
		}else if(BITS(21,22) == 0x1){
		/* Reg pre */
			return MisGetAddrRegPre(cpu, instr, bb, read);
		}
	}
	printf(" (DEC) Error in Mis Get Reg Addr %x\n", instr);
	return NULL;
}


/* Getting Miscellaneous Address operand operation collection. */
static Value *MisGetAddr(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	if(BIT(22))
		return MisGetAddrImm(cpu, instr, bb, read);
	else
		return MisGetAddrReg(cpu, instr, bb, read);

	return NULL;
}

/* Addr Mode 4 */
/* Getting Load Store Multiple Address and Increment After operand */
Value *LSMGetAddrIA(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	int i =  BITS(0,15);
	int count = 0;
	Value *Addr;
	while(i){
		if(i & 1)
			count ++;
		i = i >> 1;
	}

	Addr = CHECK_READ_REG15_WA(RN);

	//bb = arch_check_mm(cpu, bb, Addr, count * 4 , read, cpu->dyncom_engine->bb_trap);
	if(LSWBIT){
		//LET(RN, ADD(CHECK_READ_REG15_WA(RN), CONST(count * 4)));
		RECORD_WB(ADD(CHECK_READ_REG15_WA(RN), CONST(count * 4)), 1);
	}
	return  Addr;
}

/* Getting Load Store Multiple Address and Increment Before operand */
Value *LSMGetAddrIB(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	int i =  BITS(0,15);
	int count = 0;
	Value *Addr;
	while(i){
		if(i & 1)
			count ++;
		i = i >> 1;
	}
	assert(count != 0);
	Addr = ADD(CHECK_READ_REG15_WA(RN), CONST(4));
	//bb = arch_check_mm(cpu, bb, Addr, count * 4, read, cpu->dyncom_engine->bb_trap);
	if(LSWBIT){
		//LET(RN, ADD(CHECK_READ_REG15_WA(RN), CONST(count * 4)));
		RECORD_WB(ADD(CHECK_READ_REG15_WA(RN), CONST(count * 4)), 1);
	}

	return  Addr;
}

/* Getting Load Store Multiple Address and Decrement After operand. */
Value *LSMGetAddrDA(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	int i =  BITS(0,15);
	int count = 0;
	Value *Addr;
	while(i){
		if(i & 1)
			count ++;
		i = i >> 1;
	}

	Addr = ADD(SUB(CHECK_READ_REG15_WA(RN), CONST(count * 4)), CONST(4));
	//bb = arch_check_mm(cpu, bb, Addr, count * 4, read, cpu->dyncom_engine->bb_trap);
	if(LSWBIT){
		//LET(RN, SUB(CHECK_READ_REG15_WA(RN), CONST(count * 4)));
		RECORD_WB(SUB(CHECK_READ_REG15_WA(RN), CONST(count * 4)), 1);
	}
	return  Addr;
}

/* Getting Load Store Multiple Address and Decrement Before operand. */
Value *LSMGetAddrDB(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	int i =  BITS(0,15);
	int count = 0;
	Value *Addr;
	while(i){
		if(i & 1)
			count ++;
		i = i >> 1;
	}

	Addr = SUB(CHECK_READ_REG15_WA(RN), CONST(count * 4));
	//bb = arch_check_mm(cpu, bb, Addr, count * 4, read, cpu->dyncom_engine->bb_trap);
	if(LSWBIT){
		//LET(RN, SUB(CHECK_READ_REG15_WA(RN), CONST(count * 4)));
		RECORD_WB(SUB(CHECK_READ_REG15_WA(RN), CONST(count * 4)), 1);
	}
	return  Addr;
}

/* Getting Load Store Multiple Address operand operation collection. */
Value *LSMGetAddr(cpu_t *cpu, uint32_t instr, BasicBlock *bb,int read)
{
	if(BITS(24,27) == 0x8){
		if(BIT(23)){
		/* IA */
			return LSMGetAddrIA(cpu, instr, bb, read);
		}else{
		/* DA */
			return LSMGetAddrDA(cpu, instr, bb, read);
		}
	}else if(BITS(24,27) == 0x9){
		if(BIT(23)){
		/* IB */
			return LSMGetAddrIB(cpu, instr, bb, read);
		}else{
		/* DB */
			return LSMGetAddrDB(cpu, instr, bb, read);
		}
	}

	printf(" Error in LSM Get Imm Addr BITS(24,27) is 0x%x\n", BITS(24,27));
	return NULL;
}
Value* GetPhysAddr(cpu_t *cpu, uint32_t instr, BasicBlock *bb, Value* virtAddr, int read){
        
        //Value* phys_page = arm_dyncom_check_mm(cpu, bb, instr, virtAddr, read);
	//arch_check_mm(cpu, instr, bb, bb, cpu->dyncom_engine->bb_trap);
	//Value *cond = ICMP_EQ(AND(phys_page, CONST(0x1)), CONST(0));
	//arch_branch(1, cpu->dyncom_engine->bb_trap, cpu->dyncom_engine->bb_load_store, cond, bb);
        
	//bb = cpu->dyncom_engine->bb_load_store;
        /* The memory translation is successfully, we should: is finished */
	//return OR(AND(phys_page, CONST(0xFFFFF000)), AND(virtAddr, CONST(0xFFF)));
	return CONST(0);
}
/* all,Getting Load Store Address operand operation collection */
Value *GetAddr(cpu_t *cpu, uint32_t instr, BasicBlock *bb, int read)
{
	if(BITS(24,27) == 0x1 || BITS(24,27) == 0x2 || BITS(24, 27) == 0){
		return MisGetAddr(cpu,instr,bb, read);
	}else if(BITS(24,27) == 0x4 || BITS(24,27) == 0x5 || BITS(24,27) == 0x6 || BITS(24,27) == 0x7 ){
		return WOrUBGetAddr(cpu,instr,bb, read);
	}else if(BITS(24,27) == 0x8 || BITS(24,27) == 0x9){
		return LSMGetAddr(cpu,instr,bb, read);
	}

	printf("Not a Load Store Addr operation %x\n", instr);
	return CONST(0);
}

#define OPERAND_RETURN_CHECK_PC  do{  \
	if(RM == 15)		\
		return ADD(R(RM), CONST(INSTR_SIZE * 2));	\
	else	\
		return R(RM);	\
}while(0)

#if 1
/* index:0 */
/* register immediate */
Value *Data_ope_Reg(cpu_t *cpu,  uint32_t instr, BasicBlock *bb, uint32_t shift_imm, Value *shamt, Value *sco)
{
	if (!shift_imm) { /* Register */
		OPERAND_RETURN_CHECK_PC;
		/* No changes in SCO */
	} else {	/* logic shift left by imm */
		if (sco != NULL)
		{
			new StoreInst(ICMP_SLT(SHL(R(RM), CONST(shift_imm-1)), CONST(0)),sco,bb);
		}
		return SHL(R(RM), CONST(shift_imm));
	}
}

/* Get date from instruction operand */
/* index:1 */
/* Getting data form Logic Shift Left register operand. following arm doc. */
Value *Data_ope_LogLReg(cpu_t *cpu,  uint32_t instr, BasicBlock *bb, uint32_t shift_imm, Value *shamt, Value *sco)
{
	if (sco != NULL)
	{
		Value *flag = SELECT(ICMP_EQ(shamt, CONST(0)), new LoadInst(ptr_C, "", false, bb), /* Rs[7:0] == 0 */
				     SELECT(ICMP_ULT(shamt, CONST(32)), ICMP_SLT(SHL(R(RM), SUB(shamt, CONST(1))), CONST(0)), /* Rs[7:0] < 32 */
					    SELECT(ICMP_EQ(shamt, CONST(32)), TRUNC1(R(RM)), /* Rs[7:0] == 32*/
						   CONST1(0) /* Rs[7:0] > 32 */
						   )
					    )
				     );
		new StoreInst(flag, sco, bb);
	}
	/* logic shift left by reg ICMP_ULE(shamt, CONST(32)) ?????? */
	return SELECT(ICMP_EQ(shamt, CONST(0)), R(RM), SELECT(ICMP_UGE(shamt, CONST(32)), CONST(0), SHL(R(RM), shamt)));
}

/* index:2 */
/* Getting data form Logic Shift Right immdiate operand. following arm doc. */
Value *Data_ope_LogRImm(cpu_t *cpu,  uint32_t instr, BasicBlock *bb, uint32_t shift_imm, Value *shamt, Value *sco)
{
	/* logic shift right by imm */
	if(!shift_imm) {
		if (sco != NULL)
			new StoreInst(ICMP_SLT(R(RM), CONST(0)), sco, bb);
 		return CONST(0);
	} else {
		if (sco != NULL)
			new StoreInst(ICMP_SLT(SHL(R(RM), CONST(32 - shift_imm)), CONST(0)), sco, bb);
 		return LSHR(R(RM), CONST(shift_imm));
	}
}

/* index:3 */
/* Getting data form Logic Shift Right register operand. following arm doc. */
Value *Data_ope_LogRReg(cpu_t *cpu,  uint32_t instr, BasicBlock *bb, uint32_t shift_imm, Value *shamt, Value *sco)
{
	if (sco != NULL)
	{
		Value *flag = SELECT(ICMP_EQ(shamt, CONST(0)), new LoadInst(ptr_C, "", false, bb), /* Rs[7:0] == 0 */
				     SELECT(ICMP_ULT(shamt, CONST(32)), ICMP_SLT(SHL(R(RM), SUB(CONST(32), shamt)), CONST(0)), /* Rs[7:0] < 32 */
					    SELECT(ICMP_EQ(shamt, CONST(32)), ICMP_SLT(shamt, CONST(0)), /* Rs[7:0] == 32*/
						   CONST1(0) /* Rs[7:0] > 32 */
						   )
					    )
				     );
		new StoreInst(flag, sco, bb);
	}
	/* logic shift right by reg*/
	return SELECT(ICMP_EQ(shamt, CONST(0)), R(RM), SELECT(ICMP_UGE(shamt, CONST(32)), CONST(0), LSHR(R(RM), shamt)));
}

/* index:4 */
/* Getting data form Shift Right immdiate operand. following arm doc. */
Value *Data_ope_AriRImm(cpu_t *cpu,  uint32_t instr, BasicBlock *bb, uint32_t shift_imm, Value *shamt, Value *sco)
{
 	/* shift right by imm */
	if(!shift_imm) {
		if (sco != NULL)
			new StoreInst(ICMP_SLT(R(RM), CONST(0)), sco, bb);
		return SELECT(LSHR(R(RM), CONST(31)), CONST(0xffffffff), CONST(0));
	} else {
		if (sco != NULL)
			new StoreInst(ICMP_SLT(SHL(R(RM), CONST(32-shift_imm)), CONST(0)), sco, bb);
 		return ASHR(R(RM), CONST(shift_imm));
	}
}

/* index:5 */
/* Getting data form Shift Right register operand. following arm doc. */
Value *Data_ope_AriRReg(cpu_t *cpu,  uint32_t instr, BasicBlock *bb, uint32_t shift_imm, Value *shamt, Value *sco)
{
	/* arth shift right by reg */
	if (sco != NULL)
	{
		Value *flag = SELECT(ICMP_EQ(shamt, CONST(0)), new LoadInst(ptr_C, "", false, bb), /* Rs[7:0] == 0 */
				     SELECT(ICMP_ULT(shamt, CONST(32)), ICMP_SLT(SHL(R(RM), SUB(CONST(32), shamt)), CONST(0)), /* Rs[7:0] < 32 */
					    ICMP_SLT(R(RM), CONST(0)) /* Rs[7:0] <= 32 */
					    )
				     );
		new StoreInst(flag, sco, bb);
	}
	return SELECT(ICMP_EQ(shamt, CONST(0)), R(RM),
			SELECT(ICMP_ULT(shamt, CONST(32)), ASHR(R(RM), shamt),
				SELECT(ICMP_EQ(LSHR(R(RM), CONST(31)), CONST(0)), CONST(0), CONST(0xffffffff))));
}

/* index:6 */
/* Getting data form Rotate Shift Right immdiate operand. following arm doc. */
Value *Data_ope_RotRImm(cpu_t *cpu,  uint32_t instr, BasicBlock *bb, uint32_t shift_imm, Value *shamt, Value *sco)
{
 	if(!shift_imm){
 		/* Rotate right with extend */
		Value *ret = ROTL(OR(SHL(ptr_C, CONST(31)), ASHR(R(RM), CONST(1))), CONST(1));
		if (sco != NULL)
			new StoreInst(TRUNC1(R(RM)), sco, bb); /* Beware, ptr_C nust be modified after */
		return ret;
	} else {
		if (sco != NULL)
			new StoreInst(ICMP_SLT(SHL(R(RM), CONST(32 - shift_imm)), CONST(0)), sco, bb);
 		/* Rotate right by imm */
 		return ROTL(R(RM), CONST(32 - shift_imm));
	}
}

/* index:7 */
/* Getting data form Rotate Shift Right register operand. following arm doc. */
Value *Data_ope_RotRReg(cpu_t *cpu,  uint32_t instr, BasicBlock *bb, uint32_t shift_imm, Value *shamt, Value *sco)
{
	Value *sham = AND(R(BITS(8, 11)), CONST(0x1f));
	/* Rotate right by reg */
	if (sco != NULL) {
		Value *flag = SELECT(ICMP_EQ(shamt, CONST(0)), new LoadInst(ptr_C, "", false, bb), /* Rs[7:0] == 0 */
				     SELECT(ICMP_EQ(sham, CONST(0)), ICMP_SLT(R(RM), CONST(0)), /* Rs[4:0] == 0 */
					    ICMP_SLT(SHL(R(RM), SUB(CONST(32), sham)), CONST(0)) /* Rs[4:0] > 0 */
					    )
				     );
		new StoreInst(flag, sco, bb);
	}
	return SELECT(ICMP_EQ(shamt, CONST(0)), R(RM), SELECT(ICMP_EQ(sham, CONST(0)), R(RM), ROTL(R(RM), SUB(CONST(32), sham))));
}

Value *(*Data_operand[8])(cpu_t*, uint32_t, BasicBlock *, uint32_t, Value*, Value*) = {Data_ope_Reg, Data_ope_LogLReg, Data_ope_LogRImm, Data_ope_LogRReg, Data_ope_AriRImm, Data_ope_AriRReg, Data_ope_RotRImm, Data_ope_RotRReg};

/* Getting data form operand collection. */
Value *operand(cpu_t *cpu,  uint32_t instr, BasicBlock *bb, Value *sco)
{
	uint32_t shift = BITS(4, 6);
	uint32_t shift_imm = BITS(7,11);
	Value *shamt = AND(R(BITS(8,11)), CONST(0xff));

	if(I) {
		/* 32-bit immediate */
		uint32_t immed_8 = instr & 0xFF;
		int rotate_immx2 = (instr & 0xF00) >> 7; //((instr >> 8) & 0xF) << 1;
		uint32_t immediate = (immed_8 >> (rotate_immx2)) | (immed_8 << (32 - rotate_immx2));
			
		if (sco != NULL)
		{
			if(rotate_immx2) {
				new StoreInst(CONST1(immediate >> 31),sco, bb);
			}
			/* No changes in C flag else */
		}
		//printf("instr=0x%x, immediate=0x%x\n", instr, immediate);	
		return CONST(immediate);
	} else if (BITS(4, 11) == 0x6 && BITS(25, 27) == 0) {
		/*  Rotate right with extend  */
		Value* carry_out = TRUNC1(LSHR(R(RM), CONST(31)));
		Value *rm = LSHR(R(RM), CONST(1));
		Value *tmp = SELECT(ICMP_EQ(LOAD(ptr_C), CONST1(0)), CONST(0), CONST(0x80000000));
		if(sco != NULL)
			new StoreInst(carry_out,sco, bb);

		return OR(rm, tmp);
	} else {
		/* operand with BIT 4 ~ 6 */
		return (Data_operand[shift])(cpu, instr, bb, shift_imm, shamt, sco);
	}
}
#endif
#if 0
Value *operand(cpu_t *cpu,  uint32_t instr, BasicBlock *bb)
{

        if (I) { /* 32-bit immediate */
                //XXX TODO: shifter carry out
                uint32_t immed_8 = instr & 0xFF;
                int rotate_imm = ((instr >> 8) & 0xF) << 1;
                return CONST((immed_8 >> rotate_imm) | (immed_8 << (32 - rotate_imm)));
        } else {
                if (!BIT(4)) { /* Immediate shifts */
                        int shift = BITS(5,6);
                        int shift_imm = BITS(7,11);
                        LOG("shift=%x\n", shift);
                        LOG("shift_imm=%x\n", shift_imm);
                        if (!shift && !shift_imm) { /* Register */
                                //return R(RM);
				OPERAND_RETURN_CHECK_PC;
                        } else {
				switch(shift){
				case 0: /* logic shift left by imm */
					if(!shift_imm)
						//return R(RM);
						OPERAND_RETURN_CHECK_PC;
					else
						return SHL(R(RM), CONST(shift_imm));
				case 1:	/* logi shift right by imm */
					if(!shift_imm)
						return CONST(0);
					else
						return LSHR(R(RM), CONST(shift_imm));
				case 2:
					if(!shift_imm)
						return SELECT(ICMP_ULE(R(RM), CONST(0x80000000)), CONST(0), LSHR(R(RM), CONST(31)));
					else
						return ASHR(R(RM), CONST(shift_imm));
				case 3:
					if(!shift_imm){
						BAD;
					}
					else
						return ROTL(R(RM), CONST(shift_imm));
                                //BAD;

				}
                        }
                } else {
                        if (!BIT(7)) { /* Register shifts */
				Value *shamt = AND(R(BITS(8,11)), CONST(0xff));
				switch(BITS(5,6)){
					case 0:  /* LSL */
						return SELECT( ICMP_EQ(shamt, CONST(0)), R(RM), SELECT(ICMP_UGE(shamt, CONST(32)), CONST(0), SHL(R(RM), shamt)));
					case 1:  /* LSR */
						return SELECT( ICMP_EQ(shamt, CONST(0)), R(RM), SELECT(ICMP_UGE(shamt, CONST(32)), CONST(0), LSHR(R(RM), shamt)));
					case 2:  /* ASR */
						return SELECT( ICMP_EQ(shamt, CONST(0)), R(RM), SELECT(ICMP_UGE(shamt, CONST(32)), LSHR(R(RM), CONST(31)),LSHR(R(RM), shamt)));
					case 3: /* ROR */
						return SELECT( ICMP_EQ(shamt, CONST(0)), R(RM), ROTL(R(RM),SUB(CONST(32), shamt)));
				}
                        } else { /* arithmetic or Load/Store instruction extension space */
                                BAD;
                        }
                }
        }
}
#endif

/* Getting data from branch instruction operand */
uint32_t boperand(uint32_t instr)
{
	#if 1
               uint32_t rotate_imm = instr;
               if(instr &  0x800000)
                       rotate_imm = (~rotate_imm + 1) & 0x0ffffff;
#else
		uint32_t rotate_imm = instr & 0xffffff;
		if(rotate_imm &  0x800000) {
			rotate_imm |= 0xff000000;
			//rotate_imm = (~rotate_imm + 1) & 0x3fffffff;
			//rotate_imm &= 0x3fffffff;
		}
#endif
		else
			rotate_imm &= 0x0ffffff;

		rotate_imm = rotate_imm << 2;

//		printf("rotate_imm is %x\n", rotate_imm);
		return rotate_imm;
}
#if 0
#define OPERAND operand(cpu,instr,bb)
#define BOPERAND boperand(cpu,instr,bb)

#define CHECK_RN_PC  (RN==15? ADD(R(RN), CONST(8)):R(RN))
#endif

const ISEITEM arm_instruction[] = {
	#define VFP_DECODE
	#include "vfp/vfpinstr.c"
	#undef VFP_DECODE
	{"srs"	,  4	,  6	, 25, 31, 0x0000007c, 22, 22, 0x00000001, 16, 20, 0x0000000d,  8, 11, 0x00000005},
	{"rfe"	,  4	,  6	, 25, 31, 0x0000007c, 22, 22, 0x00000000, 20, 20, 0x00000001,  8, 11, 0x0000000a},
	{"bkpt"	,  2	,  3	, 20, 31, 0x00000e12,  4,  7, 0x00000007},
	{"blx"	,  1	,  3	, 25, 31, 0x0000007d},
	{"cps"	,  3	,  6	, 20, 31, 0x00000f10, 16, 16, 0x00000000,  5,  5, 0x00000000},
	{"pld"	,  4	,  4	, 26, 31, 0x0000003d, 24, 24, 0x00000001, 20, 22, 0x00000005, 12, 15, 0x0000000f},
	{"setend"	,  2	,  6	, 16, 31, 0x0000f101,  4,  7, 0x00000000},
	{"clrex"	,  1	,  6	,  0, 31, 0xf57ff01f},
	{"rev16"	,  2	,  6	, 16, 27, 0x000006bf,  4, 11, 0x000000fb},
	{"usad8"	,  3	,  6	, 20, 27, 0x00000078, 12, 15, 0x0000000f,  4,  7, 0x00000001},
	{"sxtb"	,  2	,  6	, 16, 27, 0x000006af,  4,  7, 0x00000007},
	{"uxtb"	,  2	,  6	, 16, 27, 0x000006ef,  4,  7, 0x00000007},
	{"sxth"	,  2	,  6	, 16, 27, 0x000006bf,  4,  7, 0x00000007},
	{"sxtb16"	,  2	,  6	, 16, 27, 0x0000068f,  4,  7, 0x00000007},
	{"uxth"	,  2	,  6	, 16, 27, 0x000006ff,  4,  7, 0x00000007},
	{"uxtb16"	,  2	,  6	, 16, 27, 0x000006cf,  4,  7, 0x00000007},
	{"cpy"	,  2	,  6	, 20, 27, 0x0000001a,  4, 11, 0x00000000},
	{"uxtab"	,  2	,  6	, 20, 27, 0x0000006e,  4,  9, 0x00000007},
	{"ssub8"	,  2	,  6	, 20, 27, 0x00000061,  4,  7, 0x0000000f},
	{"shsub8"	,  2	,  6	, 20, 27, 0x00000063,  4,  7, 0x0000000f},
	{"ssubaddx"	,  2	,  6	, 20, 27, 0x00000061,  4,  7, 0x00000005},
	{"strex"	,  2	,  6	, 20, 27, 0x00000018,  4,  7, 0x00000009},
	{"strexb"	,  2	,  7	, 20, 27, 0x0000001c,  4,  7, 0x00000009},
	{"swp"	,  2	,  0	, 20, 27, 0x00000010,  4,  7, 0x00000009},
	{"swpb"	,  2	,  0	, 20, 27, 0x00000014,  4,  7, 0x00000009},
	{"ssub16"	,  2	,  6	, 20, 27, 0x00000061,  4,  7, 0x00000007},
	{"ssat16"	,  2	,  6	, 20, 27, 0x0000006a,  4,  7, 0x00000003},
	{"shsubaddx"	,  2	,  6	, 20, 27, 0x00000063,  4,  7, 0x00000005},
	{"qsubaddx"	,  2	,  6	, 20, 27, 0x00000062,  4,  7, 0x00000005},
	{"shaddsubx"	,  2	,  6	, 20, 27, 0x00000063,  4,  7, 0x00000003},
	{"shadd8"	,  2	,  6	, 20, 27, 0x00000063,  4,  7, 0x00000009},
	{"shadd16"	,  2	,  6	, 20, 27, 0x00000063,  4,  7, 0x00000001},
	{"sel"	,  2	,  6	, 20, 27, 0x00000068,  4,  7, 0x0000000b},
	{"saddsubx"	,  2	,  6	, 20, 27, 0x00000061,  4,  7, 0x00000003},
	{"sadd8"	,  2	,  6	, 20, 27, 0x00000061,  4,  7, 0x00000009},
	{"sadd16"	,  2	,  6	, 20, 27, 0x00000061,  4,  7, 0x00000001},
	{"shsub16"	,  2	,  6	, 20, 27, 0x00000063,  4,  7, 0x00000007},
	{"umaal"	,  2	,  6	, 20, 27, 0x00000004,  4,  7, 0x00000009},
	{"uxtab16"	,  2	,  6	, 20, 27, 0x0000006c,  4,  7, 0x00000007},
	{"usubaddx"	,  2	,  6	, 20, 27, 0x00000065,  4,  7, 0x00000005},
	{"usub8"	,  2	,  6	, 20, 27, 0x00000065,  4,  7, 0x0000000f},
	{"usub16"	,  2	,  6	, 20, 27, 0x00000065,  4,  7, 0x00000007},
	{"usat16"	,  2	,  6	, 20, 27, 0x0000006e,  4,  7, 0x00000003},
	{"usada8"	,  2	,  6	, 20, 27, 0x00000078,  4,  7, 0x00000001},
	{"uqsubaddx"	,  2	,  6	, 20, 27, 0x00000066,  4,  7, 0x00000005},
	{"uqsub8"	,  2	,  6	, 20, 27, 0x00000066,  4,  7, 0x0000000f},
	{"uqsub16"	,  2	,  6	, 20, 27, 0x00000066,  4,  7, 0x00000007},
	{"uqaddsubx"	,  2	,  6	, 20, 27, 0x00000066,  4,  7, 0x00000003},
	{"uqadd8"	,  2	,  6	, 20, 27, 0x00000066,  4,  7, 0x00000009},
	{"uqadd16"	,  2	,  6	, 20, 27, 0x00000066,  4,  7, 0x00000001},
	{"sxtab"	,  2	,  6	, 20, 27, 0x0000006a,  4,  7, 0x00000007},
	{"uhsubaddx"	,  2	,  6	, 20, 27, 0x00000067,  4,  7, 0x00000005},
	{"uhsub8"	,  2	,  6	, 20, 27, 0x00000067,  4,  7, 0x0000000f},
	{"uhsub16"	,  2	,  6	, 20, 27, 0x00000067,  4,  7, 0x00000007},
	{"uhaddsubx"	,  2	,  6	, 20, 27, 0x00000067,  4,  7, 0x00000003},
	{"uhadd8"	,  2	,  6	, 20, 27, 0x00000067,  4,  7, 0x00000009},
	{"uhadd16"	,  2	,  6	, 20, 27, 0x00000067,  4,  7, 0x00000001},
	{"uaddsubx"	,  2	,  6	, 20, 27, 0x00000065,  4,  7, 0x00000003},
	{"uadd8"	,  2	,  6	, 20, 27, 0x00000065,  4,  7, 0x00000009},
	{"uadd16"	,  2	,  6	, 20, 27, 0x00000065,  4,  7, 0x00000001},
	{"sxtah"	,  2	,  6	, 20, 27, 0x0000006b,  4,  7, 0x00000007},
	{"sxtab16"	,  2	,  6	, 20, 27, 0x00000068,  4,  7, 0x00000007},
	{"qadd8"	,  2	,  6	, 20, 27, 0x00000062,  4,  7, 0x00000009},
	{"bxj"	,  2	,  5	, 20, 27, 0x00000012,  4,  7, 0x00000002},
	{"clz"	,  2	,  3	, 20, 27, 0x00000016,  4,  7, 0x00000001},
	{"uxtah"	,  2	,  6	, 20, 27, 0x0000006f,  4,  7, 0x00000007},
	{"bx"	,  2	,  2	, 20, 27, 0x00000012,  4,  7, 0x00000001},
	{"rev"	,  2	,  6	, 20, 27, 0x0000006b,  4,  7, 0x00000003},
	{"blx"	,  2	,  3	, 20, 27, 0x00000012,  4,  7, 0x00000003},
	{"revsh"	,  2	,  6	, 20, 27, 0x0000006f,  4,  7, 0x0000000b},
	{"qadd"	,  2	,  4	, 20, 27, 0x00000010,  4,  7, 0x00000005},
	{"qadd16"	,  2	,  6	, 20, 27, 0x00000062,  4,  7, 0x00000001},
	{"qaddsubx"	,  2	,  6	, 20, 27, 0x00000062,  4,  7, 0x00000003},
	{"ldrex"	,  2	,  0	, 20, 27, 0x00000019,  4,  7, 0x00000009},
	{"qdadd"	,  2	,  4	, 20, 27, 0x00000014,  4,  7, 0x00000005},
	{"qdsub"	,  2	,  4	, 20, 27, 0x00000016,  4,  7, 0x00000005},
	{"qsub"	,  2	,  4	, 20, 27, 0x00000012,  4,  7, 0x00000005},
	{"ldrexb"	,  2	,  7	, 20, 27, 0x0000001d,  4,  7, 0x00000009},
	{"qsub8"	,  2	,  6	, 20, 27, 0x00000062,  4,  7, 0x0000000f},
	{"qsub16"	,  2	,  6	, 20, 27, 0x00000062,  4,  7, 0x00000007},
	{"smuad"	,  4	,  6	, 20, 27, 0x00000070, 12, 15, 0x0000000f,  6,  7, 0x00000000,  4,  4, 0x00000001},
	{"smmul"	,  4	,  6	, 20, 27, 0x00000075, 12, 15, 0x0000000f,  6,  7, 0x00000000,  4,  4, 0x00000001},
	{"smusd"	,  4	,  6	, 20, 27, 0x00000070, 12, 15, 0x0000000f,  6,  7, 0x00000001,  4,  4, 0x00000001},
	{"smlsd"	,  3	,  6	, 20, 27, 0x00000070,  6,  7, 0x00000001,  4,  4, 0x00000001},
	{"smlsld"	,  3	,  6	, 20, 27, 0x00000074,  6,  7, 0x00000001,  4,  4, 0x00000001},
	{"smmla"	,  3	,  6	, 20, 27, 0x00000075,  6,  7, 0x00000000,  4,  4, 0x00000001},
	{"smmls"	,  3	,  6	, 20, 27, 0x00000075,  6,  7, 0x00000003,  4,  4, 0x00000001},
	{"smlald"	,  3	,  6	, 20, 27, 0x00000074,  6,  7, 0x00000000,  4,  4, 0x00000001},
	{"smlad"	,  3	,  6	, 20, 27, 0x00000070,  6,  7, 0x00000000,  4,  4, 0x00000001},
	{"smlaw"	,  3	,  4	, 20, 27, 0x00000012,  7,  7, 0x00000001,  4,  5, 0x00000000},
	{"smulw"	,  3	,  4	, 20, 27, 0x00000012,  7,  7, 0x00000001,  4,  5, 0x00000002},
	{"pkhtb"	,  2	,  6	, 20, 27, 0x00000068,  4,  6, 0x00000005},
	{"pkhbt"	,  2	,  6	, 20, 27, 0x00000068,  4,  6, 0x00000001},
	{"smul"	,  3	,  4	, 20, 27, 0x00000016,  7,  7, 0x00000001,  4,  4, 0x00000000},
	{"smlalxy"	,  3	,  4	, 20, 27, 0x00000014,  7,  7, 0x00000001,  4,  4, 0x00000000},
//	{"smlal"	,  2	,  4	, 21, 27, 0x00000007,  4,  7, 0x00000009},
	{"smla"	,  3	,  4	, 20, 27, 0x00000010,  7,  7, 0x00000001,  4,  4, 0x00000000},
	{"mcrr"	,  1	,  6	, 20, 27, 0x000000c4},
	{"mrrc"	,  1	,  6	, 20, 27, 0x000000c5},
	{"cmp"	,  2	,  0	, 26, 27, 0x00000000, 20, 24, 0x00000015},
	{"tst"	,  2	,  0	, 26, 27, 0x00000000, 20, 24, 0x00000011},
	{"teq"	,  2	,  0	, 26, 27, 0x00000000, 20, 24, 0x00000013},
	{"cmn"	,  2	,  0	, 26, 27, 0x00000000, 20, 24, 0x00000017},
	{"smull"	,  2	,  0	, 21, 27, 0x00000006,  4,  7, 0x00000009},
	{"umull"	,  2	,  0	, 21, 27, 0x00000004,  4,  7, 0x00000009},
	{"umlal"	,  2	,  0	, 21, 27, 0x00000005,  4,  7, 0x00000009},
	{"smlal"	,  2	,  0	, 21, 27, 0x00000007,  4,  7, 0x00000009},
	{"mul"	,  2	,  0	, 21, 27, 0x00000000,  4,  7, 0x00000009},
	{"mla"	,  2	,  0	, 21, 27, 0x00000001,  4,  7, 0x00000009},
	{"ssat"	,  2	,  6	, 21, 27, 0x00000035,  4,  5, 0x00000001},
	{"usat"	,  2	,  6	, 21, 27, 0x00000037,  4,  5, 0x00000001},
	{"mrs"	,  4	,  0	, 23, 27, 0x00000002, 20, 21, 0x00000000, 16, 19, 0x0000000f,  0, 11, 0x00000000},
	{"msr"	,  3	,  0	, 23, 27, 0x00000002, 20, 21, 0x00000002,  4,  7, 0x00000000},
	{"and"	,  2	,  0	, 26, 27, 0x00000000, 21, 24, 0x00000000},
	{"bic"	,  2	,  0	, 26, 27, 0x00000000, 21, 24, 0x0000000e},
	{"ldm"	,  3	,  0	, 25, 27, 0x00000004, 20, 22, 0x00000005, 15, 15, 0x00000000},
	{"eor"	,  2	,  0	, 26, 27, 0x00000000, 21, 24, 0x00000001},
	{"add"	,  2	,  0	, 26, 27, 0x00000000, 21, 24, 0x00000004},
	{"rsb"	,  2	,  0	, 26, 27, 0x00000000, 21, 24, 0x00000003},
	{"rsc"	,  2	,  0	, 26, 27, 0x00000000, 21, 24, 0x00000007},
	{"sbc"	,  2	,  0	, 26, 27, 0x00000000, 21, 24, 0x00000006},
	{"adc"	,  2	,  0	, 26, 27, 0x00000000, 21, 24, 0x00000005},
	{"sub"	,  2	,  0	, 26, 27, 0x00000000, 21, 24, 0x00000002},
	{"orr"	,  2	,  0	, 26, 27, 0x00000000, 21, 24, 0x0000000c},
	{"mvn"	,  2	,  0	, 26, 27, 0x00000000, 21, 24, 0x0000000f},
	{"mov"	,  2	,  0	, 26, 27, 0x00000000, 21, 24, 0x0000000d},
	{"stm"	,  2	,  0	, 25, 27, 0x00000004, 20, 22, 0x00000004},
	{"ldm"	,  4	,  0	, 25, 27, 0x00000004, 22, 22, 0x00000001, 20, 20, 0x00000001, 15, 15, 0x00000001},
	{"ldrsh"	,  3	,  2	, 25, 27, 0x00000000, 20, 20, 0x00000001,  4,  7, 0x0000000f},
	{"stm"	,  3	,  0	, 25, 27, 0x00000004, 22, 22, 0x00000000, 20, 20, 0x00000000},
	{"ldm"	,  3	,  0	, 25, 27, 0x00000004, 22, 22, 0x00000000, 20, 20, 0x00000001},
	{"ldrsb"	,  3	,  2	, 25, 27, 0x00000000, 20, 20, 0x00000001,  4,  7, 0x0000000d},
	{"strd"	,  3	,  4	, 25, 27, 0x00000000, 20, 20, 0x00000000,  4,  7, 0x0000000f},
	{"ldrh"	,  3	,  0	, 25, 27, 0x00000000, 20, 20, 0x00000001,  4,  7, 0x0000000b},
	{"strh"	,  3	,  0	, 25, 27, 0x00000000, 20, 20, 0x00000000,  4,  7, 0x0000000b},
	{"ldrd"	,  3	,  4	, 25, 27, 0x00000000, 20, 20, 0x00000000,  4,  7, 0x0000000d},
	{"strt"	,  3	,  0	, 26, 27, 0x00000001, 24, 24, 0x00000000, 20, 22, 0x00000002},
	{"strbt"	,  3	,  0	, 26, 27, 0x00000001, 24, 24, 0x00000000, 20, 22, 0x00000006},
	{"ldrbt"	,  3	,  0	, 26, 27, 0x00000001, 24, 24, 0x00000000, 20, 22, 0x00000007},
	{"ldrt"	,  3	,  0	, 26, 27, 0x00000001, 24, 24, 0x00000000, 20, 22, 0x00000003},
	{"mrc"	,  3	,  6	, 24, 27, 0x0000000e, 20, 20, 0x00000001,  4,  4, 0x00000001},
	{"mcr"	,  3	,  0	, 24, 27, 0x0000000e, 20, 20, 0x00000000,  4,  4, 0x00000001},
	{"msr"	,  2	,  0	, 23, 27, 0x00000006, 20, 21, 0x00000002},
	{"ldrb"	,  3	,  0	, 26, 27, 0x00000001, 22, 22, 0x00000001, 20, 20, 0x00000001},
	{"strb"	,  3	,  0	, 26, 27, 0x00000001, 22, 22, 0x00000001, 20, 20, 0x00000000},
	{"ldr"	,  4	,  0	, 28, 31, 0x0000000e, 26, 27, 0x00000001, 22, 22, 0x00000000, 20, 20, 0x00000001},
	{"ldrcond"	,  3	,  0	, 26, 27, 0x00000001, 22, 22, 0x00000000, 20, 20, 0x00000001},
	{"str"	,  3	,  0	, 26, 27, 0x00000001, 22, 22, 0x00000000, 20, 20, 0x00000000},
	{"cdp"	,  2	,  0	, 24, 27, 0x0000000e,  4,  4, 0x00000000},
	{"stc"	,  2	,  0	, 25, 27, 0x00000006, 20, 20, 0x00000000},
	{"ldc"	,  2	,  0	, 25, 27, 0x00000006, 20, 20, 0x00000001},
	{"swi"	,  1	,  0	, 24, 27, 0x0000000f},
	{"bbl"	,  1	,  0	, 25, 27, 0x00000005},
};

const ISEITEM arm_exclusion_code[] = {
	#define VFP_DECODE_EXCLUSION
	#include "vfp/vfpinstr.c"
	#undef VFP_DECODE_EXCLUSION
	{"srs"	,  0	,  6	,  0},
	{"rfe"	,  0	,  6	,  0},
	{"bkpt"	,  0	,  3	,  0},
	{"blx"	,  0	,  3	,  0},
	{"cps"	,  0	,  6	,  0},
	{"pld"	,  0	,  4	,  0},
	{"setend"	,  0	,  6	,  0},
	{"clrex"	,  0	,  6	,  0},
	{"rev16"	,  0	,  6	,  0},
	{"usad8"	,  0	,  6	,  0},
	{"sxtb"	,  0	,  6	,  0},
	{"uxtb"	,  0	,  6	,  0},
	{"sxth"	,  0	,  6	,  0},
	{"sxtb16"	,  0	,  6	,  0},
	{"uxth"	,  0	,  6	,  0},
	{"uxtb16"	,  0	,  6	,  0},
	{"cpy"	,  0	,  6	,  0},
	{"uxtab"	,  0	,  6	,  0},
	{"ssub8"	,  0	,  6	,  0},
	{"shsub8"	,  0	,  6	,  0},
	{"ssubaddx"	,  0	,  6	,  0},
	{"strex"	,  0	,  6	,  0},
	{"strexb"	,  0	,  7	,  0},
	{"swp"	,  0	,  0	,  0},
	{"swpb"	,  0	,  0	,  0},
	{"ssub16"	,  0	,  6	,  0},
	{"ssat16"	,  0	,  6	,  0},
	{"shsubaddx"	,  0	,  6	,  0},
	{"qsubaddx"	,  0	,  6	,  0},
	{"shaddsubx"	,  0	,  6	,  0},
	{"shadd8"	,  0	,  6	,  0},
	{"shadd16"	,  0	,  6	,  0},
	{"sel"	,  0	,  6	,  0},
	{"saddsubx"	,  0	,  6	,  0},
	{"sadd8"	,  0	,  6	,  0},
	{"sadd16"	,  0	,  6	,  0},
	{"shsub16"	,  0	,  6	,  0},
	{"umaal"	,  0	,  6	,  0},
	{"uxtab16"	,  0	,  6	,  0},
	{"usubaddx"	,  0	,  6	,  0},
	{"usub8"	,  0	,  6	,  0},
	{"usub16"	,  0	,  6	,  0},
	{"usat16"	,  0	,  6	,  0},
	{"usada8"	,  0	,  6	,  0},
	{"uqsubaddx"	,  0	,  6	,  0},
	{"uqsub8"	,  0	,  6	,  0},
	{"uqsub16"	,  0	,  6	,  0},
	{"uqaddsubx"	,  0	,  6	,  0},
	{"uqadd8"	,  0	,  6	,  0},
	{"uqadd16"	,  0	,  6	,  0},
	{"sxtab"	,  0	,  6	,  0},
	{"uhsubaddx"	,  0	,  6	,  0},
	{"uhsub8"	,  0	,  6	,  0},
	{"uhsub16"	,  0	,  6	,  0},
	{"uhaddsubx"	,  0	,  6	,  0},
	{"uhadd8"	,  0	,  6	,  0},
	{"uhadd16"	,  0	,  6	,  0},
	{"uaddsubx"	,  0	,  6	,  0},
	{"uadd8"	,  0	,  6	,  0},
	{"uadd16"	,  0	,  6	,  0},
	{"sxtah"	,  0	,  6	,  0},
	{"sxtab16"	,  0	,  6	,  0},
	{"qadd8"	,  0	,  6	,  0},
	{"bxj"	,  0	,  5	,  0},
	{"clz"	,  0	,  3	,  0},
	{"uxtah"	,  0	,  6	,  0},
	{"bx"	,  0	,  2	,  0},
	{"rev"	,  0	,  6	,  0},
	{"blx"	,  0	,  3	,  0},
	{"revsh"	,  0	,  6	,  0},
	{"qadd"	,  0	,  4	,  0},
	{"qadd16"	,  0	,  6	,  0},
	{"qaddsubx"	,  0	,  6	,  0},
	{"ldrex"	,  0	,  0	,  0},
	{"qdadd"	,  0	,  4	,  0},
	{"qdsub"	,  0	,  4	,  0},
	{"qsub"	,  0	,  4	,  0},
	{"ldrexb"	,  0	,  7	,  0},
	{"qsub8"	,  0	,  6	,  0},
	{"qsub16"	,  0	,  6	,  0},
	{"smuad"	,  0	,  6	,  0},
	{"smmul"	,  0	,  6	,  0},
	{"smusd"	,  0	,  6	,  0},
	{"smlsd"	,  0	,  6	,  0},
	{"smlsld"	,  0	,  6	,  0},
	{"smmla"	,  0	,  6	,  0},
	{"smmls"	,  0	,  6	,  0},
	{"smlald"	,  0	,  6	,  0},
	{"smlad"	,  0	,  6	,  0},
	{"smlaw"	,  0	,  4	,  0},
	{"smulw"	,  0	,  4	,  0},
	{"pkhtb"	,  0	,  6	,  0},
	{"pkhbt"	,  0	,  6	,  0},
	{"smul"	,  0	,  4	,  0},
	{"smlal"	,  0	,  4	,  0},
	{"smla"	,  0	,  4	,  0},
	{"mcrr"	,  0	,  6	,  0},
	{"mrrc"	,  0	,  6	,  0},
	{"cmp"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"tst"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"teq"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"cmn"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"smull"	,  0	,  0	,  0},
	{"umull"	,  0	,  0	,  0},
	{"umlal"	,  0	,  0	,  0},
	{"smlal"	,  0	,  0	,  0},
	{"mul"	,  0	,  0	,  0},
	{"mla"	,  0	,  0	,  0},
	{"ssat"	,  0	,  6	,  0},
	{"usat"	,  0	,  6	,  0},
	{"mrs"	,  0	,  0	,  0},
	{"msr"	,  0	,  0	,  0},
	{"and"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"bic"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"ldm"	,  0	,  0	,  0},
	{"eor"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"add"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"rsb"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"rsc"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"sbc"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"adc"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"sub"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"orr"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"mvn"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"mov"	,  3	,  0	,  4,  4, 0x00000001,  7,  7, 0x00000001, 25, 25, 0x00000000},
	{"stm"	,  0	,  0	,  0},
	{"ldm"	,  0	,  0	,  0},
	{"ldrsh"	,  0	,  2	,  0},
	{"stm"	,  0	,  0	,  0},
	{"ldm"	,  0	,  0	,  0},
	{"ldrsb"	,  0	,  2	,  0},
	{"strd"	,  0	,  4	,  0},
	{"ldrh"	,  0	,  0	,  0},
	{"strh"	,  0	,  0	,  0},
	{"ldrd"	,  0	,  4	,  0},
	{"strt"	,  0	,  0	,  0},
	{"strbt"	,  0	,  0	,  0},
	{"ldrbt"	,  0	,  0	,  0},
	{"ldrt"	,  0	,  0	,  0},
	{"mrc"	,  0	,  6	,  0},
	{"mcr"	,  0	,  0	,  0},
	{"msr"	,  0	,  0	,  0},
	{"ldrb"	,  0	,  0	,  0},
	{"strb"	,  0	,  0	,  0},
	{"ldr"	,  0	,  0	,  0},
	{"ldrcond"	,  1	,  0	,  28, 31, 0x0000000e},
	{"str"	,  0	,  0	,  0},
	{"cdp"	,  0	,  0	,  0},
	{"stc"	,  0	,  0	,  0},
	{"ldc"	,  0	,  0	,  0},
	{"swi"	,  0	,  0	,  0},
	{"bbl"	,  0	,  0	,  0},
        {"bl_1_thumb",      0,      INVALID, 0},/* should be table[-4] */         
        {"bl_2_thumb",      0,      INVALID, 0}, /* should be located at the end of the table[-3] */
	{"blx_1_thumb",      0,      INVALID, 0}, /* should be located at table[-2] */
        {"invalid",      0,      INVALID, 0}         
};

int decode_arm_instr(uint32_t instr, int32_t *idx)
{
	int n = 0;
	int base = 0;
	int ret = DECODE_FAILURE;
	int i = 0;
	int instr_slots = sizeof(arm_instruction)/sizeof(ISEITEM);
	for (i = 0; i < instr_slots; i++)
	{
//		ret = DECODE_SUCCESS;
		n = arm_instruction[i].attribute_value;
		base = 0;
		while (n) {
			if (arm_instruction[i].content[base + 1] == 31 && arm_instruction[i].content[base] == 0) {
				/* clrex */
				if (instr != arm_instruction[i].content[base + 2]) {
					break;
				}
			} else if (BITS(arm_instruction[i].content[base], arm_instruction[i].content[base + 1]) != arm_instruction[i].content[base + 2]) {
				break;
			}
			base += 3;
			n --;
		}
		//All conditions is satisfied.
		if (n == 0)
			ret = DECODE_SUCCESS;

		if (ret == DECODE_SUCCESS) {
			n = arm_exclusion_code[i].attribute_value;
			if (n != 0) {
				base = 0;
				while (n) {
					if (BITS(arm_exclusion_code[i].content[base], arm_exclusion_code[i].content[base + 1]) != arm_exclusion_code[i].content[base + 2]) {
						break;					}
					base += 3;
					n --;
				}
				//All conditions is satisfied.
				if (n == 0)
					ret = DECODE_FAILURE;
			}
		}

		if (ret == DECODE_SUCCESS) {
			*idx = i;
			return ret;
		}
	}
	return ret;
}

