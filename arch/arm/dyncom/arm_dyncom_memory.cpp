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
* @file memory.cpp
* @brief memory access
* @author Michael.Kang blackfin.kang@gmail.com
* @version 7849
* @date 2012-05-29
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
#include <dyncom/dyncom_llvm.h>
#include <skyeye_log.h>
#include <dyncom/tag.h>

#include "armdefs.h"
#include "dyncom/memory.h"
#include "arm_dyncom_parallel.h"
#include "dyncom/tlb.h"
#include "dyncom/defines.h"
#include "common/mmu/arm1176jzf_s_mmu.h"
#include "armmmu.h"
typedef llvm::ArrayRef<llvm::Type*> TypeArray;
typedef llvm::ArrayRef<llvm::Value*> ValueArray;

static BasicBlock* create_mmu_fault_bb(cpu_t* cpu, int fault, Value* fault_addr){
	BasicBlock *bb = BasicBlock::Create(_CTX(), "mmu_fault", cpu->dyncom_engine->cur_func, 0);
	LET(CP15_TLB_FAULT_STATUS, CONST(fault));
	LET(CP15_TLB_FAULT_ADDR, fault_addr);
	BranchInst::Create(cpu->dyncom_engine->bb_trap, bb);
	return bb;
}

static BasicBlock* create_io_read_bb(cpu_t* cpu, Value* addr, int size, BasicBlock* load_store_end){
	BasicBlock *check_io_bb = BasicBlock::Create(_CTX(), "check_io_tlb", cpu->dyncom_engine->cur_func, 0);
	BasicBlock* bb = check_io_bb;
	#if 0
	Value* result = CONST1(0);

	Value* va = AND(addr, CONST(0xFFFFF000));
	/* get index , index = tlb_cache[access_type][va & 0xff][(va >> 12) % TLB_SIZE]; */
	Value* a = ADD(cpu->dyncom_engine->ptr_io_tlb, ZEXT64(MUL(UREM(LSHR(addr, CONST(12)), CONST(TLB_SIZE)), CONST(sizeof(uint64_t)))));
	a = new IntToPtrInst(a, PointerType::get(XgetType(Int64Ty), 0), "", bb);
	Value* tlb_entry = new LoadInst(a, "", false, bb);
	result = ICMP_EQ(TRUNC32(LSHR(AND(tlb_entry, CONST64(0xFFFFFFFF00000000)), CONST64(32))), va);

	a = ADD(cpu->dyncom_engine->ptr_mixed_tlb, ZEXT64(MUL(UREM(LSHR(addr, CONST(12)), CONST(TLB_SIZE)), CONST(sizeof(uint64_t)))));
	a = new IntToPtrInst(a, PointerType::get(XgetType(Int64Ty), 0), "", bb);
	tlb_entry = SELECT(result, tlb_entry, new LoadInst(a, "", false, bb));
	result = OR(result, ICMP_EQ(TRUNC32(LSHR(AND(tlb_entry, CONST64(0xFFFFFFFF00000000)), CONST64(32))), va));

	//BasicBlock *check_io_tlb_bb = create_io_tlb_bb(cpu, addr, load_store_end);
	//new StoreInst(ret, cpu->dyncom_engine->read_value, false, 0, bb);
	BasicBlock *io_read_bb = BasicBlock::Create(_CTX(), "io_read", cpu->dyncom_engine->cur_func, 0);
	BasicBlock* mmu_fault_bb = create_mmu_fault_bb(cpu, TLB_READ_MISS, addr);
	arch_branch(1, io_read_bb, mmu_fault_bb, result, bb);

	if (cpu->dyncom_engine->ptr_func_read_memory == NULL) {
		return NULL;
	}
	bb = io_read_bb;
	Value* phys_page = TRUNC32(AND(tlb_entry, CONST64(0xFFFFF000)));
	Value* phys_addr = OR(phys_page, AND(CONST(0xFFF), addr));

	//Value* phys_addr = get_phys_addr(cpu, bb, addr, 1, cpu->dyncom_engine->bb_trap);
	#endif
	//Type const *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	IntegerType *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
	Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
	std::vector<Value *> params;
	params.push_back(v_cpu_ptr);
	params.push_back(addr);
	params.push_back(CONST(size));
	params.push_back(CONST(cpu->dyncom_engine->need_exclusive));

	CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_func_read_memory, ValueArray(params), "", bb);
	new StoreInst(ret, cpu->dyncom_engine->read_value, false, 0, bb);
	Value* cond = ICMP_EQ(R(CP15_TLB_FAULT_STATUS), CONST(0));
	//BranchInst::Create(load_store_end, bb);
	arch_branch(1, load_store_end, cpu->dyncom_engine->bb_trap, cond, bb);
	return check_io_bb;
	//new StoreInst(tlb_entry, cpu->dyncom_engine->tlb_entry, false, 0, bb);
}

static BasicBlock* create_io_write_bb(cpu_t* cpu, Value* addr, Value* value, int size, BasicBlock* load_store_end){
	BasicBlock *check_io_bb = BasicBlock::Create(_CTX(), "check_io_tlb", cpu->dyncom_engine->cur_func, 0);
	BasicBlock* bb = check_io_bb;
	#if 0
	Value* result = CONST1(0);
	Value* va = AND(addr, CONST(0xFFFFF000));
	/* get index , index = tlb_cache[access_type][va & 0xff][(va >> 12) % TLB_SIZE]; */
	Value* a = ADD(cpu->dyncom_engine->ptr_io_tlb, ZEXT64(MUL(UREM(LSHR(addr, CONST(12)), CONST(TLB_SIZE)), CONST(sizeof(uint64_t)))));
	a = new IntToPtrInst(a, PointerType::get(XgetType(Int64Ty), 0), "", bb);
	Value* tlb_entry = new LoadInst(a, "", false, bb);
	result = ICMP_EQ(TRUNC32(LSHR(AND(tlb_entry, CONST64(0xFFFFFFFF00000000)), CONST64(32))), va);

	a = ADD(cpu->dyncom_engine->ptr_mixed_tlb, ZEXT64(MUL(UREM(LSHR(addr, CONST(12)), CONST(TLB_SIZE)), CONST(sizeof(uint64_t)))));
	a = new IntToPtrInst(a, PointerType::get(XgetType(Int64Ty), 0), "", bb);
	tlb_entry = SELECT(result, tlb_entry, new LoadInst(a, "", false, bb));
	result = OR(result, ICMP_EQ(TRUNC32(LSHR(AND(tlb_entry, CONST64(0xFFFFFFFF00000000)), CONST64(32))), va));

	//BasicBlock *check_io_tlb_bb = create_io_tlb_bb(cpu, addr, load_store_end);
	//new StoreInst(ret, cpu->dyncom_engine->read_value, false, 0, bb);
	BasicBlock *io_write_bb = BasicBlock::Create(_CTX(), "io_write", cpu->dyncom_engine->cur_func, 0);
	BasicBlock* mmu_fault_bb = create_mmu_fault_bb(cpu, TLB_WRITE_MISS, addr);
	arch_branch(1, io_write_bb, mmu_fault_bb, result, bb);

	bb = io_write_bb;
	Value* phys_page = TRUNC32(AND(tlb_entry, CONST64(0xFFFFF000)));
	Value* phys_addr = OR(phys_page, AND(CONST(0xFFF), addr));
	//arch_arm_debug_print(cpu, bb, ZEXT64(phys_addr), R(15), CONST(18));
	#endif
	if (cpu->dyncom_engine->ptr_func_write_memory == NULL) {
		return NULL;
	}
	//Type const *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	IntegerType *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
	Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
	std::vector<Value *> params;
	params.push_back(v_cpu_ptr);
	params.push_back(addr);
	params.push_back(value);
	params.push_back(CONST(size));
	params.push_back(CONST(cpu->dyncom_engine->need_exclusive));

	CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_func_write_memory, ValueArray(params), "", bb);

	Value* cond = ICMP_EQ(R(CP15_TLB_FAULT_STATUS), CONST(0));
	//BranchInst::Create(load_store_end, bb);
	arch_branch(1, load_store_end, cpu->dyncom_engine->bb_trap, cond, bb);

	//BranchInst::Create(load_store_end, bb);
	return check_io_bb;
}


void memory_read(cpu_t* cpu, BasicBlock*bb, Value* addr, uint32_t sign, uint32_t size){
	int fault = TLB_READ_MISS;
	Value* va =  AND(addr, CONST(0xFFFFF000));
	Value* a = ADD(cpu->dyncom_engine->ptr_data_read_tlb, ZEXT64(MUL(UREM(LSHR(addr, CONST(12)), CONST(TLB_SIZE)), CONST(sizeof(uint64_t)))));
	a = new IntToPtrInst(a, PointerType::get(XgetType(Int64Ty), 0), "", bb);
	Value* tlb_entry = new LoadInst(a, "", false, bb);
	//arch_arm_debug_print(cpu, bb, ZEXT64(addr), R(15), CONST(15));
	//new StoreInst(tlb_entry, cpu->dyncom_engine->tlb_entry, false, 0, bb);
	Value* result = ICMP_EQ(TRUNC32(LSHR(AND(tlb_entry, CONST64(0xFFFFFFFF00000000LL)), CONST64(32))), va);
	BasicBlock *memory_read_bb = BasicBlock::Create(_CTX(), "memory_read", cpu->dyncom_engine->cur_func, 0);
	BasicBlock* load_store_end = BasicBlock::Create(_CTX(), "load_store_end", cpu->dyncom_engine->cur_func, 0);
	//cpu->dyncom_engine->bb_load_store = load_store_bb;
	BasicBlock* io_read_bb = create_io_read_bb(cpu, addr, size, load_store_end);

	arch_branch(1, memory_read_bb, io_read_bb, result, bb);
	bb = memory_read_bb;
	Value* phys_page = TRUNC32(AND(tlb_entry, CONST64(0xFFFFF000)));
	Value* phys_addr = OR(phys_page, AND(CONST(0xFFF), addr));

	//arch_arm_debug_print(cpu, bb, ZEXT64(phys_addr), R(15), CONST(25));
	if(cpu->dyncom_engine->need_exclusive){
		LET(EXCLUSIVE_TAG, phys_addr);
		LET(EXCLUSIVE_STATE, CONST(1));
	}
	Value* tmp;
	/* shenoubang add win32 2012-6-14 */
	#ifndef __WIN32__
	if(size == 8){
		tmp = arch_load8(cpu, phys_addr, bb);
		if(sign)
			tmp = SEXT32(tmp);
		else
			tmp = ZEXT32(tmp); 
	}
	else if(size == 16){
		tmp = arch_load16_aligned(cpu, phys_addr, bb);
		if(sign)
			tmp = SEXT32(tmp);
		else
			tmp = ZEXT32(tmp); 
	}
	else if(size == 32)
		tmp = arch_load32_aligned(cpu, phys_addr, bb);
	else{
		printf("in %s, error size\n", __func__);
		exit(0);
	}
	#else
	//Type const *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	IntegerType *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
	Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
	std::vector<Value *> params;
	params.push_back(v_cpu_ptr);
	params.push_back(phys_addr);
	params.push_back(CONST(size));
	/* shenoubang 2012-6-14 */
	params.push_back(CONST(cpu->dyncom_engine->need_exclusive));
	CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_func_read_memory, ValueArray(params), "", bb);
	tmp = ret;
	#endif
	//arch_arm_debug_print(cpu, bb, ZEXT64(tmp), R(15), CONST(26));
	cpu->dyncom_engine->bb = load_store_end;
	new StoreInst(tmp, cpu->dyncom_engine->read_value, false, 0, bb);
	BranchInst::Create(load_store_end, bb);
	return;
}

void memory_write(cpu_t* cpu, BasicBlock*bb, Value* addr, Value* value, uint32_t size){
	int fault = TLB_WRITE_MISS;
	Value* va =  AND(addr, CONST(0xFFFFF000));
	Value* a = ADD(cpu->dyncom_engine->ptr_data_write_tlb, ZEXT64(MUL(UREM(LSHR(addr, CONST(12)), CONST(TLB_SIZE)), CONST(sizeof(uint64_t)))));
	a = new IntToPtrInst(a, PointerType::get(XgetType(Int64Ty), 0), "", bb);
	Value* tlb_entry = new LoadInst(a, "", false, bb);
	//new StoreInst(tlb_entry, cpu->dyncom_engine->tlb_entry, false, 0, bb);
#if DIFF_WRITE
	Value* result = CONST1(0);
#else
	Value* result = ICMP_EQ(TRUNC32(LSHR(AND(tlb_entry, CONST64(0xFFFFFFFF00000000LL)), CONST64(32))), va);
#endif
	//Value *cond = ICMP_NE(result, CONST1(1));
	//arch_arm_debug_print(cpu, bb, ZEXT64(addr), R(15), CONST(15));
	BasicBlock *memory_write_bb = BasicBlock::Create(_CTX(), "memory_write", cpu->dyncom_engine->cur_func, 0);
	BasicBlock* load_store_end = BasicBlock::Create(_CTX(), "load_store_end", cpu->dyncom_engine->cur_func, 0);
	//cpu->dyncom_engine->bb_load_store = load_store_bb;
	BasicBlock* io_write_bb = create_io_write_bb(cpu, addr, value, size, load_store_end);

	arch_branch(1, memory_write_bb, io_write_bb, result, bb);
	bb = memory_write_bb;
	Value* phys_page = TRUNC32(AND(tlb_entry, CONST64(0xFFFFF000)));
	Value* phys_addr = OR(phys_page, AND(CONST(0xFFF), addr));

	//arch_arm_debug_print(cpu, bb, ZEXT64(phys_addr), R(15), CONST(15));
	//arch_arm_debug_print(cpu, bb, ZEXT64(value), R(15), CONST(16));
	if(cpu->dyncom_engine->need_exclusive){
		//arch_arm_debug_print(cpu, bb, ZEXT64(phys_addr), R(15), CONST(18));
		Value* real_value;
		/* if need exclusive, strore a same value to the address */
		Value* cond = AND(ICMP_EQ(R(EXCLUSIVE_TAG), phys_addr), ICMP_EQ(R(EXCLUSIVE_STATE), CONST(1)));
		LET(EXCLUSIVE_TAG, SELECT(cond, CONST(0xFFFFFFFF), R(EXCLUSIVE_TAG)));
		LET(EXCLUSIVE_STATE, SELECT(cond, CONST(0), R(EXCLUSIVE_STATE)));
		LET(EXCLUSIVE_RESULT, SELECT(cond, CONST(0), CONST(1)));
		Value* data;
		
		if(size == 8){
			data = arch_load8(cpu, phys_addr, bb);
			real_value = SELECT(cond, TRUNC8(value), data);
			//real_value = TRUNC8(value);
		}
		else if(size == 16){
			data = arch_load16_aligned(cpu, phys_addr, bb);
			real_value = SELECT(cond, TRUNC16(value), data);
			//real_value = TRUNC16(value);
		}
		else if(size == 32){
			data = arch_load32_aligned(cpu, phys_addr, bb);
			real_value = SELECT(cond, (value), data);
			//real_value = value;
		}
		else{
			printf("in %s, error size\n", __func__);
			exit(0);
		}
		#if 1
		if(size == 8){
			//STORE8(real_value, phys_addr);
			Value* addr8 = new IntToPtrInst(phys_addr, PointerType::get(XgetType(Int8Ty), 0), "", bb);
			 new StoreInst(real_value, addr8, bb);
		}
		else if(size == 16)
			STORE16(real_value, phys_addr);
		else if(size == 32)
			STORE32(real_value, phys_addr);
		else{
			printf("in %s, error size\n", __func__);
			exit(0);
		}
		#else
		//Type const *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
		IntegerType *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
	        Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
        	Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
	        std::vector<Value *> params;
        	params.push_back(v_cpu_ptr);
	        params.push_back(phys_addr);
		Value* tmp = ZEXT32(real_value);
	        params.push_back(tmp);
	        params.push_back(CONST(size));
		/* shenoubang 2012-6-14 */
		params.push_back(CONST(cpu->dyncom_engine->need_exclusive));
        	CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_func_write_memory, ValueArray(params), "", bb);
		#endif
	}
	else{
		/* shenoubang add win32 2012-6-14 */
	#ifndef __WIN32__
		//arch_arm_debug_print(cpu, bb, ZEXT64(phys_addr), R(15), CONST(17));
		if(size == 8)
			STORE8(value, phys_addr);
		else if(size == 16)
			STORE16(value, phys_addr);
		else if(size == 32)
			STORE32(value, phys_addr);
		else{
			printf("in %s, error size\n", __func__);
			exit(0);
		}
	#else
		//Type const *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
		IntegerType *intptr_type = cpu->dyncom_engine->exec_engine->getTargetData()->getIntPtrType(_CTX());
		Constant *v_cpu = ConstantInt::get(intptr_type, (uintptr_t)cpu);
		Value *v_cpu_ptr = ConstantExpr::getIntToPtr(v_cpu, PointerType::getUnqual(intptr_type));
		std::vector<Value *> params;
		params.push_back(v_cpu_ptr);
		params.push_back(phys_addr);
		params.push_back(value);
		params.push_back(CONST(size));
		/* shenoubang 2012-6-14 */
		params.push_back(CONST(cpu->dyncom_engine->need_exclusive));
		CallInst *ret = CallInst::Create(cpu->dyncom_engine->ptr_func_write_memory, ValueArray(params), "", bb);
	#endif
	}
	//arch_arm_debug_print(cpu, bb, ZEXT64(value), R(15), CONST(16));
	BranchInst::Create(load_store_end, bb);
	cpu->dyncom_engine->bb = load_store_end;
	return;
}
