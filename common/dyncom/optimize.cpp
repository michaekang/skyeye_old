/*
 * libcpu: optimize.cpp
 *
 * Tell LLVM to run optimizers over the IR.
 */

#include "llvm/ExecutionEngine/ExecutionEngine.h"
#include "llvm/PassManager.h"
#include "llvm/Transforms/IPO.h"
#include "llvm/Analysis/Passes.h"
//#include "llvm/Support/StandardPasses.h"
#include "llvm/Target/TargetData.h"

#include "llvm/DerivedTypes.h"
#include "llvm/ExecutionEngine/ExecutionEngine.h"
#include "llvm/ExecutionEngine/JIT.h"
#include "llvm/LLVMContext.h"
#include "llvm/Module.h"
#include "llvm/PassManager.h"
#include "llvm/Analysis/Verifier.h"
#include "llvm/Analysis/Passes.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Support/IRBuilder.h"
//#include "llvm/Support/TargetSelect.h"


#include "skyeye_dyncom.h"

void
optimize(cpu_t *cpu)
{
	dyncom_engine_t* de = cpu->dyncom_engine;
	FunctionPassManager pm = FunctionPassManager(de->mod);

	std::string data_layout = de->exec_engine->getTargetData()->getStringRepresentation();
	TargetData *TD = new TargetData(data_layout);
	pm.add(TD);
	pm.add(createPromoteMemoryToRegisterPass());
	pm.add(createInstructionCombiningPass());
	pm.add(createConstantPropagationPass());
	pm.add(createDeadCodeEliminationPass());
	pm.run(*de->cur_func);
}

