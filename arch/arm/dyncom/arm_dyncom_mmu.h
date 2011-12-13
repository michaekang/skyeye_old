#ifndef ARM_DYNCOM_MMU_H
#define ARM_DYNCOM_MMU_H

#include <skyeye_dyncom.h>
#include <skyeye_types.h>
#include <skyeye_obj.h>
#include <skyeye.h>
#include <bank_defs.h>
#include <skyeye_pref.h>

#include "arm_regformat.h"
#include "armdefs.h"
#include "memory.h"
#include "armmmu.h"


#include "dyncom/frontend.h"

#define CP15(idx)	(idx - CP15_BASE)
#define CP15REG(idx)	(core->CP15[CP15(idx)])
#define MMU_ENABLED	core->CP15[CP15(CP15_CONTROL)]

#define MMU_DEBUG	0

extern arch_mem_ops_t arm_dyncom_mem_ops;
/* FIXME, the physical address for s3c6410, should get 
these value from skyeye.conf */
#define BANK0_START 0x50000000
#define BANK0_SIZE 0x10000000
#define BANK0_END (BANK0_START + BANK0_SIZE);

#endif
