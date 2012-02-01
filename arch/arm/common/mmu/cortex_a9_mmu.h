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
* @file cortex_a9_mmu.h
* @brief The function list for cortex a9 mmu
* @author Michael.Kang blackfin.kang@gmail.com
* @version 78.77
* @date 2012-02-01
*/
#ifndef __CORTEX_A9_MMU_H__
#define __CORTEX_A9_MMU_H__
extern mmu_ops_t cortex_a9_mmu_ops;

ARMword
cortex_a9_mmu_mrc (ARMul_State *state, ARMword instr, ARMword *value);
#endif
