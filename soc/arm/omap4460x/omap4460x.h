/*
 * omap4460x.h - definitions of "s3c6410" machine  for skyeye
 *
 * Copyright (C) 2012 Oubang Shen <shenoubang@gmail.com>
 * Skyeye Develop Group, for help please send mail to
 * <skyeye-developer@lists.gro.clinux.org>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __OMAP4460X_H__
#define __OMAP4460X_H__  1

/******************************************/
/***** SYSCTRL_GENERAL_CORE Register  *****/
/******************************************/
#define SYSCTRL_CORE_BASE (0x4A002000)
#define CONTROL_ID_CODE  (SYSCTRL_CORE_BASE + 0x0204)	/* ID_CODE Key Register */


/******************************************/
/********* SCU register     ***************/
/******************************************/
#define SCU_BASE (0x48240000)
#define SCU_CTRL (0x48240000)			/* SCU Control Register */
#define SCU_CONFIG (SCU_BASE + 0x04)	/* SCU Configuration Register */


typedef struct omap4460x_io {

}omap4460x_io_t;

void omap4460x_mach_init(void *arch_instance, machine_config_t *this_mach);
#endif /* __OMAP4460X_H__ */
