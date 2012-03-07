/*
 * omap4460x_module.c - register omap4460x soc to common library
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
#include <stdlib.h>
#include <skyeye_module.h>
#include <skyeye_mach.h>

#include "omap4460x.h"

const char* skyeye_module = "omap4460";

void module_init() {
	/* register the soc to the common library*/
	register_mach("omap4460x", omap4460x_mach_init);

	extern void init_omap4460_mach();
	init_omap4460_mach();
}

void module_fini() {

}
