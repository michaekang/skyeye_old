/* 
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2, or (at your option)
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  */

/*
 * author alloc <alloc.young@gmail.com>
 */
#ifndef __GOLDFISH_NAND__
#define __GOLDFISH_NAND__

#include "skyeye_types.h"
#include "memory_space.h"

typedef struct {
	char* devname;
	size_t devname_len;
	uint8_t* data;
	uint32_t flags;
	uint32_t page_size;
	uint32_t extra_size;
	uint32_t erase_size;
	uint64_t max_size;
	uint32_t read_only;
	char* rwfile;
	char* initfile;
	int fd;
}nand_dev_t;

typedef struct {
	conf_object_t *obj;
	uint32_t base;
	uint32_t dev;
	uint32_t addr_low;
	uint32_t addr_high;
	uint32_t transfer_size;
	uint32_t data;
	uint32_t result;
	uint32_t dev_size;
	memory_space_intf* io_memory;
	uint32_t dev_count;
	nand_dev_t *nand_dev;
}nand_dev_controller_t;
#endif
