/*
 * hello.c: 
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
#define BOGO_MIPS 1000000

void hello(void) {
	int i = 0;
	char* hello_str = "helloworld";
	long* paddr = (long*)0x80000070;
	char c = 'w';
	int timeout;

	i = *hello_str - 'h';
	while (1) {
		timeout = 0;
		while(++timeout != BOGO_MIPS);
		for (i = 0; i < 10; i++) {
			*paddr = hello_str[i];
		}
	}

	return;
}
