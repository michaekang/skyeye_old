/* vi: set sw=4 ts=4: */
/*
 * goldfish_bus_module.c: This file is part of ____
 *
 * Copyright (C) 2012 Oubang Shen <shenoubang@gmail.com>
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
 * */
#include "skyeye_module.h"

const char* skyeye_module = "goldfish_bus";

extern void init_goldfish_bus();

void module_init(){
    init_goldfish_bus();
}

void module_fini(){
}

