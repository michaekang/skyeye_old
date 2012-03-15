#include "skyeye_module.h"

const char* skyeye_module = "goldfish_tty";

extern void init_goldfish_tty();

void module_init(){
	init_goldfish_tty();
}

void module_fini(){
}
