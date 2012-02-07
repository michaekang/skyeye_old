#include "skyeye_module.h"

const char* skyeye_module = "goldfish_timer";

extern void init_goldfish_timer();

void module_init(){
	init_goldfish_timer();
}

void module_fini(){
}
