#include "skyeye_module.h"

const char* skyeye_module = "goldfish_events";

extern void init_goldfish_events();

void module_init(){
	init_goldfish_events();
}

void module_fini(){
}
