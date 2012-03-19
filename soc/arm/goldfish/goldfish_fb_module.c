#include "skyeye_module.h"

const char* skyeye_module = "goldfish_fb";

extern void init_goldfish_fb();

void module_init(){
	init_goldfish_fb();
}

void module_fini(){
}
