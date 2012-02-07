#include "skyeye_module.h"

const char* skyeye_module = "goldfish_pic";

extern void init_goldfish_pic();

void module_init(){
	init_goldfish_pic();
}

void module_fini(){
}
