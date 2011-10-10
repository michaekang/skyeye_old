#include "skyeye_module.h"

const char* skyeye_module = "touchscreen_s3c6410";

extern void init_s3c6410_touchscreen();

void module_init(){
	init_s3c6410_touchscreen();
}

void module_fini(){
}
