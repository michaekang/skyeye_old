#include "skyeye_module.h"

const char* skyeye_module = "sysctrl_s3c6410";

extern void init_s3c6410_sysctrl();

void module_init(){
	init_s3c6410_sysctrl();
}

void module_fini(){
}
