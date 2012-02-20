#include "skyeye_module.h"

const char* skyeye_module = "uart_16550";

extern void init_16550_uart();

void module_init(){
	init_16550_uart();
}

void module_fini(){
}
