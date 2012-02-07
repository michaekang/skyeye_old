#include <stdlib.h>
#include <skyeye_module.h>
#include <skyeye_mach.h>

const char* skyeye_module = "goldfish";

extern void goldfish_mach_init();

void module_init(){
        /*
         * register the soc to the common library.
         */
	register_mach("goldfish", goldfish_mach_init);
}

void module_fini(){
}
