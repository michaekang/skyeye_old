#include "skyeye_sched.h"
#include "skyeye_io.h"
#include <assert.h>
#define IO_CYCLE_DEFAULT 1000
void register_io_cycle(io_cycle_func_t do_io_cycle,void* arg){
	assert(do_io_cycle);
	uint32 id;
	create_thread_scheduler(IO_CYCLE_DEFAULT, Periodic_sched,do_io_cycle,arg,&id);
}
