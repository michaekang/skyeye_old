/*skyeye_io.h provide do_io_cycle function
 *
 *
 *
 *
 * */
#ifndef __SKYEYE_IO_H__
#define __SKYEYE_IO_H__
#include <skyeye_types.h>
typedef void (*io_cycle_func_t)(void *arg);
void register_io_cycle(io_cycle_func_t func,void* arg);
#endif
