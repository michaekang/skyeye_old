/* Copyright (C) 2007-2008 The Android Open Source Project
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
*/
//#define DEBUG
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <bank_defs.h>
#include <skyeye_sched.h>
#include <skyeye_options.h>
#include <skyeye_config.h>
#include <skyeye_command.h>
#include <skyeye_uart_ops.h>
#include <skyeye_class.h>
#include <skyeye_mm.h>
#include <skyeye_arch.h>

#include "goldfish_tty.h"

enum {
    TTY_PUT_CHAR       = 0x00,
    TTY_BYTES_READY    = 0x04,
    TTY_CMD            = 0x08,

    TTY_DATA_PTR       = 0x10,
    TTY_DATA_LEN       = 0x14,

    TTY_CMD_INT_DISABLE    = 0,
    TTY_CMD_INT_ENABLE     = 1,
    TTY_CMD_WRITE_BUFFER   = 2,
    TTY_CMD_READ_BUFFER    = 3,
};


static exception_t tty_read(conf_object_t *opaque, generic_address_t offset, void* value, size_t count)
{
    goldfish_tty_device *dev = (goldfish_tty_device*)(opaque->obj);
    tty_state_t* s = dev->tty;
    switch (offset) {
        case TTY_BYTES_READY:
            return s->data_count;
    default:
        //cpu_abort (cpu_single_env, "goldfish_tty_read: Bad offset %x\n", offset);
        return 0;
    }
    return No_exp;
}

static exception_t tty_write(conf_object_t *opaque, generic_address_t offset, uint32_t* value, size_t count)
{
    goldfish_tty_device *dev = (goldfish_tty_device*)(opaque->obj);
    tty_state_t* s = dev->tty;
    switch(offset) {
        case TTY_PUT_CHAR: {
           // uint8_t ch = value;
	    char ch = *(char *)value;
	    skyeye_uart_write(-1, &ch, 1, NULL);
#if 0
            if(s->cs)
                qemu_chr_write(s->cs, &ch, 1);
#endif
        } break;

        case TTY_CMD:
            switch(*value) {
                case TTY_CMD_INT_DISABLE:
                    if(s->ready) {
                        if(s->data_count > 0)
			    dev->master->lower_signal(dev->signal_target, dev->line_no);
                        s->ready = 0;
                    }
                    break;

                case TTY_CMD_INT_ENABLE:
                    if(!s->ready) {
                        if(s->data_count > 0)
			    dev->master->raise_signal(dev->signal_target, dev->line_no);
                        s->ready = 1;
                    }
                    break;

                case TTY_CMD_WRITE_BUFFER:
		    {
                        int len;
			uint32_t buf;

                        buf = s->ptr;
                        len = s->ptr_len;

                        while (len) {
                            char   temp[1];
#if 0
                            int    to_write = sizeof(temp);
			    printf("to_write is %d\n",to_write);
                            if (to_write > len)
                                to_write = len;
#endif

#if 0
#ifdef TARGET_I386
                            if (kvm_enabled())
                                cpu_synchronize_state(cpu_single_env, 0);
#endif
                            cpu_memory_rw_debug(cpu_single_env, buf, (uint8_t*)temp, to_write, 0);
                            qemu_chr_write(s->cs, (const uint8_t*)temp, to_write);
#endif
			    int fault;
			    skyeye_config_t* config = get_current_config();
			    generic_arch_t *arch_instance = get_arch_instance(config->arch->arch_name);
			    fault = arch_instance->mmu_read(8, buf, (uint32_t *)temp);
			    if(fault)
				fprintf(stderr, "SKYEYE:read virtual address 0x%x error!!!\n",buf );
			    buf +=1;
			    len -=1;

			    skyeye_uart_write(-1, (void *)temp, 1, NULL);
//                            buf += to_write;
 //                           len -= to_write;
                        }
                        //printf("goldfish_tty_write: got %d bytes from %x\n", s->ptr_len, s->ptr);
		    }
                    break;

                case TTY_CMD_READ_BUFFER:
		    {
#if 0
                    if(s->ptr_len > s->data_count)
                        cpu_abort (cpu_single_env, "goldfish_tty_write: reading more data than available %d %d\n", s->ptr_len, s->data_count);
#ifdef TARGET_I386
                    if (kvm_enabled())
                        cpu_synchronize_state(cpu_single_env, 0);
#endif
                    cpu_memory_rw_debug(cpu_single_env,s->ptr, s->data, s->ptr_len,1);
#endif
                    //printf("goldfish_tty_write: read %d bytes to %x\n", s->ptr_len, s->ptr);
                    if(s->ptr_len > s->data_count)
			fprintf(stderr, "goldfish_tty_write: reading more data than available %d %d\n", s->ptr_len, s->data_count);
		    int fault;
		    char   temp[1];
		    int i;
		    skyeye_config_t* config = get_current_config();
		    generic_arch_t *arch_instance = get_arch_instance(config->arch->arch_name);
		    temp[0] = (char)(s->data)[0];
		    fault = arch_instance->mmu_write(8, s->ptr, (uint32_t)temp[0]);
		    if(fault)
			fprintf(stderr, "SKYEYE:read virtual address error!!!\n" );

		    for (i = 0;i < s->data_count;i++)
			    printf("data[%d],%c\n",i,(char)(s->data)[i]);

                   if(s->data_count > s->ptr_len)
                        memmove(s->data, s->data + s->ptr_len, s->data_count - s->ptr_len);
                    s->data_count -= 1;
                    if(s->data_count == 0 && s->ready)
			 dev->master->lower_signal(dev->signal_target, dev->line_no);
		    }
                    break;
                default:
                    //cpu_abort (cpu_single_env, "goldfish_tty_write: Bad command %x\n", value);
		    break;
            };
            break;

        case TTY_DATA_PTR:
            s->ptr = *value;
            break;

        case TTY_DATA_LEN:
            s->ptr_len = *value;
            break;

        default:
            //cpu_abort (cpu_single_env, "goldfish_tty_write: Bad offset %x\n", offset);
	    break;
    }
    return No_exp;
}

#if 0
static int tty_can_receive(void *opaque)
{
    struct tty_state *s = opaque;

    return (sizeof(s->data) - s->data_count);
}

static void tty_receive(void *opaque, const uint8_t *buf, int size)
{
    struct tty_state *s = opaque;

    memcpy(s->data + s->data_count, buf, size);
    s->data_count += size;
    if(s->data_count > 0 && s->ready)
        goldfish_device_set_irq(&s->dev, 0, 1);
}
#endif

static void tty_io_do_cycle(void* tty_dev){
	goldfish_tty_device* dev = (goldfish_tty_device*)tty_dev;
	tty_state_t* s = dev->tty;

	struct timeval tv;
	unsigned char buf;

	tv.tv_sec = 0;
	tv.tv_usec = 0;

	if(skyeye_uart_read(-1, &buf, 1, &tv, NULL) > 0)
	{
	        memcpy(s->data + s->data_count, &buf, 1);
		s->data_count += 1;
		printf("in %s,%c\n",__func__,buf);
		//dev->master->lower_signal(dev->signal_target, dev->line_no);
		if(s->data_count > 0 && s->ready)
		{
			printf("tty send interrupt\n");
			dev->master->raise_signal(dev->signal_target, dev->line_no);
		}
	}


}
static conf_object_t* new_goldfish_tty_device(char* obj_name){
	goldfish_tty_device* dev = skyeye_mm_zero(sizeof(goldfish_tty_device));
	dev->obj = new_conf_object(obj_name, dev);
	tty_state_t* tty =  skyeye_mm_zero(sizeof(tty_state_t));
	tty->dev = dev;
	//tty_state.tty = qemu_new_tty_ns(vm_clock, goldfish_tty_tick, &tty_state);

	dev->tty = tty;
	dev->io_memory = skyeye_mm_zero(sizeof(memory_space_intf));
	dev->io_memory->conf_obj = dev->obj;
	dev->io_memory->read = tty_read;
	dev->io_memory->write = tty_write;
	uint32 id;
//	create_timer_scheduler(1, Periodic_sched, tty_io_do_cycle, dev, &id);
	create_thread_scheduler(1000, Periodic_sched,  tty_io_do_cycle, dev, &id);
	return dev->obj;
}
static void del_goldfish_tty_device(conf_object_t* dev){
	
}

void goldfish_tty_device_init()
{
}

void init_goldfish_tty(){
	static skyeye_class_t class_data = {
		.class_name = "goldfish_tty",
		.class_desc = "goldfish tty",
		.new_instance = new_goldfish_tty_device,
		.free_instance = del_goldfish_tty_device,
		.get_attr = NULL,
		.set_attr = NULL
	};
	SKY_register_class(class_data.class_name,&class_data);
}

