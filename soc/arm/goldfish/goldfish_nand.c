/* 
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2, or (at your option)
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  */

/*
 * author alloc <alloc.young@gmail.com>
 */

#include "skyeye_mm.h"
#include "skyeye_class.h"
#include "goldfish_nand.h"
#include "goldfish_nand_reg.h"
#include "android/utils/tempfile.h"
#include "android/utils/debug.h"
#include "android/android.h"
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdarg.h>
#include "skyeye_arch.h"
#include <skyeye_interface.h>
#include <skyeye_nand_intf.h>
#include "stdlib.h"

static nand_dev_t *nand_devs = NULL;
static uint32_t nand_dev_count = 0;

#define  DEBUG  1
#if DEBUG
#  define  D(...)    VERBOSE_PRINT(init,__VA_ARGS__)
#  define  D_ACTIVE  VERBOSE_CHECK(init)
#  define  T(...)    VERBOSE_PRINT(nand_limits,__VA_ARGS__)
#  define  T_ACTIVE  VERBOSE_CHECK(nand_limits)
#else
#  define  D(...)    ((void)0)
#  define  D_ACTIVE  0
#  define  T(...)    ((void)0)
#  define  T_ACTIVE  0
#endif

/* lseek uses 64-bit offsets on Darwin. */
/* prefer lseek64 on Linux              */
#ifdef __APPLE__
#  define  llseek  lseek
#elif defined(__linux__)
#  define  llseek  lseek64
#endif

#define  XLOG  xlog

static void
xlog( const char*  format, ... )
{
    va_list  args;
    va_start(args, format);
    fprintf(stderr, "NAND: ");
    vfprintf(stderr, format, args);
    va_end(args);
}
static int  do_read(int  fd, void*  buf, size_t  size)
{
    int  ret;
    do {
        ret = read(fd, buf, size);
	//perror("read");
    } while (ret < 0 && errno == EINTR);

    return ret;
}

/* EINTR-proof write - due to SIGALRM in use elsewhere */
static int  do_write(int  fd, const void*  buf, size_t  size)
{
    int  ret;
    do {
        ret = write(fd, buf, size);
    } while (ret < 0 && errno == EINTR);

    return ret;
}

/* EINTR-proof lseek - due to SIGALRM in use elsewhere */
static int  do_lseek(int  fd, off_t offset, int whence)
{
    int  ret;
    do {
        ret = lseek(fd, offset, whence);
//	perror("lseek");
    } while (ret < 0 && errno == EINTR);

    return ret;
}

/* EINTR-proof ftruncate - due to SIGALRM in use elsewhere */
static int  do_ftruncate(int  fd, size_t  size)
{
    int  ret;
    do {
        ret = ftruncate(fd, size);
    } while (ret < 0 && errno == EINTR);

    return ret;
}

static exception_t nand_read(conf_object_t *obj,generic_address_t offset,void* buf,size_t count){
	nand_dev_controller_t* s = obj->obj;
    	nand_dev_t *dev;
	uint32_t data;

    	switch (offset) {
    	case NAND_VERSION:
                *(uint32 *)buf =  NAND_VERSION_CURRENT;
                return No_exp;
    	case NAND_NUM_DEV:
        	//data = s->dev_count;
               *(uint32 *)buf = nand_dev_count;
                return No_exp;
   	 case NAND_RESULT:
                *(uint32 *)buf = s->result;
                return No_exp;
	}

	if(s->dev >= nand_dev_count)
		data = 0;

	dev = nand_devs + s->dev;

    	switch (offset) {
    	case NAND_DEV_FLAGS:
        	data = dev->flags;
		break;
    	case NAND_DEV_NAME_LEN:
		data  =  dev->devname_len;
		break;
    	case NAND_DEV_PAGE_SIZE:
        	data =  dev->page_size;
		break;
    	case NAND_DEV_EXTRA_SIZE:
        	data =  dev->extra_size;
		break;
    	case NAND_DEV_ERASE_SIZE:
        	data = dev->erase_size;
		break;
    	case NAND_DEV_SIZE_LOW:
        	data = (uint32_t)dev->max_size;
		break;
   	 case NAND_DEV_SIZE_HIGH:
        	data = (uint32_t)(dev->max_size >> 32);
		break;
    	default:
        	fprintf(stderr, "in %s: Bad offset %x\n",__FUNCTION__, offset);
	}
	*(uint32 *)buf = data;
        return No_exp;
}

static uint32_t nand_dev_read_file(nand_dev_t *dev, uint32_t data, uint64_t addr, uint32_t total_len)
{
    uint32_t len = total_len;
    size_t read_len = dev->erase_size;
    uint32_t dst  = data;
    int ret  = 0;
    int eof = 0;
    int i;
  //  NAND_UPDATE_READ_THRESHOLD(total_len);

    generic_arch_t *arch = get_arch_instance("");
    ret = do_lseek(dev->fd, addr, SEEK_SET);
    while(len > 0) {
        if(read_len < dev->erase_size) { // read all
            memset(dev->data, 0xff, dev->erase_size);
            read_len = dev->erase_size;
            eof = 1;
        }
        if(len < read_len)
            read_len = len;
        if(!eof) {
            read_len = do_read(dev->fd, dev->data, read_len);
	}
	for(i = 0; i< read_len;i++){
		arch->mmu_write(8,(dst + i),dev->data[i]);
	}/*
	if(addr == 5280){
		printf("readlen 0x%x,readlen 0x%x\n",total_len,read_len);
		for(i = 0; i < 0x200;i++){
			printf("offset %d,value 0x%x\n",i ,dev->data[ i]);
		}
	}*/
        dst += read_len;
        len -= read_len;
    }
    return total_len;
}
static uint32_t nand_dev_write_file(nand_dev_t *dev, uint32_t data, uint64_t addr, uint32_t total_len)
{
    uint32_t len = total_len;
    size_t write_len = dev->erase_size;
    int ret;
	uint32_t i;
	generic_arch_t *arch = get_arch_instance("");
//    NAND_UPDATE_WRITE_THRESHOLD(total_len);

    do_lseek(dev->fd, addr, SEEK_SET);
    while(len > 0) {
        if(len < write_len)
            write_len = len;
    	for(i = 0; i < write_len; i++){
		arch->mmu_read(8,(data + i),&(dev->data[i]));
	}
        ret = do_write(dev->fd, dev->data, write_len);
        if(ret < write_len) {
            fprintf(stderr,"nand_dev_write_file, write failed: %s\n", strerror(errno));
            break;
        }
        data += write_len;
        len -= write_len;
    }
    return total_len - len;
}

static uint32_t nand_dev_erase_file(nand_dev_t *dev, uint64_t addr, uint32_t total_len)
{
    uint32_t len = total_len;
    size_t write_len = dev->erase_size;
    int ret;

    do_lseek(dev->fd, addr, SEEK_SET);
    memset(dev->data, 0xff, dev->erase_size);
    while(len > 0) {
        if(len < write_len)
            write_len = len;
        ret = do_write(dev->fd, dev->data, write_len);
        if(ret < write_len) {
            fprintf(stderr, "nand_dev_write_file, write failed: %s\n", strerror(errno));
            break;
        }
        len -= write_len;
    }
    return total_len - len;
}
static uint32_t nand_dev_do_cmd(nand_dev_controller_t *s, uint32_t cmd)
{
    uint32_t size;
    uint64_t addr;
    nand_dev_t  *dev;

    addr = s->addr_low | ((uint64_t)s->addr_high << 32);
    size = s->transfer_size;
    if(s->dev >= nand_dev_count)
        return 0;
    dev = nand_devs + s->dev;

    switch(cmd) {
    case NAND_CMD_GET_DEV_NAME:
	printf("size is %d,devname_len %d,devname %s\n",size,dev->devname_len,dev->devname);
        if(size > dev->devname_len)
            size = dev->devname_len;

        int fault;
        skyeye_config_t* config = get_current_config();
        generic_arch_t *arch_instance = get_arch_instance(config->arch->arch_name);
        fault = arch_instance->mmu_write(8, addr, (uint32_t *)dev->devname);
        if(fault)
                fprintf(stderr, "SKYEYE:write virtual address error!!!\n");

        return size;
    case NAND_CMD_READ:
        if(addr >= dev->max_size)
            return 0;
        if(size > dev->max_size - addr)
            size = dev->max_size - addr;
        if(dev->fd >= 0)
            return nand_dev_read_file(dev, s->data, addr, size);
    case NAND_CMD_WRITE:
        if(dev->flags & NAND_DEV_FLAG_READ_ONLY)
            return 0;
        if(addr >= dev->max_size)
            return 0;
        if(size > dev->max_size - addr)
            size = dev->max_size - addr;
        if(dev->fd >= 0)
            return nand_dev_write_file(dev, s->data, addr, size);
        return size;
    case NAND_CMD_ERASE:
        if(dev->flags & NAND_DEV_FLAG_READ_ONLY)
            return 0;
        if(addr >= dev->max_size)
            return 0;
        if(size > dev->max_size - addr)
            size = dev->max_size - addr;
        if(dev->fd >= 0)
            return nand_dev_erase_file(dev, addr, size);
        memset(&dev->data[addr], 0xff, size);
        return size;
    case NAND_CMD_BLOCK_BAD_GET: // no bad block support
        return 0;
    case NAND_CMD_BLOCK_BAD_SET:
        if(dev->flags & NAND_DEV_FLAG_READ_ONLY)
            return 0;
    default:
        fprintf(stderr, "nand_dev_do_cmd: Bad command %x\n", cmd);
        return 0;
    }
}
static exception_t nand_write(conf_object_t *obj,generic_address_t addr,void* buf,size_t count){
	nand_dev_controller_t *s = obj->obj;
	uint32_t value = *(uint32_t *)buf;
	exception_t exp = No_exp;
	switch (addr) {
	case NAND_DEV:
		s->dev = value;
		if(s->dev >= nand_dev_count) {
			fprintf(stderr,"in %s,bad dev %x\n",__FUNCTION__,value);
		}
		break;
	case NAND_ADDR_HIGH:
		s->addr_high = value;
		break;
	case NAND_ADDR_LOW:
		s->addr_low = value;
		break;
	case NAND_TRANSFER_SIZE:
		s->transfer_size = value;
		break;
	case NAND_DATA:
		s->data = value;
		break;
	case NAND_COMMAND:
		s->result = nand_dev_do_cmd(s,value);
		break;
	default:
		fprintf(stderr,"in %s : BAD offset %x\n",__FUNCTION__,addr);
		exp = -1;
		break;
	}
	return exp;
}

static int arg_match(const char *a, const char *b, size_t b_len)
{
    while(*a && b_len--) {
        if(*a++ != *b++)
            return 0;
    }
    return b_len == 0;
}

void nand_add_dev(const char *arg)
{
    uint64_t dev_size = 0;
    const char *next_arg;
    const char *value;
    size_t arg_len, value_len;
    nand_dev_t *new_devs, *dev;
    char *devname = NULL;
    size_t devname_len = 0;
    char *initfilename = NULL;
    char *rwfilename = NULL;
    int initfd = -1;
    int rwfd = -1;
    int read_only = 0;
    int pad;
    ssize_t read_size;
    //uint32_t page_size = 2048;
#if 0
    uint32_t page_size = 512;
    uint32_t extra_size = 16;
    uint32_t erase_pages = 16;
#endif

    uint32_t page_size = 2048;
    uint32_t extra_size = 64;
    uint32_t erase_pages = 64;

    VERBOSE_PRINT(init, "%s: %s\n", __FUNCTION__, arg);
    printf("%s: %s,nand_dev_count %d\n", __FUNCTION__, arg,nand_dev_count);

    while(arg) {
        next_arg = strchr(arg, ',');
        value = strchr(arg, '=');
        if(next_arg != NULL) {
            arg_len = next_arg - arg;
            next_arg++;
            if(value >= next_arg)
                value = NULL;
        }
        else
            arg_len = strlen(arg);
        if(value != NULL) {
            size_t new_arg_len = value - arg;
            value_len = arg_len - new_arg_len - 1;
            arg_len = new_arg_len;
            value++;
        }
        else
            value_len = 0;

        if(devname == NULL) {
            if(value != NULL)
                goto bad_arg_and_value;
            devname_len = arg_len;
            devname = malloc(arg_len+1);
            if(devname == NULL)
                goto out_of_memory;
            memcpy(devname, arg, arg_len);
            devname[arg_len] = 0;
        }
        else if(value == NULL) {
            if(arg_match("readonly", arg, arg_len)) {
                read_only = 1;
            }
            else {
                XLOG("bad arg: %.*s\n", arg_len, arg);
                exit(1);
            }
        }
        else {
            if(arg_match("size", arg, arg_len)) {
                char *ep;
                dev_size = strtoull(value, &ep, 0);
                if(ep != value + value_len)
                    goto bad_arg_and_value;
            }
            else if(arg_match("pagesize", arg, arg_len)) {
                char *ep;
                page_size = strtoul(value, &ep, 0);
                if(ep != value + value_len)
                    goto bad_arg_and_value;
            }
            else if(arg_match("extrasize", arg, arg_len)) {
                char *ep;
                extra_size = strtoul(value, &ep, 0);
                if(ep != value + value_len)
                    goto bad_arg_and_value;
            }
            else if(arg_match("erasepages", arg, arg_len)) {
                char *ep;
                erase_pages = strtoul(value, &ep, 0);
                if(ep != value + value_len)
                    goto bad_arg_and_value;
            }
            else if(arg_match("initfile", arg, arg_len)) {
                initfilename = malloc(value_len + 1);
                if(initfilename == NULL)
                    goto out_of_memory;
                memcpy(initfilename, value, value_len);
                initfilename[value_len] = '\0';
            }
            else if(arg_match("file", arg, arg_len)) {
                rwfilename = malloc(value_len + 1);
                if(rwfilename == NULL)
                    goto out_of_memory;
                memcpy(rwfilename, value, value_len);
                rwfilename[value_len] = '\0';
            }
            else {
                goto bad_arg_and_value;
            }
        }

        arg = next_arg;
    }

    if (rwfilename == NULL) {
        /* we create a temporary file to store everything */
        TempFile*    tmp = tempfile_create();

        if (tmp == NULL) {
            XLOG("could not create temp file for %.*s NAND disk image: %s\n",
                  devname_len, devname, strerror(errno));
            exit(1);
        }
        rwfilename = (char*) tempfile_path(tmp);
        if (VERBOSE_CHECK(init))
            dprint( "mapping '%.*s' NAND image to %s", devname_len, devname, rwfilename);
    }

#ifndef O_BINARY
#define O_BINARY 0
    if(rwfilename) {
        rwfd = open(rwfilename, O_BINARY | (read_only ? O_RDONLY : O_RDWR));
        if(rwfd < 0) {
            XLOG("could not open file %s, %s\n", rwfilename, strerror(errno));
            exit(1);
        }
        /* this could be a writable temporary file. use atexit_close_fd to ensure
         * that it is properly cleaned up at exit on Win32
         */
        if (!read_only)
            atexit_close_fd(rwfd);
    }

    if(initfilename) {
        initfd = open(initfilename, O_BINARY | O_RDONLY);
        if(initfd < 0) {
            XLOG("could not open file %s, %s\n", initfilename, strerror(errno));
            exit(1);
        }
        if(dev_size == 0) {
            dev_size = do_lseek(initfd, 0, SEEK_END);
            do_lseek(initfd, 0, SEEK_SET);
        }
    }
#undef O_BINARY
#endif

    new_devs = realloc(nand_devs, sizeof(nand_devs[0]) * (nand_dev_count + 1));
    if(new_devs == NULL)
        goto out_of_memory;
    nand_devs = new_devs;
    dev = &new_devs[nand_dev_count];

    dev->page_size = page_size;
    dev->extra_size = extra_size;
    dev->erase_size = erase_pages * (page_size + extra_size);
    pad = dev_size % dev->erase_size;
    if (pad != 0) {
        dev_size += (dev->erase_size - pad);
        D("rounding devsize up to a full eraseunit, now %llx\n", dev_size);
    }
    dev->devname = devname;
    dev->devname_len = devname_len;
    dev->initfile = initfilename; 
    dev->rwfile = rwfilename; 
    dev->max_size = dev_size;
    dev->data = malloc(dev->erase_size);
    if(dev->data == NULL)
        goto out_of_memory;
    dev->flags = read_only ? NAND_DEV_FLAG_READ_ONLY : 0;

    if (initfd >= 0) {
        do {
            read_size = do_read(initfd, dev->data, dev->erase_size);
            if(read_size < 0) {
                XLOG("could not read file %s, %s\n", initfilename, strerror(errno));
                exit(1);
            }
            if(do_write(rwfd, dev->data, read_size) != read_size) {
                XLOG("could not write file %s, %s\n", rwfilename, strerror(errno));
                exit(1);
            }
        } while(read_size == dev->erase_size);
        close(initfd);
    }
    dev->fd = rwfd;

    nand_dev_count++;

    return;

out_of_memory:
    XLOG("out of memory\n");
    exit(1);

bad_arg_and_value:
    XLOG("bad arg: %.*s=%.*s\n", arg_len, arg, value_len, value);
    exit(1);
}

static conf_object_t* new_nand_device(char *name){
	nand_dev_controller_t* s = skyeye_mm_zero(sizeof(*s));
	s->obj = new_conf_object(name,s);
	s->io_memory = skyeye_mm_zero(sizeof(memory_space_intf));
	s->io_memory->conf_obj  = s->obj;
	s->io_memory->read = nand_read;
	s->io_memory->write = nand_write;

	goldfish_nand_control_intf * nand_control = skyeye_mm_zero(sizeof(goldfish_nand_control_intf));
	nand_control->conf_obj = s->obj;
	nand_control->nand_ctrl = nand_add_dev;
	SKY_register_interface(nand_control, name, NAND_CTRL_INTF_NAME);
#define O_BINARY 0
	int initfd;
	uint32_t dev_size;
	initfd  = open("/home/xiaoqiao/develop/android_ui_qemu//ramdisk.img",O_BINARY | O_RDONLY);
	dev_size = do_lseek(initfd,0,SEEK_END);
	do_lseek(initfd,0,SEEK_SET);
	printf("ramdisk size 0x%x\n",dev_size);
	char * ramdisk_arg = "ramdisk,size=0x24625,file=/home/xiaoqiao/develop/android_ui_qemu//ramdisk.img";
	nand_add_dev(ramdisk_arg);


#if 0
	s->nand_dev = skyeye_mm_zero(sizeof(nand_dev_t) * DEV_COUNT);
	s->dev_count  = DEV_COUNT;	
	s->nand_dev[DEV_COUNT - 1].devname = strdup(name);
	s->nand_dev[DEV_COUNT - 1].devname_len = strlen(name);
	s->nand_dev[DEV_COUNT- 1].initfile = "initrd.img";
	s->nand_dev[DEV_COUNT - 1].rwfile = "nand.bak";
	nand_finalize_instance(s->obj);
#endif
	return s->obj;
}

static void nand_finalize_instance(conf_object_t *obj){
	nand_dev_controller_t* s = obj->obj;
	nand_dev_t *dev  = &s->nand_dev[s->dev];
	dev->devname_len = strlen(dev->devname);
	uint32_t dev_siz;
	int initfd;
	int rwfd;
	int read_size;
#ifndef O_BINARY
#define O_BINARY 0
	if(dev->initfile){
		initfd  = open(dev->initfile,O_BINARY | O_RDONLY);
		if(initfd < 0){
			fprintf(stderr,"could not open file %s in %s\n",dev->initfile,__FUNCTION__);
			//skyeye_exit(-10);
		}
		if(s->dev_size == 0){
			s->dev_size = do_lseek(initfd,0,SEEK_END);
			do_lseek(initfd,0,SEEK_SET);
		}
	}else {
		fprintf(stderr,"you need to tell initfile\n");
	}	
	if(dev->rwfile){
		rwfd = open(dev->rwfile,O_BINARY | ((dev->read_only) ? O_RDONLY : O_RDWR ));
		if(rwfd < 0) {
			fprintf(stderr,"can not open rwfile %s\n",dev->rwfile);
		
		}
	}
#undef O_BINARY
#endif
	uint32_t page_size = dev->page_size;
	if(!page_size){
		page_size = 512;
		dev->page_size = page_size;
	}
	uint32_t extra_size = dev->extra_size;
	if(!extra_size){
		extra_size  = 16;
		dev->extra_size  = extra_size;
	}
	dev->erase_size  = 32 * (page_size + extra_size);
	uint32_t pad = s->dev_size % dev->erase_size;
	if(pad){
		s->dev_size += (dev->erase_size - pad);
	}
	dev->max_size = s->dev_size;
	dev->data = skyeye_mm(dev->erase_size);
	if(!dev->data){
		fprintf(stderr,"mm alloc failed\n");
	} 
	dev->flags = dev->read_only ? NAND_DEV_FLAG_READ_ONLY : 0;
	if(initfd >= 0){
		do {
			read_size  = do_read(initfd,dev->data,dev->erase_size);
			if(read_size < 0){
				fprintf(stderr,"could not read file %s\n",dev->initfile);
			}
			if(do_write(rwfd,dev->data,read_size) != read_size){
				fprintf(stderr,"could not write file %s\n",dev->rwfile);
			}
		}while(read_size == dev->erase_size);
		close(initfd);
	}
	dev->fd  = rwfd;

}

static void del_nand(char *obj){
}

void nand_device_init(){
	static skyeye_class_t goldfish_nand_cls = {
		.class_name = "goldfish_nand",
		.class_desc = "goldfish nand",
		.new_instance = new_nand_device,
		.free_instance = del_nand,
		.get_attr = NULL,
		.set_attr = NULL
	};
	SKY_register_class(goldfish_nand_cls.class_name,&goldfish_nand_cls);
}
