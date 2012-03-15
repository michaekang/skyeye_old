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
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include "skyeye_arch.h"
#include "stdlib.h"

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

    	if(s->dev >= s->dev_count)
        	return -1;
    	dev = s->dev + s->nand_dev;
    	switch (offset) {
    	case NAND_VERSION:
        	data =  NAND_VERSION_CURRENT;
		break;
    	case NAND_NUM_DEV:
        	data = s->dev_count;
		break;
   	 case NAND_RESULT:
        	data = s->result;
		break;
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
        if(read_len < dev->erase_size) {
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
    if(s->dev >= s->dev_count)
        return 0;
    dev = s->nand_dev + s->dev;

    switch(cmd) {
    case NAND_CMD_GET_DEV_NAME:
        if(size > dev->devname_len)
            size = dev->devname_len;
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
		if(s->dev >= s->dev_count) {
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

#define DEV_COUNT 1
static void nand_finalize_instance(conf_object_t *obj);

static conf_object_t* new_nand_device(char *name){
	nand_dev_controller_t* s = skyeye_mm_zero(sizeof(*s));
	s->obj = new_conf_object(name,s);
	s->io_memory = skyeye_mm_zero(sizeof(memory_space_intf));
	s->io_memory->conf_obj  = s->obj;
	s->io_memory->read = nand_read;
	s->io_memory->write = nand_write;
	s->nand_dev = skyeye_mm_zero(sizeof(nand_dev_t) * DEV_COUNT);
	s->dev_count  = DEV_COUNT;	
	s->nand_dev[DEV_COUNT - 1].devname = strdup(name);
	s->nand_dev[DEV_COUNT - 1].devname_len = strlen(name);
	s->nand_dev[DEV_COUNT- 1].initfile = "initrd.img";
	s->nand_dev[DEV_COUNT - 1].rwfile = "nand.bak";
	nand_finalize_instance(s->obj);
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
