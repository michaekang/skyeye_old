#!/bin/sh
#mkimage -A arm -O linux -T kernel -C none -a 0x50008000 -e 0x50008040 -n 'Linux-3.0.0' -d zImage uImage
#gcc -Wall -g -o mknandflashdump mknandflashdump.c && chmod 666 nand.dump && ./mknandflashdump ramdisk.img  nand.dump 0  yaffs && ./mknandflashdump system.img nand.dump 0x100000 yaffs && ./mknandflashdump userdata.img nand.dump 0x8800000 yaffs 

gcc -Wall -g -o mknandflashdump mknandflashdump.c && chmod 666 nand.dump && ./mknandflashdump ramdisk.img  nand.dump 0  yaffs 0x600000 && ./mknandflashdump system.img nand.dump 0x600000 yaffs 0x6000000 && ./mknandflashdump userdata.img nand.dump 0x6600000 yaffs  0x1a00000  

