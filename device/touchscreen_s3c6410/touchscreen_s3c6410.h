#ifndef __TOUCHSCREEN_S3C6410_H__
#define __TOUCHSCREEN_S3C6410_H__
typedef struct touchscreen_reg{
	uint32 adccon; // 0x7E00_B000
	uint32 adctsc; // 0x7E00_B004
	uint32 adcdly; // 0x7E00_B008
	uint32 adcdat0; // 0x7E00_B00C
	uint32 adcdat1; // 0x7E00_B010
	uint32 adcupdn; // 0x7E00_B014
	uint32 adcclrint; // 0x7E00_B018
	uint32 adcclrintpndnup; // 0x7E00_B020
}touchscreen_reg_t; 

typedef struct s3c6410_touchscreen_device{
	conf_object_t* obj;
	touchscreen_reg_t* regs;
}s3c6410_touchscreen_device;
#endif
