#ifndef __TOUCHSCREEN_S3C6410_H__
#define __TOUCHSCREEN_S3C6410_H__

#include <gtk/gtk.h>
#include "skyeye_lcd_intf.h"
#include "skyeye_log.h"

typedef char BOOL;

typedef struct touchscreen_reg{
	uint32_t adccon; // 0x7E00_B000
	uint32_t adctsc; // 0x7E00_B004
	uint32_t adcdly; // 0x7E00_B008
	uint32_t adcdat0; // 0x7E00_B00C
	uint32_t adcdat1; // 0x7E00_B010
	uint32_t adcupdn; // 0x7E00_B014
	uint32_t adcclrint; // 0x7E00_B018
	uint32_t adcclrintpndnup; // 0x7E00_B020
}touchscreen_reg_t; 

typedef struct s3c6410_touchscreen_status{
	int x;
	int y;
	int stylus; 	// down, up 
	int event; 	// down, up 
	int adcbit;	// 0 : 10 bit, 1 : 12 bit
	int adc_con;	// 0 : not conversion else 1
}s3c6410_touchscreen_status;

typedef struct s3c6410_touchscreen_device{
	conf_object_t* obj;
	touchscreen_reg_t* regs;
	s3c6410_touchscreen_status* status;
}s3c6410_touchscreen_device;

/* adccon mask */
#define ENABLE_START	0x01 << 0
#define READ_START	0x01 << 1
#define STDBM		0x01 << 2
#define SEL_MUX		0x07 << 3
#define PRSCVL		0x7f << 6
#define PRSCEN		0x01 << 14
#define ECFLG		0x01 << 15
#define RESSEL		0x01 << 16
/* adctsc mask */
#define TSC_XY_PST	0x03 << 0
#define TSC_AUTO_PST	0x01 << 2
#define PULL_UP		0x01 << 3
#define XP_SEN		0x01 << 4
#define XM_SEN		0x01 << 5
#define YP_SEN		0x01 << 6
#define YM_SEN		0x01 << 7
#define UD_SEN		0x01 << 8
/* adcdly mask*/

/* adcdat0 adcdat1 mask */
#define XY_PDATA_10		0x3ff << 0
#define XY_PDATA_12		0xfff << 0
#define DATA_XY_PST			0x003 << 12 
#define DATA_AUTO_PST		0x001 << 14 
#define UPDOWN			0x001 << 15 
/* adcupdn mask */
#define TSC_DN		0x1 << 1
#define TSC_UP		0x1 << 0

#define GET_START(val)		(val >> 17) & 0x1
#define GET_RESSEL(val)		(val && RESSEL) >> 16

#define DBG_TS	0
#define DEBUG_TS(fmt, ...)      if(DBG_TS){							\
					skyeye_log(Debug_log, __func__, fmt, ## __VA_ARGS__);	\
				}
#endif
