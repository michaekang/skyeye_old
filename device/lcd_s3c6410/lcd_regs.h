#ifndef __LCD_REGS_H__
#define __LCD_REGS_H__
#include <skyeye_types.h>
typedef struct s3c_lcd_reg{
	uint32 vidcon[3];
	uint32 vidtcon[3];
	uint32 wincon[5];
	uint32 vidos[5][3];
	uint32 vidosd2d;
	ldi_cmd[12];
}s3c_lcd_reg_t;
#endif
