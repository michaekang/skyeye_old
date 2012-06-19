#ifndef _SKYEYE_DISAS_H
#define _SKYEYE_DISAS_H
#include <stdio.h>
void disas(FILE *out, void *code, unsigned long size, unsigned long pc);
void set_thumb_mode(int enabled);
#endif /* _SKYEYE_DISAS_H */
