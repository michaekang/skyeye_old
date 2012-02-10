#utils = 
libandroid_la_SOURCES = android.c android_module.c
libandroid_la_LDFLAGS = -module

pkglib_LTLIBRARIES = libandroid.la

DEFS = -DMODET -g -O3 -D_FILE_OFFSET_BITS=64 -DSTANDALONE -DDEFAULT_INLINE=0 -DNO_LCD @COMMON_CFLAGS@

INCLUDES = -I$(top_srcdir)/arch/arm/common -I$(top_srcdir)/arch/arm/mach/ -I$(top_srcdir)/common/dyncom/  @COMMON_INCLUDES@ -I$(top_srcdir)/third-party/include/
