#utils = 
libandroid_la_SOURCES = android.c android_module.c


pkglib_LTLIBRARIES = libandroid.la

if WIN32
libandroid_la_LDFLAGS = -no-undefined -shared -module -Xlinker -L../../common/.libs/ -Xlinker -lcommon -Xlinker -lmsvcrt -Xlinker -lkernel32 -Xlinker -lole32 -Xlinker -lkernel32 -Xlinker -lgdi32 -Xlinker -luuid -Xlinker -luser32  -Xlinker -lshell32 -Xlinker -lvfw32 -Xlinker -liphlpapi -Xlinker --export-all-symbol
else
libandroid_la_LDFLAGS = -module
endif

DEFS = -DMODET -g -O3 -D_FILE_OFFSET_BITS=64 -DSTANDALONE -DDEFAULT_INLINE=0 -DNO_LCD @COMMON_CFLAGS@

INCLUDES = -I$(top_srcdir)/arch/arm/common -I$(top_srcdir)/arch/arm/mach/ -I$(top_srcdir)/common/dyncom/  @COMMON_INCLUDES@ -I$(top_srcdir)/third-party/include/
