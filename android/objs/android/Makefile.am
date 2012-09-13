##############################################################################
##############################################################################
###
###  android: LAUNCHER FOR TARGET-SPECIFIC EMULATOR
###
###

android_dir = \
	$(top_srcdir)/android/android.c \
	$(top_srcdir)/android/android_module.c

###dyn
#
libandroid_la_SOURCES = $(android_dir)
if WIN32
libandroid_la_LDFLAGS =  -no-undefined -shared -module -Xlinker -L$(top_srcdir)/common/.libs/ -Xlinker -lcommon -Xlinker -L$(top_srcdir)/android/objs/emulator_libui/.libs/ -Xlinker -lemulator_libui -Xlinker -L$(top_srcdir)/android/objs/emulator_common/.libs/ -Xlinker -lemulator_common -Xlinker -lmsvcrt -Xlinker -lkernel32 -Xlinker -lole32 -Xlinker -lkernel32 -Xlinker -lgdi32 -Xlinker -luuid -Xlinker -lshell32 -Xlinker --export-all-symbol
else
libandroid_la_LDFLAGS = -module -L$(top_srcdir)/common -lcommon  -L$(top_srcdir)/android/objs/emulator_libui -lemulator_libui -L$(top_builddir)/third-party/libiberty/pic
endif
pkglib_LTLIBRARIES = libandroid.la

#libandroid_la_LDFLAGS = -module -L$(top_srcdir)/common -lcommon  -L$(top_srcdir)/android/objs/emulator_libui -lemulator_libui -L$(top_srcdir)/android/objs/emulator_libqemu/ -lemulator_libqemu  -L$(top_srcdir)/android/objs/emulator_common/ -lemulator_common -lrt  -liconv
#libandroid_la_LDFLAGS = -module -L$(top_srcdir)/common -lcommon  $(top_srcdir)/android/objs/emulator_libui/libemulator_libui.a
#libandroid_la_LDFLAGS = -module -L$(top_srcdir)/common -lcommon 
###static
#
#$(top_srcdir)/android/libandroid.a:
#noinst_LIBRARIES += libandroid.a 
#libandroid_a_SOURCES = $(android_dir)
#libandroid_a_LDFLAGS =  $(top_srcdir)/android/libemulator_common.a -L$(top_srcdir)/common -lcommon  -liconv

INCLUDES = -I$(top_srcdir)/common/include/
INCLUDES += -I$(top_srcdir)/android/
#INCLUDES += -I$(top_srcdir)/android/skyeye/
#INCLUDES += -I$(top_srcdir)/android/utils/
#INCLUDES += -I$(top_srcdir)/third-party/distrib/libpng-1.2.19/
#INCLUDES += -I$(top_srcdir)/third-party/distrib/zlib-1.2.3/
#INCLUDES += -I$(top_srcdir)/android/config/linux-x86/
#INCLUDES += -I$(top_srcdir)/third-party/distrib/sdl-1.2.12/include/

