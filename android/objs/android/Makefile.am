##############################################################################
##############################################################################
###
###  android: LAUNCHER FOR TARGET-SPECIFIC EMULATOR
###
###

android_dir = \
	$(top_srcdir)/android/skyeye/user-events-qemu.c \
	$(top_srcdir)/android/cmdline-option.c \
	$(top_srcdir)/android/console.c \
	$(top_srcdir)/android/display.c \
	$(top_srcdir)/android/display-core.c \
	$(top_srcdir)/android/help.c \
	$(top_srcdir)/android/main-common.c \
	$(top_srcdir)/android/android_main.c \
	$(top_srcdir)/android/protocol/core-commands-qemu.c \
	$(top_srcdir)/android/protocol/ui-commands-qemu.c \
	$(top_srcdir)/android/main-emulator.c \
	$(top_srcdir)/android/android.c \
	$(top_srcdir)/android/android_module.c

#utils/main/skyeye.c
###dyn
#
libandroid_la_SOURCES = $(android_dir)
libandroid_la_LDFLAGS = -module -L$(top_srcdir)/common -lcommon  -L$(top_srcdir)/android/objs/emulator_libui -lemulator_libui -L$(top_srcdir)/android/objs/emulator_libqemu/ -lemulator_libqemu  -L$(top_srcdir)/android/objs/emulator_common/ -lemulator_common -lrt  -liconv

###static
#
#$(top_srcdir)/android/libandroid.a:
#noinst_LIBRARIES += libandroid.a 
#libandroid_a_SOURCES = $(android_dir)
#libandroid_a_LDFLAGS =  $(top_srcdir)/android/libemulator_common.a -L$(top_srcdir)/common -lcommon  -liconv

INCLUDES = -I$(top_srcdir)/common/include/
INCLUDES += -I$(top_srcdir)/android/skyeye/
INCLUDES += -I$(top_srcdir)/android/
INCLUDES += -I$(top_srcdir)/android/utils/
INCLUDES += -I$(top_srcdir)/third-party/distrib/libpng-1.2.19/
INCLUDES += -I$(top_srcdir)/third-party/distrib/zlib-1.2.3/
INCLUDES += -I$(top_srcdir)/android/config/linux-x86/
INCLUDES += -I$(top_srcdir)/third-party/distrib/sdl-1.2.12/include/

pkglib_LTLIBRARIES = libandroid.la
