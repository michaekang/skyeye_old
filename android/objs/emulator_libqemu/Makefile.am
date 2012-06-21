##############################################################################
##############################################################################
###
###  emulator_libqemu: TARGET-INDEPENDENT QEMU FUNCTIONS
###
###  THESE ARE USED BY EVERYTHING EXCEPT 'emulator-ui'
###


# misc. sources
#
CORE_MISC_SOURCES = \
    $(top_srcdir)/android/skyeye/aio-android.c \
    $(top_srcdir)/android/skyeye/async.c \
    $(top_srcdir)/android/skyeye/cbuffer.c \
    $(top_srcdir)/android/skyeye/charpipe.c \
    $(top_srcdir)/android/skyeye/console.c \
    $(top_srcdir)/android/skyeye/cutils.c \
    $(top_srcdir)/android/skyeye/input.c \
    $(top_srcdir)/android/skyeye/iohandler.c \
    $(top_srcdir)/android/skyeye/notify.c \
    $(top_srcdir)/android/skyeye/osdep.c \
    $(top_srcdir)/android/skyeye/qemu-char.c \
    $(top_srcdir)/android/skyeye/qemu-config.c \
    $(top_srcdir)/android/skyeye/qemu-error.c \
    $(top_srcdir)/android/skyeye/qemu-malloc.c \
    $(top_srcdir)/android/skyeye/qemu-option.c \
    $(top_srcdir)/android/skyeye/qemu-sockets-android.c \
    $(top_srcdir)/android/skyeye/qerror.c \
    $(top_srcdir)/android/config.c \
    $(top_srcdir)/android/core-init-utils.c   \
    $(top_srcdir)/android/gps.c \
    $(top_srcdir)/android/hw-kmsg.c \
    $(top_srcdir)/android/hw-events.c \
    $(top_srcdir)/android/qemu-setup.c \
    $(top_srcdir)/android/utils/timezone.c
if WIN32
CORE_MISC_SOURCES += \
    $(top_srcdir)/android/skyeye/os-win32.c \
    $(top_srcdir)/android/skyeye/qemu_iothread.c
endif


#$(top_srcdir)/android/hw-lcd.c \
#$(top_srcdir)/qemu-setup.c 
#snapshot.c \
#hw-control.c \
#hw-sensors.c \
#hw-pipe-net.c \
#looper-qemu.c \
#boot-properties.c \
#hw-qemud.c \
#qemu-setup.c \
#skyeye/path.c

# sources inherited from upstream, but not fully
# integrated into android emulator
#
emulator_libqemu = $(CORE_MISC_SOURCES) \
    $(top_srcdir)/android/skyeye/json-lexer.c \
    $(top_srcdir)/android/skyeye/json-parser.c \
    $(top_srcdir)/android/skyeye/json-streamer.c \
    $(top_srcdir)/android/skyeye/qjson.c \
    $(top_srcdir)/android/skyeye/qbool.c \
    $(top_srcdir)/android/skyeye/qdict.c \
    $(top_srcdir)/android/skyeye/qfloat.c \
    $(top_srcdir)/android/skyeye/qint.c \
    $(top_srcdir)/android/skyeye/qlist.c \
    $(top_srcdir)/android/skyeye/qstring.c
##static
#
#
#$(top_srcdir)/android/libemulator_libqemu.a:
#noinst_LIBRARIES = libemulator_libqemu.a 
#libemulator_libqemu_a_SOURCES = $(emulator_libqemu)
#libemulator_libqemu_a_LIBADD = $(top_srcdir)/android/objs/emulator_common/libemulator_common.a 

##dyn
#
#$(top_srcdir)/android/libemulator_libqemu.so:
libemulator_libqemu_la_SOURCES = $(emulator_libqemu)
libemulator_libqemu_la_LDFLAGS = -module
if WIN32
libemulator_libqemu_la_LDFLAGS +=  -no-undefined -shared -module -Xlinker -L$(top_srcdir)/common/.libs/ -Xlinker -lcommon -Xlinker -L$(top_srcdir)/android/objs/emulator_common/.libs/ -Xlinker -lemulator_common -Xlinker -lmsvcrt -Xlinker -lkernel32 -Xlinker -lole32 -Xlinker -lkernel32 -Xlinker -lgdi32 -Xlinker -luuid -Xlinker -luser32  -Xlinker -lshell32 -Xlinker --export-all-symbol
else
libemulator_libqemu_la_LDFLAGS += -module -L$(top_srcdir)/common -lcommon -L$(top_srcdir)/android/objs/emulator_common/ -lemulator_common -lrt
endif
pkglib_LTLIBRARIES = libemulator_libqemu.la

INCLUDES = -I$(top_srcdir)/common/include/
INCLUDES += -I$(top_srcdir)/android/skyeye/
INCLUDES += -I$(top_srcdir)/android/
INCLUDES += -I$(top_srcdir)/android/utils/
INCLUDES += -I$(top_srcdir)/third-party/distrib/libpng-1.2.19/
INCLUDES += -I$(top_srcdir)/third-party/distrib/zlib-1.2.3/
INCLUDES += -I$(top_srcdir)/third-party/distrib/sdl-1.2.12/include/
if WIN32
INCLUDES += -I$(top_srcdir)/android/config/windows/
else
INCLUDES += -I$(top_srcdir)/android/config/linux-x86/
endif

