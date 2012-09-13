##############################################################################
##############################################################################
###
###  emulator_common: LIBRARY OF COMMON FUNCTIONS
###
###  THESE ARE POTENTIALLY USED BY ALL COMPONENTS
###

###########################################################
# Zlib sources
#
ZLIB_SOURCES = $(top_srcdir)/third-party/distrib/zlib-1.2.3/adler32.c $(top_srcdir)/third-party/distrib/zlib-1.2.3/compress.c $(top_srcdir)/third-party/distrib/zlib-1.2.3/crc32.c $(top_srcdir)/third-party/distrib/zlib-1.2.3/deflate.c $(top_srcdir)/third-party/distrib/zlib-1.2.3/gzio.c $(top_srcdir)/third-party/distrib/zlib-1.2.3/infback.c $(top_srcdir)/third-party/distrib/zlib-1.2.3/inffast.c $(top_srcdir)/third-party/distrib/zlib-1.2.3/inflate.c $(top_srcdir)/third-party/distrib/zlib-1.2.3/inftrees.c $(top_srcdir)/third-party/distrib/zlib-1.2.3/trees.c $(top_srcdir)/third-party/distrib/zlib-1.2.3/uncompr.c $(top_srcdir)/third-party/distrib/zlib-1.2.3/zutil.c
emulator_common = $(ZLIB_SOURCES) 
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

###########################################################
# Android utility functions
#
emulator_common += \
	$(top_srcdir)/android/skyeye/json-lexer.c \
	$(top_srcdir)/android/skyeye/json-parser.c \
	$(top_srcdir)/android/skyeye/json-streamer.c \
	$(top_srcdir)/android/skyeye/qjson.c \
	$(top_srcdir)/android/skyeye/qbool.c \
	$(top_srcdir)/android/skyeye/qdict.c \
	$(top_srcdir)/android/skyeye/qfloat.c \
	$(top_srcdir)/android/skyeye/qint.c \
	$(top_srcdir)/android/skyeye/qlist.c \
	$(top_srcdir)/android/skyeye/qstring.c \
        $(top_srcdir)/android/skyeye/sockets.c \
	$(top_srcdir)/android/skyeye/user-events-qemu.c \
        $(top_srcdir)/android/skyeye/iolooper-select.c \
	$(top_srcdir)/android/skyeye/input.c \
	$(top_srcdir)/android/skyeye/notify.c \
	$(top_srcdir)/android/skyeye/aio-android.c \
	$(top_srcdir)/android/skyeye/async.c \
	$(top_srcdir)/android/skyeye/cbuffer.c \
	$(top_srcdir)/android/skyeye/charpipe.c \
	$(top_srcdir)/android/skyeye/console.c \
	$(top_srcdir)/android/skyeye/cutils.c \
	$(top_srcdir)/android/skyeye/iohandler.c \
	$(top_srcdir)/android/skyeye/osdep.c \
	$(top_srcdir)/android/skyeye/qemu-config.c \
	$(top_srcdir)/android/skyeye/qemu-error.c \
	$(top_srcdir)/android/skyeye/qemu-malloc.c \
	$(top_srcdir)/android/skyeye/qemu-option.c \
	$(top_srcdir)/android/skyeye/qemu-sockets-android.c \
	$(top_srcdir)/android/skyeye/qerror.c \
	$(top_srcdir)/android/skyeye/qemu-char.c \
	$(top_srcdir)/android/hw-kmsg.c \
	$(top_srcdir)/android/config.c \
	$(top_srcdir)/android/core-init-utils.c   \
	$(top_srcdir)/android/hw-events.c \
	$(top_srcdir)/android/qemu-setup.c \
	$(top_srcdir)/android/utils/timezone.c \
        $(top_srcdir)/android/async-console.c \
        $(top_srcdir)/android/async-utils.c \
        $(top_srcdir)/android/charmap.c \
        $(top_srcdir)/android/framebuffer.c \
        $(top_srcdir)/android/keycode-array.c \
        $(top_srcdir)/android/avd/hw-config.c \
        $(top_srcdir)/android/avd/info.c \
        $(top_srcdir)/android/avd/util.c \
        $(top_srcdir)/android/sync-utils.c \
        $(top_srcdir)/android/utils/assert.c \
        $(top_srcdir)/android/utils/bufprint.c \
        $(top_srcdir)/android/utils/debug.c \
        $(top_srcdir)/android/utils/dirscanner.c \
        $(top_srcdir)/android/utils/filelock.c \
        $(top_srcdir)/android/utils/ini.c \
        $(top_srcdir)/android/utils/intmap.c \
        $(top_srcdir)/android/utils/lineinput.c \
        $(top_srcdir)/android/utils/mapfile.c \
        $(top_srcdir)/android/utils/misc.c \
        $(top_srcdir)/android/utils/panic.c \
        $(top_srcdir)/android/utils/path.c \
        $(top_srcdir)/android/utils/reflist.c \
        $(top_srcdir)/android/utils/refset.c \
        $(top_srcdir)/android/utils/stralloc.c \
        $(top_srcdir)/android/utils/system.c \
        $(top_srcdir)/android/utils/tempfile.c \
        $(top_srcdir)/android/utils/vector.c
######static
#
if WIN32
emulator_common += \
       $(top_srcdir)/android/skyeye/os-win32.c \
	$(top_srcdir)/android/skyeye/qemu_iothread.c
endif

#$(top_srcdir)/android/libemulator_common.a: 
#noinst_LIBRARIES = libemulator_common.a 
#libemulator_common_a_SOURCES = $(emulator_common)

######dyn
#
#$(top_srcdir)/android/libemulator_common.so:
libemulator_common_la_SOURCES = $(emulator_common)
libemulator_common_la_LDFLAGS = -module
if WIN32
libemulator_common_la_LDFLAGS += -no-undefined -shared -module -Xlinker -L$(top_srcdir)/common/.libs/ -Xlinker -lcommon -Xlinker -lwinmm -Xlinker -lmsvcrt -Xlinker -lkernel32 -Xlinker -lole32 -Xlinker -lkernel32 -Xlinker -lgdi32 -Xlinker-luuid -Xlinker -luser32  -Xlinker -lshell32 -Xlinker -lws2_32 -Xlinker --export-all-symbol
else
libemulator_common_la_LDFLAGS += -module -L$(top_srcdir)/common -lcommon -lrt -L$(top_builddir)/third-party/libiberty/pic
endif
pkglib_LTLIBRARIES = libemulator_common.la

if WIN32
INCLUDES = -I$(top_srcdir)/android/config/windows/
else
INCLUDES = -I$(top_srcdir)/android/config/linux-x86/
endif
INCLUDES += -I$(top_srcdir)/common/include/
INCLUDES += -I$(top_srcdir)/android/skyeye/
INCLUDES += -I$(top_srcdir)/android/
INCLUDES += -I$(top_srcdir)/android/utils/
INCLUDES += -I$(top_srcdir)/third-party/distrib/zlib-1.2.3/

