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

###########################################################
# Android utility functions
#
emulator_common += \
        $(top_srcdir)/android/skyeye/sockets.c \
	$(top_srcdir)/android/skyeye/user-events-qemu.c \
        $(top_srcdir)/android/skyeye/iolooper-select.c \
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

#$(top_srcdir)/android/libemulator_common.a: 
#noinst_LIBRARIES = libemulator_common.a 
#libemulator_common_a_SOURCES = $(emulator_common)

######dyn
#
#$(top_srcdir)/android/libemulator_common.so:
libemulator_common_la_SOURCES = $(emulator_common)
libemulator_common_la_LDFLAGS = -module -L$(top_srcdir)/common -lcommon -lrt 
pkglib_LTLIBRARIES = libemulator_common.la

INCLUDES = -I$(top_srcdir)/android/config/linux-x86/
INCLUDES += -I$(top_srcdir)/common/include/
INCLUDES += -I$(top_srcdir)/android/skyeye/
INCLUDES += -I$(top_srcdir)/android/
INCLUDES += -I$(top_srcdir)/android/utils/
INCLUDES += -I$(top_srcdir)/third-party/distrib/zlib-1.2.3/

