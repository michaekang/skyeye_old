##############################################################################
##############################################################################
###
###  emulator-libui: LIBRARY OF UI-RELATED FUNCTIONS
###
###  THESE ARE USED BY 'emulator-ui' AND THE STANDALONE PROGRAMS
###

###########################################################
# Libpng configuration
#
LIBPNG_SOURCES = $(top_srcdir)/third-party/distrib/libpng-1.2.19/png.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pngerror.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pngget.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pngmem.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pngpread.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pngread.c \
	$(top_srcdir)/third-party/distrib/libpng-1.2.19/pngrio.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pngrtran.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pngrutil.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pngset.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pngtrans.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pngvcrd.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pngwio.c \
	$(top_srcdir)/third-party/distrib/libpng-1.2.19/pngwrite.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pngwtran.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pngwutil.c $(top_srcdir)/third-party/distrib/libpng-1.2.19/pnggccrd.c

LIBPNG_SOURCES += $(top_srcdir)/android/skyeye/loadpng.c
INCLUDES = -I$(top_srcdir)/third-party/distrib/libpng-1.2.19/

##############################################################################
# SDL-related definitions
#

# Build SDL from sources except in certain cases where we use
# prebuilt libraries instead.
#
SDL_SOURCES =  \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/SDL.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/SDL_error.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/SDL_fatal.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/events/SDL_active.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/events/SDL_events.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/events/SDL_expose.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/events/SDL_keyboard.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/events/SDL_mouse.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/events/SDL_quit.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/events/SDL_resize.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/file/SDL_rwops.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/stdlib/SDL_getenv.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/stdlib/SDL_iconv.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/stdlib/SDL_malloc.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/stdlib/SDL_qsort.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/stdlib/SDL_stdlib.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/stdlib/SDL_string.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/cpuinfo/SDL_cpuinfo.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/loadso/dlopen/SDL_sysloadso.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/thread/SDL_thread.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/thread/pthread/SDL_syscond.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/thread/pthread/SDL_sysmutex.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/thread/pthread/SDL_syssem.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/thread/pthread/SDL_systhread.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/timer/SDL_timer.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/timer/unix/SDL_systimer.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_RLEaccel.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_blit.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_blit_0.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_blit_1.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_blit_A.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_blit_N.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_bmp.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_cursor.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_gamma.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_pixels.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_stretch.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_surface.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_yuv.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_yuv_mmx.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_yuv_sw.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/dummy/SDL_nullevents.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/dummy/SDL_nullmouse.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/dummy/SDL_nullvideo.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/x11/SDL_x11dyn.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/x11/SDL_x11events.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/x11/SDL_x11gamma.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/x11/SDL_x11gl.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/x11/SDL_x11image.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/x11/SDL_x11modes.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/x11/SDL_x11mouse.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/x11/SDL_x11video.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/x11/SDL_x11wm.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/x11/SDL_x11yuv.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/x11/SDL_x11dga.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_video.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/Xext/XME/xme.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/Xext/Xinerama/Xinerama.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/Xext/Xv/Xv.c \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/main/dummy/SDL_dummy_main.c

SDL_LDLIBS = -ldl
INCLUDES += -I$(top_srcdir)/third-party/distrib/sdl-1.2.12/include/


# the skin support sources
#
SKIN_SOURCES  = $(top_srcdir)/android/skin/rect.c \
	$(top_srcdir)/android/skin/region.c \
	$(top_srcdir)/android/skin/image.c \
	$(top_srcdir)/android/skin/trackball.c \
	$(top_srcdir)/android/skin/keyboard.c \
	$(top_srcdir)/android/skin/keyset.c \
	$(top_srcdir)/android/skin/file.c \
	$(top_srcdir)/android/skin/window.c \
	$(top_srcdir)/android/skin/scaler.c \
	$(top_srcdir)/android/skin/composer.c \
	$(top_srcdir)/android/skin/surface.c

#$(top_srcdir)/android/libemulator_libui.so:
emulator_libui = $(LIBPNG_SOURCES) $(SDL_SOURCES) $(SKIN_SOURCES)
emulator_libui += $(top_srcdir)/android/user-config.c \
	$(top_srcdir)/android/resource.c \
	$(top_srcdir)/android/qemulator.c \
	$(top_srcdir)/android/keycode.c

main_dir = \
	$(top_srcdir)/android/cmdline-option.c \
	$(top_srcdir)/android/console.c \
	$(top_srcdir)/android/display.c \
	$(top_srcdir)/android/display-core.c \
	$(top_srcdir)/android/help.c \
	$(top_srcdir)/android/main-common.c \
	$(top_srcdir)/android/android_main.c \
	$(top_srcdir)/android/protocol/core-commands-qemu.c \
	$(top_srcdir)/android/protocol/ui-commands-qemu.c 
	

##dyn
#
#$(top_srcdir)/android/libemulator_libui.so: 
libemulator_libui_la_SOURCES = $(emulator_libui) $(main_dir)
libemulator_libui_la_LDFLAGS = -module -L$(top_srcdir)/android/objs/emulator_common/ -lemulator_common -L$(top_srcdir)/android/objs/emulator_libqemu/ -lemulator_libqemu -L/usr/local/lib -liconv 

#libemulator_libui_la_LDFLAGS = -module $(top_srcdir)/third-party/distrib/sdl-1.2.12/objs/bin/sdl-config --libs -L$(top_srcdir)/android/objs/emulator_common/ -lemulator_common -L$(top_srcdir)/android/objs/emulator_libqemu/ -lemulator_libqemu  

pkglib_LTLIBRARIES = libemulator_libui.la

###static
#
#noinst_LIBRARIES = libemulator_libui.a 
#libemulator_libui_a_SOURCES = $(emulator_libui) $(main_dir)
#libemulator_libui_a_LIBADD = $(top_srcdir)/third-party/distrib/sdl-1.2.12/objs/bin/sdl-config --libs $(top_srcdir)/android/objs/emulator_common/libemulator_common.a $(top_srcdir)/android/objs/emulator_libqemu/emulator_libqemu.a -L$(top_srcdir)/common -lcommon 
#libemulator_libui_a_LIBADD = $(top_srcdir)/third-party/distrib/sdl-1.2.12/objs/lib/libSDLmain.a $(top_srcdir)/third-party/distrib/sdl-1.2.12/objs/lib/libSDL.a  $(top_srcdir)/android/objs/emulator_common/libemulator_common.a $(top_srcdir)/android/objs/emulator_libqemu/libemulator_libqemu.a  

INCLUDES += -I$(top_srcdir)/common/include/
INCLUDES += -I$(top_srcdir)/android/skyeye/
INCLUDES += -I$(top_srcdir)/android/
INCLUDES += -I$(top_srcdir)/android/utils/
INCLUDES += -I$(top_srcdir)/third-party/distrib/libpng-1.2.19/
INCLUDES += -I$(top_srcdir)/third-party/distrib/zlib-1.2.3/
INCLUDES += -I$(top_srcdir)/android/config/linux-x86/
INCLUDES += -I/usr/local/include/
#INCLUDES += -I$(top_srcdir)/third-party/distrib/sdl-1.2.12/objs/include/SDL -D_GNU_SOURCE=1 -D_REENTRANT
