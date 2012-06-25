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
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/cpuinfo/SDL_cpuinfo.c
if WIN32
SDL_SOURCES +=  \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/loadso/win32/SDL_sysloadso_win32.c
else
SDL_SOURCES +=  \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/loadso/dlopen/SDL_sysloadso.c
endif
SDL_SOURCES +=  \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/thread/SDL_thread.c

if WIN32
SDL_SOURCES += \
       $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/thread/win32/SDL_sysmutex_win32.c \
       $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/thread/win32/SDL_syssem_win32.c \
       $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/thread/win32/SDL_systhread_win32.c
else
SDL_SOURCES +=  \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/thread/pthread/SDL_syscond.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/thread/pthread/SDL_sysmutex.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/thread/pthread/SDL_syssem.c \
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/thread/pthread/SDL_systhread.c
endif
SDL_SOURCES +=  \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/timer/SDL_timer.c
if WIN32
SDL_SOURCES +=  \
       $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/timer/win32/SDL_systimer_win32.c
else
SDL_SOURCES +=  \
	$(top_srcdir)/third-party/distrib/sdl-1.2.12/src/timer/unix/SDL_systimer.c
endif

SDL_SOURCES +=  \
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
        $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/dummy/SDL_nullvideo.c

if WIN32
SDL_SOURCES += \
       $(top_srcdir)/android/utils/setenv.c \
       $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/windib/SDL_dibevents.c \
       $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/windib/SDL_dibvideo.c \
       $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/wincommon/SDL_sysevents.c \
       $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/wincommon/SDL_sysmouse.c \
       $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/wincommon/SDL_syswm.c \
       $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/wincommon/SDL_wingl.c \
       $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/video/SDL_video.c \
       $(top_srcdir)/third-party/distrib/sdl-1.2.12/src/main/win32/SDL_win32_main.c
else
SDL_SOURCES +=  \
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
endif

if WIN32
SDL_LDLIBS = -Xlinker -luser32 -Xlinker -lgdi32
else
SDL_LDLIBS = -ldl
endif
INCLUDES += -I$(top_srcdir)/third-party/distrib/sdl-1.2.12/include/

if WIN32
INCLUDES += -I /MinGW/GnuWin32/include/
endif

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
	$(top_srcdir)/android/skyeye/vl-android.c \
	$(top_srcdir)/android/protocol/core-commands-qemu.c \
	$(top_srcdir)/android/protocol/ui-commands-qemu.c 
	

##dyn
#
libemulator_libui_la_SOURCES = $(emulator_libui) $(main_dir)
if WIN32
libemulator_libui_la_LDFLAGS = -no-undefined -shared -module  -Xlinker --enable-auto-import -Xlinker -L$(top_srcdir)/common/.libs/ -Xlinker -lcommon -Xlinker -L$(top_srcdir)/android/objs/emulator_common/.libs/ -Xlinker -lemulator_common -Xlinker -L/MinGW/GnuWin32/bin -Xlinker -llibiconv2 -Xlinker -lgdi32 -Xlinker -lws2_32 -Xlinker -lkernel32 -Xlinker -lkernel32 -Xlinker -lole32 -Xlinker -lkernel32 -Xlinker -lgdi32 -Xlinker -luuid -Xlinker -luser32 -Xlinker -lshell32 -Xlinker --export-all-symbol
else
libemulator_libui_la_LDFLAGS = -module -L$(top_srcdir)/common -lcommon -L$(top_srcdir)/android/objs/emulator_common/ -lemulator_common  -L/usr/local/lib -liconv
endif


pkglib_LTLIBRARIES = libemulator_libui.la

INCLUDES += -I$(top_srcdir)/common/include/
INCLUDES += -I$(top_srcdir)/android/skyeye/
INCLUDES += -I$(top_srcdir)/android/
INCLUDES += -I$(top_srcdir)/android/utils/
INCLUDES += -I$(top_srcdir)/third-party/distrib/libpng-1.2.19/
INCLUDES += -I$(top_srcdir)/third-party/distrib/zlib-1.2.3/
INCLUDES += -I$(top_srcdir)/soc/arm/goldfish/
if WIN32
INCLUDES += -I$(top_srcdir)/android/config/windows/ \
	 -I$(top_srcdir)/third-party/distrib/sdl-1.2.12/
else
INCLUDES += -I$(top_srcdir)/android/config/linux-x86/ \
	 -I/usr/local/include/
endif
