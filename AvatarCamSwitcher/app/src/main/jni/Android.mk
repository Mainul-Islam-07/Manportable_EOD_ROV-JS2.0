LOCAL_PATH := $(call my-dir)

# ---------------------------------------------------------------------------
#  Our native pipeline shim (gst_backend.c) — loaded from Kotlin as
#  System.loadLibrary("gst_backend"). Depends on the prebuilt
#  libgstreamer_android.so produced by gstreamer-1.0.mk below.
# ---------------------------------------------------------------------------
include $(CLEAR_VARS)

LOCAL_MODULE            := gst_backend
LOCAL_SRC_FILES         := gst_backend.c
LOCAL_SHARED_LIBRARIES  := gstreamer_android
LOCAL_LDLIBS            := -llog -landroid
# Align LOAD segments to 16 KB for Android 15+ 16 KB-page devices.
LOCAL_LDFLAGS           := -Wl,-z,max-page-size=16384

include $(BUILD_SHARED_LIBRARY)

# ---------------------------------------------------------------------------
#  GStreamer Android build. GSTREAMER_ROOT_ANDROID must point at the extracted
#  GStreamer Android universal SDK (passed from build.gradle / gradle.properties).
# ---------------------------------------------------------------------------
# Gradle passes the literal string "null" when the property/env var is unset, so
# a plain ifndef is not enough — check for empty and "null" explicitly.
ifeq ($(strip $(GSTREAMER_ROOT_ANDROID)),)
GSTREAMER_ROOT_ANDROID := null
endif
ifeq ($(GSTREAMER_ROOT_ANDROID),null)
$(error >>> GStreamer SDK not configured. Download the GStreamer Android *universal* \
SDK from https://gstreamer.freedesktop.org/download/#android, extract it, and set \
gstreamer.root.android=<path> in gradle.properties (forward slashes), e.g. \
gstreamer.root.android=C:/gstreamer/1.24.10/android . See AvatarCamSwitcher/GSTREAMER_SETUP.md)
endif

ifeq ($(TARGET_ARCH_ABI),arm64-v8a)
GSTREAMER_ROOT := $(GSTREAMER_ROOT_ANDROID)/arm64
else
$(error Unsupported ABI "$(TARGET_ARCH_ABI)". This build targets arm64-v8a (MK32) only.)
endif

GSTREAMER_NDK_BUILD_PATH := $(GSTREAMER_ROOT)/share/gst-android/ndk-build/

include $(GSTREAMER_NDK_BUILD_PATH)/plugins.mk

# Explicit, minimal plugin set for our H.264-over-RTSP/UDP pipeline (keeps the
# bundled libgstreamer_android.so small instead of ~200 plugins). If a runtime
# "no element X" appears on the MK32, add the plugin that provides X here.
#   coreelements ......... queue/tee/capsfilter/... (pipeline plumbing)
#   playback/typefind .... decodebin autoplugging
#   videoconvertscale .... format glue decodebin/glimagesink may insert
#   rtsp/rtp/rtpmanager/udp  rtspsrc + rtph264depay + jitterbuffer + udpsrc
#   videoparsersbad ...... h264parse
#   androidmedia ......... hardware amcviddec (H.264)
#   libav ................ avdec_h264 software fallback
#   opengl ............... glimagesink + glupload + glvideoflip (rotate-180)
GSTREAMER_PLUGINS := \
    coreelements app playback typefindfunctions videoconvertscale \
    rtsp rtp rtpmanager udp \
    videoparsersbad androidmedia libav \
    opengl

# gstreamer-video-1.0 gives us GstVideoOverlay (bind the pipeline to the Surface).
GSTREAMER_EXTRA_DEPS := gstreamer-video-1.0

include $(GSTREAMER_NDK_BUILD_PATH)/gstreamer-1.0.mk
