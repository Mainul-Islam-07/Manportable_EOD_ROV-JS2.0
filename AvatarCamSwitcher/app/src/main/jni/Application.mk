# The SIYI MK32 ground station is arm64 (Qualcomm 8-core). We ship a single ABI
# to keep the APK small; add armeabi-v7a here only if the app must also run on
# an older 32-bit device. APP_PLATFORM must match app/build.gradle minSdk (24).
APP_ABI := arm64-v8a
APP_PLATFORM := android-24
# GStreamer's link hard-references libc++_shared.so, so we must ship it (c++_static
# leaves a dangling NEEDED). The only downside is that the NDK r26 prebuilt
# libc++_shared.so is 4 KB-aligned; a full 16 KB-page fix needs NDK r27+/r28.
APP_STL := c++_shared
