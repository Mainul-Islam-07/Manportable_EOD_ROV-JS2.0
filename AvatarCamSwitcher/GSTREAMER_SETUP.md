# GStreamer video pipeline — build setup

The camera video path uses a native **GStreamer** pipeline (RTP over UDP, small
jitter buffer, hardware decode, ride-through on packet loss) instead of an RTSP
library. Building the app therefore needs a one-time toolchain setup on the build
machine. The GStreamer SDK is **not** committed to the repo.

## Prerequisites

1. **Android NDK** — Android Studio → SDK Manager → SDK Tools → NDK (Side by side).
   `app/build.gradle` pins `ndkVersion "26.3.11579264"`; install that version or
   change the pin to the one you install. (Gradle can auto-install it on first build.)

2. **GStreamer Android universal SDK** — download from
   <https://gstreamer.freedesktop.org/download/#android> (e.g.
   `gstreamer-1.0-android-universal-1.24.x.tar.xz`) and extract it. The extracted
   folder contains per-ABI subfolders (`arm64/`, `armv7/`, …). We build **arm64
   only** (the MK32 is arm64).

3. **Point the build at it** — set `GSTREAMER_ROOT_ANDROID` to the extracted folder,
   either in `gradle.properties` (uncommitted) or as an environment variable:

   ```properties
   # AvatarCamSwitcher/gradle.properties  (or the project-level one)
   gstreamer.root.android=C:/gstreamer/1.24.x/android
   ```
   ```bash
   # or environment
   export GSTREAMER_ROOT_ANDROID=/c/gstreamer/1.24.x/android
   ```

## Required one-time SDK patch (Windows)

GStreamer's `gstreamer-1.0.mk` generates two files with multi-stage shell
**pipes** (`sed | sed | ... > out`). The Android NDK (r23+) no longer bundles a
POSIX shell, so the NDK's `make` runs those recipe lines *without* a shell and the
pipe isn't handled — the build dies with
`tools/windows/sed: can't read |: Invalid argument`.

Fix (once per extracted SDK): in
`<GSTREAMER_ROOT_ANDROID>/arm64/share/gst-android/ndk-build/gstreamer-1.0.mk`,
replace the two piped recipes (the `genstatic_*` one, ~line 215, and the
`copyjavasource_*` one, ~line 252) with pipe-free sequential `sed -i` commands:
copy the input with `$(call host-cp,SRC,DST)` then run each `$(SED_LOCAL) -i
"EXPR" DST` on its own line. (The bundled sed is GNU sed 4.2.1 and supports `-i`.)
This repo's copy of the change is already applied on this machine; re-apply it if
you re-extract the SDK.

## Build

```bash
JAVA_HOME="/c/Program Files/Android/Android Studio/jbr" \
  ./gradlew :app:assembleDebug
```

The `ndkBuildGStreamer` Gradle task (in `app/build.gradle`) drives `ndk-build`
directly — building `gst_backend.c` + `libgstreamer_android.so`, staging the `.so`
into `build/gstreamerLibs` (picked up via `jniLibs.srcDirs`), and generating
`org/freedesktop/gstreamer/GStreamer.java` into `app/src/main/java`. We do NOT use
AGP's `externalNativeBuild` for this, because it can't model GStreamer's
hand-built prebuilt `.so` ("Expected output file … but there was none"). If
`gstreamer.root.android` is unset the task fails early with a clear message.

Plugin set is trimmed to our H.264 pipeline in `Android.mk` (~22 MB `.so` vs
~112 MB for all plugins). A runtime `no element "X"` on the MK32 means add the
plugin providing X to `GSTREAMER_PLUGINS`.

## Android 15+ 16 KB page alignment

`libgst_backend.so` and `libgstreamer_android.so` are linked with
`-Wl,-z,max-page-size=16384` (in `Android.mk` and the SDK's `GSTREAMER_LD`), so
their LOAD segments are 16 KB-aligned. The remaining lib, `libc++_shared.so`, is
an **NDK r26 prebuilt stuck at 4 KB** — GStreamer hard-links it, so it must ship
(`c++_static` leaves a dangling `NEEDED`). This only matters on true 16 KB-page
devices; **the MK32 is 4 KB, so the app runs fine** and this is just an advisory
warning. For full compliance (16 KB devices / Play Store), bump `ndkVersion` to
**r27+/r28**, whose prebuilt `libc++_shared.so` is 16 KB-aligned (r28 also makes
alignment the default) — then rebuild and re-verify with `llvm-readelf -l`.

## Where things live

- `app/src/main/jni/Android.mk`, `Application.mk` — ndk-build config + plugin set.
- `app/src/main/jni/gst_backend.c` — the pipeline shim
  (`rtspsrc protocols=udp latency=100 do-retransmission=false ! rtph264depay !
  h264parse ! decodebin ! [glvideoflip] ! glimagesink`), first-frame/size probes,
  and the UDP frame-arrival watchdog (stall/outage).
- `app/src/main/java/.../video/GStreamerVideoView.kt` — SurfaceView + JNI bridge.
- `MainActivity.kt` — drives `setFeed/play/stop` and maps
  `onFirstFrame/onSizeChanged/onError/onStall/onOutage/onRecovered` onto the
  cover / reconnect / ride-through UX.

## Tuning (on the MK32)

- `latency` in the pipeline string (`gst_backend.c`) — 50–200 ms trades latency
  for smoothness.
- `STALL_US` / `OUTAGE_US` in `gst_backend.c` — how long a silent UDP gap shows
  "WEAK SIGNAL" before a reconnect.
- If the GL rotate path (`glupload ! glvideoflip`) misbehaves for FRONT/BACK,
  swap that branch for `videoconvert ! videoflip method=rotate-180`.
