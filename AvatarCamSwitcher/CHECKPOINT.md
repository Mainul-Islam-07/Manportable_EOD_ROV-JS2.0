# Project Checkpoint — Avatar Cam Switcher

_Snapshot of the project state at this point in the build, so it can be resumed
later or handed to a fresh chat. Memory is off, so paste this file in if you
start a new conversation and want full context._

Status: **building** — last fix applied was the Kotlin version bump (see log #5).

---

## What this app is

Single-Activity Android app that switches between **5 SIYI RTSP camera feeds** on
one full-screen, zero-buffer viewport for the Avatar rover ground station.

| Feed  | Camera          | Endpoint                               |
|-------|-----------------|----------------------------------------|
| MAIN  | SIYI A8 mini    | `rtsp://192.168.144.64:8554/main.264`  | ← default on launch
| FRONT | SIYI R1M        | `rtsp://192.168.144.65:8554/main.264`  |
| BACK  | SIYI R1M        | `rtsp://192.168.144.66:8554/main.264`  |
| WRIST | SIYI R1M        | `rtsp://192.168.144.67:8554/main.264`  |
| GRIP  | SIYI R1M        | `rtsp://192.168.144.68:8554/main.264`  |

Engine: `com.github.alexeyvasilyev:rtsp-client-android:5.6.4` via JitPack.
`RtspSurfaceView` decodes RTP straight through `MediaCodec` (no buffer in the
render path).

---

## Verified working version matrix

| Component        | Version            | Notes                                              |
|------------------|--------------------|----------------------------------------------------|
| Gradle (wrapper) | 8.7                | jar fetched from gradle/gradle v8.7.0               |
| Android Gradle Plugin | 8.6.0         | minimum that compiles against API 35               |
| Kotlin           | 2.2.0              | must match the library's compiled metadata         |
| Java / jvmTarget | 17                 | library is built against 17 — do not lower         |
| compileSdk       | 35                 | required by media3 1.9.3 / camera-core 1.5.3       |
| targetSdk        | 34                 | not opting into Android 15 runtime behavior        |
| minSdk           | 24                 | Android 7.0+; **Android 9 (API 28) supported**     |

App deps: `core-ktx:1.13.1`, `appcompat:1.7.0`, `constraintlayout:2.1.4`.
Transitive from the RTSP lib: `media3-exoplayer:1.9.3` (+ siblings),
`camera-core:1.5.3`, `jcodec:0.2.5`.

---

## Fix log (chronological)

1. **Initial build** — Kotlin/XML/ConstraintLayout app; API verified against the
   real v5.6.4 source (`init`/`start`/`stop`/`RtspStatusListener`).
2. **Packaged as full Android Studio project** — added Gradle wrapper (8.7),
   `.gitignore`, `local.properties` template, README; zipped.
3. **Android 9 question** — confirmed compatible (`minSdk 24` ≤ API 28).
4. **AAR metadata error** (media3/camerax require API 35) → `compileSdk 34 → 35`
   and `AGP 8.5.2 → 8.6.0`. `minSdk` untouched, so Android 9 install unaffected.
5. **Kotlin metadata error** (lib metadata 2.2.0 vs compiler 1.9.0) →
   `Kotlin 1.9.24 → 2.2.0`.
6. **Switch artifacts** (both cameras flickering + mosaic/pixelation) → root cause
   was async `stop()` (no join) letting two decoders share one surface, plus
   mid-GOP P-frame decode, amplified by the SPS rewrite. Fix: SPS rewrite OFF
   (`LOW_LATENCY_SPS_REWRITE = false`), switch SERIALIZED via the
   `onRtspStatusDisconnected` callback (with an 800 ms timeout fallback), and a
   black `transition_cover` that lifts on `onRtspFirstFrameRendered`.
7. **FRONT/BACK upside down** → per-feed `videoRotation` (180 for FRONT/BACK, 0
   for the rest), applied via `MediaFormat.KEY_ROTATION` (decoder hint, no CPU).
8. **Dropped the status overlay** ("MAIN — LIVE" etc.) — active camera is shown by
   the highlighted nav button.
9. **Recording = whole-screen capture** (replaced the per-camera muxer). Uses
   `MediaProjection` → `VirtualDisplay` → `MediaRecorder` in a foreground service
   (`ScreenRecordService`, type `mediaProjection`). Saves to
   `Android/data/<pkg>/files/Movies/AvatarCam/REC_SCREEN_*.mp4`. Toggle via the
   top-right REC button or physical key `0`. Because it records the composited
   display, **switching cameras does NOT stop the recording** — it's one
   continuous file. A system consent dialog appears on each start (required).
   Captures the on-screen buttons too (acceptable per requirements).

---

## Key code locations (in `MainActivity.kt`)

- `enum class CameraFeed` — the 5 endpoints; `MAIN` is the launch default.
- `playCurrentFeed()` — the teardown→rebuild sequence: `stop()` → `init(Uri)` →
  `start(requestVideo = true, requestAudio = false)`.
- `switchTo(feed)` — single entry point for buttons + physical keys; idempotent.
- `onKeyDown(...)` — physical number keys `1`–`5` → FRONT/BACK/WRIST/GRIP/MAIN.
- `onCreate(...)` low-latency block:
  - `videoFrameRateStabilization = false`
  - `experimentalUpdateSpsFrameWithLowLatencyParams = LOW_LATENCY_SPS_REWRITE`
    (companion constant, currently `true`; flip to `false` if a feed renders
    green/corrupted).
- Surface lifecycle gating (`surfaceReady`) so the first stream starts only once
  a valid `Surface` exists.

Layout: `activity_main.xml` — full-screen `RtspSurfaceView`, bottom 5-button nav
bar, floating indicator `TextView`.

---

## Open / optional items (none blocking)

- **API 24–25 padding**: layout uses `paddingHorizontal` / `paddingVertical` /
  `layout_marginHorizontal` (API 26+ attrs). Fine on Android 9; silently ignored
  on Android 7.0–7.1. Swap to `start/end/top/bottom` for true 24+ coverage.
- **Kotlin deprecation warning**: `kotlinOptions { jvmTarget = '17' }` warns under
  Kotlin 2.x. Cosmetic; can move to the `compilerOptions` DSL to silence it.
- **Dependency trimming**: ExoPlayer/media3 + camera-core come in transitively and
  inflate the APK. Removable only after confirming the render path doesn't touch
  them at runtime (they're `implementation`-scoped in the AAR).
- **compileSdk 36 option**: would need AGP 8.9+ and Gradle 8.11+. No benefit over
  35 here.
- **Transport**: RTP runs over interleaved TCP inside the library — not a
  user-selectable UDP toggle.

---

## How to resume / build

1. `File → Open` the `AvatarCamSwitcher` folder in Android Studio.
2. Install the **Android 15 (API 35) SDK Platform** if prompted (SDK Manager).
3. First sync needs internet (downloads Gradle 8.7 + dependencies); the app
   itself only talks to the cameras on the LAN afterward.
4. Run on a device on the same network as the cameras.
   CLI: `./gradlew assembleDebug` → `app/build/outputs/apk/debug/`.
