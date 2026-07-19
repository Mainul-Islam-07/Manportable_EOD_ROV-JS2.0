/*
 * gst_backend.c — GStreamer video pipeline shim for AvatarCamSwitcher.
 *
 * Mirrors the official GStreamer Android "tutorial 3" surface/overlay pattern,
 * extended for an FPV RTSP feed:
 *   - RTP over UDP with a small jitter buffer (rtspsrc latency=..., drop-on-latency,
 *     do-retransmission=false) so the feed rides through packet loss with artifacts
 *     instead of stalling on TCP head-of-line blocking.
 *   - a sink-pad buffer probe for first-frame + frame-size signalling, and
 *   - a frame-arrival watchdog (UDP has no socket timeout) that reports STALL and
 *     OUTAGE up to Kotlin so the UI can ride through (keep last frame + "WEAK
 *     SIGNAL") rather than blanking to black.
 *
 * All JNI up-calls target com.avatarrobot.camswitcher.video.GStreamerVideoView.
 * arm64 only (MK32) — 64-bit aligned reads/writes of GstClockTime are atomic.
 */
#include <string.h>
#include <stdint.h>
#include <jni.h>
#include <pthread.h>
#include <android/log.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#define TAG "GstBackend"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,  TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)

/* Frame-arrival thresholds (microseconds, g_get_monotonic_time units). */
#define STALL_US   (1500 * 1000)   /* ~1.5 s of no frames -> WEAK SIGNAL   */
#define OUTAGE_US  (5000 * 1000)   /* ~5 s of no frames  -> reconnect      */
#define WATCHDOG_INTERVAL_MS 500

typedef struct _CustomData {
  jobject app;                 /* global ref to the GStreamerVideoView       */
  GstElement *pipeline;
  GMainContext *context;
  GMainLoop *main_loop;
  gboolean initialized;        /* main loop + surface both ready             */
  GstElement *video_sink;      /* glimagesink (GstVideoOverlay)              */
  ANativeWindow *native_window;

  gchar *uri;
  gint rotation;               /* 0 or 180                                   */

  gboolean first_frame_sent;
  gint size_w, size_h;         /* last reported frame size (0 = unknown)     */

  gint64 last_buffer_us;       /* g_get_monotonic_time() of last buffer      */
  gint stall_state;            /* 0 normal, 1 stall, 2 outage                */
  guint watchdog_id;
} CustomData;

/* JNI globals (cached in class_init / JNI_OnLoad). */
static pthread_t gst_app_thread;
static pthread_key_t current_jni_env;
static JavaVM *java_vm;
static jfieldID custom_data_field_id;
static jmethodID on_first_frame_id;
static jmethodID on_size_changed_id;
static jmethodID on_error_id;
static jmethodID on_stall_id;
static jmethodID on_outage_id;
static jmethodID on_recovered_id;

/* ------------------------------------------------------------------ */
/*  JNIEnv attach/detach per thread (tutorial pattern)                 */
/* ------------------------------------------------------------------ */
static JNIEnv *attach_current_thread(void) {
  JNIEnv *env;
  JavaVMAttachArgs args = { .version = JNI_VERSION_1_6, .name = "GStreamer", .group = NULL };
  if ((*java_vm)->AttachCurrentThread(java_vm, &env, &args) < 0) {
    LOGE("Failed to attach current thread");
    return NULL;
  }
  return env;
}

static void detach_current_thread(void *env) {
  (*java_vm)->DetachCurrentThread(java_vm);
}

static JNIEnv *get_jni_env(void) {
  JNIEnv *env = (JNIEnv *) pthread_getspecific(current_jni_env);
  if (env == NULL) {
    env = attach_current_thread();
    pthread_setspecific(current_jni_env, env);
  }
  return env;
}

/* ------------------------------------------------------------------ */
/*  Up-calls to Kotlin                                                 */
/* ------------------------------------------------------------------ */
static void call_void(CustomData *data, jmethodID mid) {
  JNIEnv *env = get_jni_env();
  (*env)->CallVoidMethod(env, data->app, mid);
  if ((*env)->ExceptionCheck(env)) { (*env)->ExceptionClear(env); }
}

static void notify_first_frame(CustomData *data) { call_void(data, on_first_frame_id); }
static void notify_stall(CustomData *data)       { call_void(data, on_stall_id); }
static void notify_outage(CustomData *data)      { call_void(data, on_outage_id); }
static void notify_recovered(CustomData *data)   { call_void(data, on_recovered_id); }

static void notify_size(CustomData *data, gint w, gint h) {
  JNIEnv *env = get_jni_env();
  (*env)->CallVoidMethod(env, data->app, on_size_changed_id, (jint) w, (jint) h);
  if ((*env)->ExceptionCheck(env)) { (*env)->ExceptionClear(env); }
}

static void notify_error(CustomData *data, const gchar *msg) {
  JNIEnv *env = get_jni_env();
  jstring jmsg = (*env)->NewStringUTF(env, msg ? msg : "error");
  (*env)->CallVoidMethod(env, data->app, on_error_id, jmsg);
  if ((*env)->ExceptionCheck(env)) { (*env)->ExceptionClear(env); }
  (*env)->DeleteLocalRef(env, jmsg);
}

/* ------------------------------------------------------------------ */
/*  Bus + probe callbacks                                              */
/* ------------------------------------------------------------------ */
static void error_cb(GstBus *bus, GstMessage *msg, CustomData *data) {
  GError *err; gchar *debug_info;
  gst_message_parse_error(msg, &err, &debug_info);
  LOGE("Pipeline error from %s: %s", GST_OBJECT_NAME(msg->src), err->message);
  gchar *m = g_strdup(err->message);
  g_clear_error(&err);
  g_free(debug_info);
  notify_error(data, m);
  g_free(m);
}

static void eos_cb(GstBus *bus, GstMessage *msg, CustomData *data) {
  LOGW("Pipeline EOS");
  notify_error(data, "eos");
}

/* Sink-pad buffer probe: first-frame, size, and stall recovery. */
static GstPadProbeReturn sink_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
  CustomData *data = (CustomData *) user_data;
  data->last_buffer_us = g_get_monotonic_time();

  if (data->stall_state != 0) {
    data->stall_state = 0;
    notify_recovered(data);
  }

  if (!data->first_frame_sent) {
    data->first_frame_sent = TRUE;
    /* Report size from the negotiated caps, once. */
    GstCaps *caps = gst_pad_get_current_caps(pad);
    if (caps) {
      GstStructure *s = gst_caps_get_structure(caps, 0);
      gint w = 0, h = 0;
      gst_structure_get_int(s, "width", &w);
      gst_structure_get_int(s, "height", &h);
      if (w > 0 && h > 0 && (w != data->size_w || h != data->size_h)) {
        data->size_w = w; data->size_h = h;
        notify_size(data, w, h);
      }
      gst_caps_unref(caps);
    }
    notify_first_frame(data);
  }
  return GST_PAD_PROBE_OK;
}

/* Periodic watchdog: UDP has no socket timeout, so detect a silent stall by
 * how long since the last buffer arrived. Drives WEAK SIGNAL / reconnect. */
static gboolean watchdog_cb(gpointer user_data) {
  CustomData *data = (CustomData *) user_data;
  if (!data->first_frame_sent) return TRUE;        /* still on initial connect */
  gint64 idle = g_get_monotonic_time() - data->last_buffer_us;
  if (idle > OUTAGE_US && data->stall_state < 2) {
    data->stall_state = 2;
    notify_outage(data);
  } else if (idle > STALL_US && data->stall_state < 1) {
    data->stall_state = 1;
    notify_stall(data);
  }
  return TRUE;
}

/* ------------------------------------------------------------------ */
/*  Pipeline build / teardown (run on the GMainContext thread)         */
/* ------------------------------------------------------------------ */
static void free_pipeline(CustomData *data) {
  if (data->watchdog_id) { GSource *s = g_main_context_find_source_by_id(data->context, data->watchdog_id); if (s) g_source_destroy(s); data->watchdog_id = 0; }
  if (data->pipeline) {
    gst_element_set_state(data->pipeline, GST_STATE_NULL);
    gst_object_unref(data->pipeline);
    data->pipeline = NULL;
  }
  data->video_sink = NULL;
  data->first_frame_sent = FALSE;
  data->stall_state = 0;
  data->size_w = data->size_h = 0;
}

static void build_pipeline(CustomData *data) {
  if (!data->uri) return;
  free_pipeline(data);

  /* FRONT/BACK are mounted upside-down -> rotate 180 in GL (no CPU download).
   * If the GL rotate path misbehaves on-device, swap the rotated branch for
   *   "! videoconvert ! videoflip method=rotate-180 ! glimagesink". */
  const gchar *rot = (data->rotation == 180)
      ? "glupload ! glvideoflip method=rotate-180 ! glimagesink name=sink sync=false"
      : "glimagesink name=sink sync=false";

  /* latency=100 = jitter buffer size (ms); do-retransmission=false avoids RTP
   * retransmit latency spikes (FPV wants fresh frames, not perfect ones). These
   * are valid rtspsrc properties; per-jitterbuffer knobs like drop-on-latency are
   * intentionally not set here (rtspsrc does not proxy them). */
  gchar *desc = g_strdup_printf(
      "rtspsrc location=%s protocols=udp latency=100 do-retransmission=false "
      "! rtph264depay ! h264parse ! decodebin ! %s",
      data->uri, rot);

  GError *err = NULL;
  data->pipeline = gst_parse_launch(desc, &err);
  g_free(desc);
  if (err) {
    LOGE("gst_parse_launch failed: %s", err->message);
    notify_error(data, err->message);
    g_clear_error(&err);
    return;
  }

  data->video_sink = gst_bin_get_by_name(GST_BIN(data->pipeline), "sink");
  if (!data->video_sink) {
    LOGE("Could not find video sink 'sink'");
    notify_error(data, "no sink");
    return;
  }
  /* gst_bin_get_by_name adds a ref; keep only a borrowed pointer. */
  gst_object_unref(data->video_sink);

  /* Bind the sink to the Android surface if we already have one. */
  if (data->native_window)
    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(data->video_sink),
                                        (guintptr) data->native_window);

  /* First-frame / size / stall-recovery probe on the sink pad. */
  GstPad *pad = gst_element_get_static_pad(data->video_sink, "sink");
  if (pad) {
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, sink_buffer_probe, data, NULL);
    gst_object_unref(pad);
  }

  /* Bus watch on our context. */
  GstBus *bus = gst_element_get_bus(data->pipeline);
  GSource *bus_source = gst_bus_create_watch(bus);
  g_source_set_callback(bus_source, (GSourceFunc) gst_bus_async_signal_func, NULL, NULL);
  g_source_attach(bus_source, data->context);
  g_source_unref(bus_source);
  g_signal_connect(G_OBJECT(bus), "message::error", (GCallback) error_cb, data);
  g_signal_connect(G_OBJECT(bus), "message::eos",   (GCallback) eos_cb, data);
  gst_object_unref(bus);

  /* Reset watchdog bookkeeping and start it. */
  data->last_buffer_us = g_get_monotonic_time();
  data->stall_state = 0;
  data->first_frame_sent = FALSE;
  GSource *wd = g_timeout_source_new(WATCHDOG_INTERVAL_MS);
  g_source_set_callback(wd, watchdog_cb, data, NULL);
  data->watchdog_id = g_source_attach(wd, data->context);
  g_source_unref(wd);

  gst_element_set_state(data->pipeline, GST_STATE_PLAYING);
  LOGI("Pipeline playing: %s (rot=%d)", data->uri, data->rotation);
}

/* ------------------------------------------------------------------ */
/*  GMainLoop thread                                                   */
/* ------------------------------------------------------------------ */
static void *app_function(void *userdata) {
  CustomData *data = (CustomData *) userdata;
  data->context = g_main_context_new();
  g_main_context_push_thread_default(data->context);
  data->main_loop = g_main_loop_new(data->context, FALSE);
  data->initialized = TRUE;
  LOGI("Entering main loop");
  g_main_loop_run(data->main_loop);
  LOGI("Exited main loop");
  g_main_loop_unref(data->main_loop);
  data->main_loop = NULL;
  g_main_context_pop_thread_default(data->context);
  g_main_context_unref(data->context);
  free_pipeline(data);
  return NULL;
}

/* ------------------------------------------------------------------ */
/*  Idle-dispatched pipeline ops (called from JNI, run on context)     */
/* ------------------------------------------------------------------ */
static gboolean idle_build(gpointer d)  { build_pipeline((CustomData *) d); return FALSE; }
static gboolean idle_stop(gpointer d)   { free_pipeline((CustomData *) d);  return FALSE; }

static void post_to_context(CustomData *data, GSourceFunc fn) {
  if (!data->context) return;
  GSource *s = g_idle_source_new();
  g_source_set_callback(s, fn, data, NULL);
  g_source_attach(s, data->context);
  g_source_unref(s);
}

/* ------------------------------------------------------------------ */
/*  JNI entry points — standard name-based binding (no RegisterNatives, */
/*  no FindClass in JNI_OnLoad, which is classloader-fragile on Android) */
/* ------------------------------------------------------------------ */
#define JFN(name) Java_com_avatarrobot_camswitcher_video_GStreamerVideoView_##name

static CustomData *get_data(JNIEnv *env, jobject thiz) {
  return (CustomData *) (gintptr) (*env)->GetLongField(env, thiz, custom_data_field_id);
}

JNIEXPORT void JNICALL JFN(nativeInit)(JNIEnv *env, jobject thiz) {
  /* Cache the field/method IDs from the instance's own class the first time. */
  if (!custom_data_field_id) {
    jclass klass = (*env)->GetObjectClass(env, thiz);
    custom_data_field_id = (*env)->GetFieldID(env, klass, "nativeCustomData", "J");
    on_first_frame_id  = (*env)->GetMethodID(env, klass, "onFirstFrameFromNative", "()V");
    on_size_changed_id = (*env)->GetMethodID(env, klass, "onSizeChangedFromNative", "(II)V");
    on_error_id        = (*env)->GetMethodID(env, klass, "onErrorFromNative", "(Ljava/lang/String;)V");
    on_stall_id        = (*env)->GetMethodID(env, klass, "onStallFromNative", "()V");
    on_outage_id       = (*env)->GetMethodID(env, klass, "onOutageFromNative", "()V");
    on_recovered_id    = (*env)->GetMethodID(env, klass, "onRecoveredFromNative", "()V");
  }
  CustomData *data = g_new0(CustomData, 1);
  data->rotation = 0;
  data->last_buffer_us = g_get_monotonic_time();
  (*env)->SetLongField(env, thiz, custom_data_field_id, (jlong) (gintptr) data);
  data->app = (*env)->NewGlobalRef(env, thiz);
  pthread_create(&gst_app_thread, NULL, &app_function, data);
}

JNIEXPORT void JNICALL JFN(nativeFinalize)(JNIEnv *env, jobject thiz) {
  CustomData *data = get_data(env, thiz);
  if (!data) return;
  if (data->main_loop) g_main_loop_quit(data->main_loop);
  pthread_join(gst_app_thread, NULL);
  (*env)->DeleteGlobalRef(env, data->app);
  g_free(data->uri);
  g_free(data);
  (*env)->SetLongField(env, thiz, custom_data_field_id, 0);
}

JNIEXPORT void JNICALL JFN(nativeSetUri)(JNIEnv *env, jobject thiz, jstring juri, jint rotation) {
  CustomData *data = get_data(env, thiz);
  if (!data) return;
  const gchar *uri = (*env)->GetStringUTFChars(env, juri, NULL);
  g_free(data->uri);
  data->uri = g_strdup(uri);
  data->rotation = rotation;
  (*env)->ReleaseStringUTFChars(env, juri, uri);
}

JNIEXPORT void JNICALL JFN(nativePlay)(JNIEnv *env, jobject thiz) {
  CustomData *data = get_data(env, thiz);
  if (data) post_to_context(data, idle_build);
}

JNIEXPORT void JNICALL JFN(nativeStop)(JNIEnv *env, jobject thiz) {
  CustomData *data = get_data(env, thiz);
  if (data) post_to_context(data, idle_stop);
}

JNIEXPORT void JNICALL JFN(nativeSurfaceInit)(JNIEnv *env, jobject thiz, jobject surface) {
  CustomData *data = get_data(env, thiz);
  if (!data) return;
  ANativeWindow *new_window = ANativeWindow_fromSurface(env, surface);
  if (data->native_window) {
    ANativeWindow_release(data->native_window);
  }
  data->native_window = new_window;
  if (data->video_sink) {
    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(data->video_sink),
                                        (guintptr) data->native_window);
    gst_video_overlay_expose(GST_VIDEO_OVERLAY(data->video_sink));
  }
}

JNIEXPORT void JNICALL JFN(nativeSurfaceFinalize)(JNIEnv *env, jobject thiz) {
  CustomData *data = get_data(env, thiz);
  if (!data) return;
  if (data->video_sink)
    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(data->video_sink), (guintptr) NULL);
  if (data->native_window) {
    ANativeWindow_release(data->native_window);
    data->native_window = NULL;
  }
}

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
  java_vm = vm;
  pthread_key_create(&current_jni_env, detach_current_thread);
  return JNI_VERSION_1_6;
}
