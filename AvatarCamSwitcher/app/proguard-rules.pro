# Keep the RTSP widget + its public listener interface intact under R8/ProGuard.
-keep class com.alexvas.rtsp.widget.** { *; }
-keep interface com.alexvas.rtsp.widget.** { *; }
