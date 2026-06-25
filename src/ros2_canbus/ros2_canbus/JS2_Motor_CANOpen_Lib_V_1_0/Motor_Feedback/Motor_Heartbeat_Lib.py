import time
import threading
import canopen

from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.CANopen_Network.Network_Lib import CANopen_Network
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Settings.Load_Settings_Lib import Load_Settings
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.User_Callback.Heartbeat_Callback_Lib import Heartbeat_Callback
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Telemetry_Lib import Motor_Telemetry
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Mapping.mapping import Avatarrobot_CANopen_Map


class Heartbeat_Lib:
    def __init__(
            self,
            node: canopen.RemoteNode,
            node_name: str,
            canopen_handle: CANopen_Network,
            settings: Load_Settings,
            telemetry: Motor_Telemetry,
            heartbeat_timeout_ms: int = 3000,
            ):

        try:
            self.node = node
            self.Node_ID = self.node.id
            self.Node_Name = node_name
            self.canopen_handle = canopen_handle
            self.settings = settings
            self.telemetry = telemetry

            self.FAILURE_EXIT = self.settings.FAILURE_EXIT
            self.DEBUG_HEARTBEAT = self.settings.DEBUG_HEARTBEAT

            self.common = Common(self.Node_ID, self.Node_Name)
            self.log = Logger(self.Node_Name, self.Node_ID)

            # Heartbeat supervision settings
            self.heartbeat_timeout_ms = int(heartbeat_timeout_ms)
            self.heartbeat_timeout_s = self.heartbeat_timeout_ms / 1000.0

            # Runtime heartbeat state
            self.hbcount = 0
            self.hbstate = self._safe_unknown_state()
            self.is_heartbeat = False
            self.last_heartbeat_time = None
            self.last_heartbeat_timestamp = None

            # Internal control
            self._running = True
            self._stop_event = threading.Event()
            self._lock = threading.Lock()
            self._timeout_logged = False

            # User callback handler
            self.callback = Heartbeat_Callback(
                self.node,
                self.Node_Name,
                self.canopen_handle,
                self.telemetry,
                self.settings
            )

            # Subscribe only after all variables are ready
            self.HEARTBEAT_ID = self.canopen_handle.get_canopen_id(
                self.Node_ID,
                "heartbeat"
            )

            self.canopen_handle.network.subscribe(
                self.HEARTBEAT_ID,
                self.heartbeat_interrupt
            )

            # Watchdog thread detects heartbeat absence
            self._watchdog_thread = threading.Thread(
                target=self._heartbeat_watchdog,
                name=f"HeartbeatWatchdog_Node_{self.Node_ID}",
                daemon=True
            )
            self._watchdog_thread.start()

            self.DEBUG_HEARTBEAT and self.log.print(
                f"Heartbeat monitor started. COB-ID=0x{self.HEARTBEAT_ID:X}, timeout={self.heartbeat_timeout_ms} ms",
                "OK",
                "Heartbeat_Lib",
                "__init__",
                "START"
            )

        except Exception as e:
            try:
                self.log.print(
                    "Heartbeat setup failure",
                    "ERROR",
                    "Heartbeat_Lib",
                    "__init__",
                    f"{e}"
                )
            except Exception:
                print(f"Heartbeat setup failure: {e}")

            if getattr(self, "FAILURE_EXIT", False):
                exit(1)

    def _safe_unknown_state(self):
        try:
            return Avatarrobot_CANopen_Map.HeartbeatState.UNKNOWN
        except Exception:
            return None

    def decode_heartbeat_state(self, data):
        """
        CANopen heartbeat data byte:
            0x00 = Boot-up
            0x04 = Stopped
            0x05 = Operational
            0x7F = Pre-operational
        """
        try:
            if data is None or len(data) == 0:
                return self._safe_unknown_state()

            state_value = int(data[0]) & 0x7F

            for state in Avatarrobot_CANopen_Map.HeartbeatState:
                if int(state) == state_value:
                    return state

            return self._safe_unknown_state()

        except Exception:
            return self._safe_unknown_state()

    def heartbeat_interrupt(self, cob_id, data, timestamp):
        """
        This function is called automatically by python-canopen
        whenever heartbeat frame 0x700 + Node_ID is received.
        """
        try:
            if not self._running:
                return

            now = time.monotonic()
            decoded_state = self.decode_heartbeat_state(data)

            with self._lock:
                self.hbcount += 1
                self.hbstate = decoded_state
                self.last_heartbeat_time = now
                self.last_heartbeat_timestamp = timestamp
                self.is_heartbeat = True
                self._timeout_logged = False
                current_count = self.hbcount

            # Forward to user callback
            self.callback.heartbeat_callback(data, current_count)

            self.DEBUG_HEARTBEAT and self.log.print(
                f"Heartbeat received. Count={current_count}, State={decoded_state}",
                "OK",
                "Heartbeat_Lib",
                "heartbeat_interrupt",
                f"COB-ID=0x{cob_id:X}"
            )

        except Exception as e:
            try:
                self.DEBUG_HEARTBEAT and self.log.print(
                    "HB callback error",
                    "ERROR",
                    "Heartbeat_Lib",
                    "heartbeat_interrupt",
                    f"{e}"
                )
            except Exception:
                print(f"HB callback error: {e}")

    def _heartbeat_watchdog(self):
        """
        Independent watchdog.
        Callback detects heartbeat arrival.
        Watchdog detects heartbeat absence.
        """
        check_interval_s = min(0.5, max(0.05, self.heartbeat_timeout_s / 10.0))

        while not self._stop_event.is_set():
            try:
                now = time.monotonic()

                with self._lock:
                    if self.last_heartbeat_time is None:
                        self.is_heartbeat = False
                        elapsed_s = None
                    else:
                        elapsed_s = now - self.last_heartbeat_time

                        if elapsed_s > self.heartbeat_timeout_s:
                            self.is_heartbeat = False

                            if not self._timeout_logged:
                                self._timeout_logged = True
                                should_log_timeout = True
                            else:
                                should_log_timeout = False
                        else:
                            self.is_heartbeat = True
                            should_log_timeout = False

                if elapsed_s is not None and should_log_timeout:
                    self.DEBUG_HEARTBEAT and self.log.print(
                        f"Heartbeat timeout. No heartbeat for {elapsed_s:.3f} s",
                        "TIMEOUT",
                        "Heartbeat_Lib",
                        "_heartbeat_watchdog",
                        f"timeout={self.heartbeat_timeout_ms} ms"
                    )

            except Exception as e:
                try:
                    self.DEBUG_HEARTBEAT and self.log.print(
                        "Heartbeat watchdog error",
                        "ERROR",
                        "Heartbeat_Lib",
                        "_heartbeat_watchdog",
                        f"{e}"
                    )
                except Exception:
                    print(f"Heartbeat watchdog error: {e}")

            self._stop_event.wait(check_interval_s)

    def get_status(self) -> dict:
        """
        Safe readout for GUI / dashboard / telemetry.
        """
        with self._lock:
            return {
                "node_id": self.Node_ID,
                "node_name": self.Node_Name,
                "heartbeat_cob_id": self.HEARTBEAT_ID,
                "is_heartbeat": self.is_heartbeat,
                "hbcount": self.hbcount,
                "hbstate": self.hbstate,
                "last_heartbeat_time": self.last_heartbeat_time,
                "last_heartbeat_timestamp": self.last_heartbeat_timestamp,
                "heartbeat_timeout_ms": self.heartbeat_timeout_ms,
            }

    def stop(self):
        """
        Cleanly stop heartbeat reading.
        Do not use del object directly.
        """
        try:
            self._running = False
            self._stop_event.set()

            # Stop watchdog thread
            if hasattr(self, "_watchdog_thread") and self._watchdog_thread.is_alive():
                self._watchdog_thread.join(timeout=1.0)

            # Unsubscribe callback if python-canopen/network wrapper supports it
            try:
                self.canopen_handle.network.unsubscribe(
                    self.HEARTBEAT_ID,
                    self.heartbeat_interrupt
                )
            except Exception:
                pass

            with self._lock:
                self.is_heartbeat = False

            self.DEBUG_HEARTBEAT and self.log.print(
                "Heartbeat monitor stopped",
                "OK",
                "Heartbeat_Lib",
                "stop",
                f"Node={self.Node_ID}"
            )

        except Exception as e:
            try:
                self.log.print(
                    "Heartbeat stop failed",
                    "ERROR",
                    "Heartbeat_Lib",
                    "stop",
                    f"{e}"
                )
            except Exception:
                print(f"Heartbeat stop failed: {e}")

    def close(self):
        self.stop()