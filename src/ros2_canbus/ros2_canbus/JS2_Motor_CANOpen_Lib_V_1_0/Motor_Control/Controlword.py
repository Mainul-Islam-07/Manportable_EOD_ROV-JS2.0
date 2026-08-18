import canopen, time, threading

from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Logger_Lib import Logger
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Housekeeping.Common_Lib import Common
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.CANopen_Network.Network_Lib import CANopen_Network
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Settings.Load_Settings_Lib import Load_Settings
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_PDO_Lib import PDO_Lib
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_Telemetry_Lib import Motor_Telemetry
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Mapping.mapping import Avatarrobot_CANopen_Map
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Control.Motor_PDO_Lib import PDO_Lib
from ros2_canbus.JS2_Motor_CANOpen_Lib_V_1_0.Motor_Settings.Set_Mode_Lib import Mode

class Controlword_Setup():
    def __init__(
            self, node: canopen.RemoteNode,
            node_name: str,
            canopen_handle: CANopen_Network,
            settings: Load_Settings,
            telemetry: Motor_Telemetry,
            mode: Mode):
        try:
            self.node = node
            self.Node_ID = self.node.id
            self.Node_Name = node_name
            self.canopen_handle = canopen_handle
            self.settings = settings
            self.telemetry = telemetry
            self.common = Common(self.Node_ID, self.Node_Name)
            self.log = Logger(self.Node_Name, self.Node_ID)
            self.pdo = PDO_Lib(
                            self.node, 
                            self.Node_Name)
            self.DEBUG_INIT = self.settings.DEBUG_INIT and True
            self.DEBUG_CONTROLWORD = self.settings.DEBUG_CONTROLWORD and True
            self.mode = mode
            self.configure()
            # self.set_command()
            self.ispositionmode = False
            # self._init_nonblocking_disarm(delay_sec=2.0, cooldown_sec=0.2)


            self.DEBUG_INIT and self.log.print(f"Starting Controlword setup...","INFO", "Controlword_Setup","__init__", "CONTROLWORD_SETUP")

        except Exception as e:
            self.log.print(f"Setup Failure","❌","Controlword_Setup","__init__",f"{e}")

    def configure(self):
        try:
            self.rpdo = {
                "Target":    self.pdo.configure_pdo_attempt_multiple(
                                                                            node_id=self.Node_ID,
                                                                            pdo_name="RPDO2",
                                                                            variables=["Controlword"],
                                                                            trans_type=255,
                                                                            event_timer=None,
                                                                            enabled=True
                                                                        ),
                "Universal": self.pdo.configure_pdo_attempt_multiple(
                                                                            node_id=1,
                                                                            pdo_name="RPDO3",
                                                                            variables=["Controlword"],
                                                                            trans_type=255,
                                                                            event_timer=None,
                                                                            enabled=True
                                                                        )
            }
            return self.rpdo
        except Exception as e:
            self.log.print(f"Controlword PDO Configuration Failure","❌","Controlword_Setup","configure",f"{e}")
            if self.settings.FAILURE_EXIT:
                self.log.print(f"Exiting due to configuration failure as FAILURE_EXIT is set to True.","❌","Controlword_Setup","configure",f"Exiting program.")
                exit(1)
    
    def CONTROLWORD(self, controlword: int):
        try:
            self.node.sdo['Controlword'].raw = controlword
            self.common.delay_SDO()
            self.DEBUG_CONTROLWORD and self.log.print(f"Controlword set to {controlword:04x}", "SDO", "CONTROLWORD")
        except Exception as e:
            self.DEBUG_CONTROLWORD and self.log.print(f"CONTROLWORD {controlword:04x} error", "❌", "CONTROLWORD", f"{e}")
    
    def controlword(self, controlword: int):
        try:
            self.rpdo["Target"]["Controlword"].raw = controlword
            self.rpdo["Target"].transmit()
            self.DEBUG_CONTROLWORD and self.log.print(f"Controlword set to {controlword:04x}", "PDO", "CONTROLWORD")
            self.common.delay_PDO()
        except Exception as e:
            self.DEBUG_CONTROLWORD and self.log.print(f"controlword {controlword:04x} error", "❌", "controlword", f"{e}")

    def ARM(self):
        try:
            # Clear any LATCHED fault first. In CiA-402 a drive in the FAULT
            # state ignores Shutdown/Switch-On/Enable-Operation entirely — the
            # only exit is a rising edge of the fault-reset bit. Without this,
            # a motor that faulted (over-current on a hard reversal, dropped
            # controlword PDO, brief supply sag) stays braked and silently
            # refuses to run until the next DISARM. RESET_FAULT (bit 7) then
            # SHUT_DOWN (bit 7 low) makes that edge; it is benign when no fault
            # is present. Same pattern DISARM() already uses.
            self.CONTROLWORD(Avatarrobot_CANopen_Map.ControlWord.RESET_FAULT)
            self.CONTROLWORD(Avatarrobot_CANopen_Map.ControlWord.SHUT_DOWN)
            self.CONTROLWORD(Avatarrobot_CANopen_Map.ControlWord.SWITCH_ON)
            self.CONTROLWORD_HALT()
            self.log.print(f"Set command to 0 before disarm", "SDO","CONTROLWORD")
            self.CONTROLWORD(Avatarrobot_CANopen_Map.ControlWord.ENABLE_VOLTAGE)

            self.DEBUG_CONTROLWORD and self.log.print(f"Motor set to ARM", "SDO", "COMBO")

        except Exception as e:
            self.DEBUG_CONTROLWORD and self.log.print(f"ARM error", "❌", "ARM", "SDO", f"{e}")
    
    def DISARM(self):
        try:
            self.CONTROLWORD_HALT()
            self.log.print(f"Set command to 0 before disarm", "SDO", "CONTROLWORD")
            self.CONTROLWORD(Avatarrobot_CANopen_Map.ControlWord.SHUT_DOWN)
            self.CONTROLWORD(Avatarrobot_CANopen_Map.ControlWord.RESET_FAULT)
            self.CONTROLWORD(Avatarrobot_CANopen_Map.ControlWord.DISABLE_VOLTAGE)


            self.DEBUG_CONTROLWORD and self.log.print(f"Motor set to DISARM", "SDO", "COMBO")

        except Exception as e:
            self.DEBUG_CONTROLWORD and self.log.print(f"DISARM error", "❌", "DISARM", "SDO", f"{e}")

    def arm(self):
        try:
            self.controlword(Avatarrobot_CANopen_Map.ControlWord.SHUT_DOWN)
            self.controlword(Avatarrobot_CANopen_Map.ControlWord.SWITCH_ON)
            self.controword_halt()
            self.log.print(f"Set command to 0 before disarm", "PDO", "controlword")
            self.controlword(Avatarrobot_CANopen_Map.ControlWord.ENABLE_VOLTAGE)
            

            self.DEBUG_CONTROLWORD and self.log.print(f"Motor set to arm", "PDO", "combo")

        except Exception as e:
            self.DEBUG_CONTROLWORD and self.log.print(f"arm error", "❌", "arm", "PDO", f"{e}")

    def disarm(self):
        try:
            self.controword_halt()
            self.log.print(f"Set command to 0 before disarm", "PDO", "controlword")
            self.controlword(Avatarrobot_CANopen_Map.ControlWord.SHUT_DOWN)
            self.controlword(Avatarrobot_CANopen_Map.ControlWord.RESET_FAULT)
            self.controlword(Avatarrobot_CANopen_Map.ControlWord.DISABLE_VOLTAGE)


            self.DEBUG_CONTROLWORD and self.log.print(f"Motor set to disarm", "PDO", "combo")

        except Exception as e:
            self.DEBUG_CONTROLWORD and self.log.print(f"disarm error", "❌", "disarm", "PDO", f"{e}")

    def controword_halt(self):
        mode = self.telemetry.data.settings.operationmode.read
        match mode:
            
            case Avatarrobot_CANopen_Map.ModesOfOperation.CYCLIC_SYNCHRONOUS_TORQUE_MODE:
                self.mode.torque.run(0)
                self.common.delay_PDO()
            
            case Avatarrobot_CANopen_Map.ModesOfOperation.CYCLIC_SYNCHRONOUS_VELOCITY_MODE:
                self.mode.velocity.run(0)
                self.common.delay_PDO()

            case Avatarrobot_CANopen_Map.ModesOfOperation.CYCLIC_SYNCHRONOUS_POSITION_MODE:
                self.mode.position.run(
                    target_position=self.telemetry.data.command.position,
                    profile_velocity=0
                )
                self.common.delay_PDO()

            case _:
                self.DEBUG_CONTROLWORD and self.log.print(f"Unsupported mode for command setting: {mode}. Command set to NONE")

    def CONTROLWORD_HALT(self):
        try:
            mode = self.telemetry.data.settings.operationmode.read
            match mode:
                
                case Avatarrobot_CANopen_Map.ModesOfOperation.CYCLIC_SYNCHRONOUS_TORQUE_MODE:
                    self.node.sdo['Target_torque'].raw = 0
                    self.common.delay_SDO()
                
                case Avatarrobot_CANopen_Map.ModesOfOperation.CYCLIC_SYNCHRONOUS_VELOCITY_MODE:
                    self.node.sdo['Target_velocity'].raw = 0
                    self.common.delay_SDO()

                case Avatarrobot_CANopen_Map.ModesOfOperation.CYCLIC_SYNCHRONOUS_POSITION_MODE:
                    
                    self.node.sdo['Profile_velocity'].raw = 0
                    self.common.delay_SDO()
                    self.node.sdo['Target_position'].raw = self.telemetry.data.command.position
                    self.common.delay_SDO()
                case _:
                    self.DEBUG_CONTROLWORD and self.log.print(f"Unsupported mode for command setting: {mode}. Command set to NONE")
        except Exception as e:
            self.DEBUG_CONTROLWORD and self.log.print("Error in setting controlword 0", "❌", "CONTROLWORD_HALT", "Controlword_Setup", f"{e}")


    # # ---- add these imports once ----
 
    # # ---- paste these methods into your existing class ----

    # def _nbd_log(self, *args):
    #     """Safe log wrapper (uses your self.log.print if available)."""
    #     try:
    #         if hasattr(self, "DEBUG_CONTROLWORD") and self.DEBUG_CONTROLWORD:
    #             if hasattr(self, "log") and hasattr(self.log, "print"):
    #                 self.log.print(*args)
    #     except Exception:
    #         pass

    # def _init_nonblocking_disarm(self, delay_sec: float = 2.0, cooldown_sec: float = 0.2):
    #     """
    #     Call once (e.g., from your __init__). If you forget, disarm() will lazily init.
    #     delay_sec: wait after QS before Shutdown (fixed; no feedback)
    #     cooldown_sec: ignore rapid re-calls to avoid churn
    #     """
    #     self._disarm_delay = float(delay_sec)
    #     self._disarm_cooldown = float(cooldown_sec)
    #     self._disarm_lock = threading.Lock()
    #     self._disarm_thread = None
    #     self._last_disarm_start = 0.0
    #     # Optional flag other code can check to stop publishing setpoints
    #     self.safe_state = False

    # def is_disarming(self) -> bool:
    #     t = getattr(self, "_disarm_thread", None)
    #     return bool(t and t.is_alive())

    # def disarm(self) -> bool:
    #     """
    #     Non-blocking disarm:
    #     - QS now
    #     - spawn one daemon thread that, after 2s, sends SD -> FR -> DV
    #     Returns:
    #     True  = started a new disarm worker
    #     False = a worker is already running (slide off)
    #     """
    #     # Lazy init if _init_nonblocking_disarm() wasn't called
    #     if not hasattr(self, "_disarm_lock"):
    #         self._init_nonblocking_disarm()

    #     now = time.monotonic()
    #     with self._disarm_lock:
    #         # Debounce: avoid churning if user hammers the button
    #         if (now - getattr(self, "_last_disarm_start", 0.0)) < getattr(self, "_disarm_cooldown", 0.2):
    #             return False
    #         # If a worker is running, slide off
    #         if self._disarm_thread and self._disarm_thread.is_alive():
    #             return False

    #         # Immediate QS (returns fast; still non-blocking overall)
    #         try:
    #             print("Sending QS")
    #             # self.controlword(Avatarrobot_CANopen_Map.ControlWord.QUICK_STOP)
    #             self.command.run(0)
    #         except Exception as e:
    #             self._nbd_log(f"QS send failed in disarm(): {e}", "❌", "DISARM")

    #         # Mark state and spawn worker
    #         self.safe_state = True
    #         self._last_disarm_start = now
    #         self._disarm_thread = threading.Thread(
    #             target=self._disarm_worker, name="disarm-worker", daemon=True
    #         )
    #         self._disarm_thread.start()
    #         self._nbd_log("Disarm scheduled (QS→+2.0s→SD→FR→DV)", "PDO", "combo")
    #         return True

    # def _disarm_worker(self):
    #     """Runs in background once; no feedback reads, best-effort sequence."""
    #     delay = getattr(self, "_disarm_delay", 2.0)
    #     try:
    #         time.sleep(delay)  # fixed grace after QS

    #         # Shutdown (your unit locks brake here)
    #         try:
    #             print("Sending SD")
    #             self.controlword(Avatarrobot_CANopen_Map.ControlWord.SHUT_DOWN)
    #             self.common.delay_PDO()
    #         except Exception as e:
    #             self._nbd_log(f"SD send failed: {e}", "❌", "DISARM")

    #         # Fault Reset (harmless if no fault), then finish DV
    #         try:
    #             self.controlword(Avatarrobot_CANopen_Map.ControlWord.RESET_FAULT)
    #             self.common.delay_PDO()
    #         except Exception as e:
    #             self._nbd_log(f"FR send failed: {e}", "❌", "DISARM")

    #         try:
    #             self.controlword(Avatarrobot_CANopen_Map.ControlWord.DISABLE_VOLTAGE)
    #         except Exception as e:
    #             self._nbd_log(f"DV send failed: {e}", "❌", "DISARM")

    #         self._nbd_log("Disarm finished (QS→SD→FR→DV)", "PDO", "combo")

    #     except Exception as e:
    #         self._nbd_log(f"Disarm worker crashed: {e}", "❌", "DISARM")

    #     finally:
    #         # Cool-down before clearing safe_state (no feedback—just a tiny buffer)
    #         try:
    #             time.sleep(0.1)
    #         except Exception:
    #             pass
    #         self.safe_state = False
    #         # Clear thread handle so future calls can schedule again
    #         with getattr(self, "_disarm_lock", threading.Lock()):
    #             self._disarm_thread = None
