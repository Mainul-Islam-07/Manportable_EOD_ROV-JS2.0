# **Software**
    # Here you enter into the library
    # Perform everytime software is run
    # This is where you use the python library
    # **It will be rewritten in C later**

# Import necessary library
    from CANopen_Network.Network_Lib         import CANopen_Network
    from Motor_Control.Motor_Lib import Motor_CANopen_Lib
    import time, sys
# Setup CANOpen protocol to the can network 
    net = {
        "Arm_CAN_Config":  CANopen_Network("CAN_Config.json", "Master", 1),
        "Carrier_CAN_Config":  CANopen_Network("Carrier_CAN_Config.json", "Master", 1),
    }
# Reset network and put it in preoperational mode 
    for net_name in net: 
        net[net_name].network_reset() 
        net[net_name].network_preoperational() 

    # Alternatively you can manipulate networks one by one
    net[Arm_CAN_Config].network_reset() 
    net[Arm_CAN_Config].network_preoperational()

    net[Carrier_CAN_Config].network_reset() 
    net[Carrier_CAN_Config].network_preoperational()
# Start reception of heartbeat for all motor
## Initialize heartbeat interrupt
    # telemetry reception is yet to set up (in the next phase). Since heartbeat is also a feedback, all other telemetry reception will show FAIL except heartbeat, which is normal
    heartbeat = {
        "Left_Drive"          : Motor_Heartbeat("Left_Drive"          , net["Arm_CAN_Config"] ,      "motor_settings.xlsx"),
        "Right_Drive"         : Motor_Heartbeat("Right_Drive"         , net["Arm_CAN_Config"] ,      "motor_settings.xlsx"),
        "Front_Flipper"       : Motor_Heartbeat("Front_Flipper"       , net["Arm_CAN_Config"] ,      "motor_settings.xlsx"),
        "Rear_Flipper"        : Motor_Heartbeat("Rear_Flipper"        , net["Arm_CAN_Config"] ,      "motor_settings.xlsx"),
        "Turret"              : Motor_Heartbeat("Turret"              , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Left_Differential"   : Motor_Heartbeat("Left_Differential"   , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Right_Differential"  : Motor_Heartbeat("Right_Differential"  , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Telescopic"          : Motor_Heartbeat("Telescopic"          , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Wrist"               : Motor_Heartbeat("Wrist"               , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Gripper_360"         : Motor_Heartbeat("Gripper_360"         , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Gripper"             : Motor_Heartbeat("Gripper"             , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
    }
## Power Cycle
    # Power On the CANOpen device
    **power_on()** # Written by user
## Wait
    # Wait a few second (Atleast 2X of timeout)
    time.sleep(6.0)
## Existance Check
    # Check if motor is online
    is_Left_Drive_alive         = heartbeat["Left_Drive"].heartbeat.is_heartbeat
    is_Right_Drive_alive        = heartbeat["Right_Drive"].heartbeat.is_heartbeat
    is_Front_Flipper_alive      = heartbeat["Front_Flipper"].heartbeat.is_heartbeat
    is_Rear_Flipper_alive       = heartbeat["Rear_Flipper"].heartbeat.is_heartbeat
    is_Turret_alive             = heartbeat["Turret"].heartbeat.is_heartbeat
    is_Left_Differential_alive  = heartbeat["Left_Differential"].heartbeat.is_heartbeat
    is_Right_Differential_alive = heartbeat["Right_Differential"].heartbeat.is_heartbeat
    is_Telescopic_alive         = heartbeat["Telescopic"].heartbeat.is_heartbeat
    is_Wrist_alive              = heartbeat["Wrist"].heartbeat.is_heartbeat
    is_Gripper_360_alive        = heartbeat["Gripper_360"].heartbeat.is_heartbeat
    is_Gripper_alive            = heartbeat["Gripper"].heartbeat.is_heartbeat
## Operation Decision
    Check if your desired nodes exist. Move on to the next phase if they are online (and operation is intended)
    select_motor_x() # Written by user

# Initialize all motors motor
    motor = {
        "Left_Drive"          : Motor_CANopen_Lib("Left_Drive"          , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Right_Drive"         : Motor_CANopen_Lib("Right_Drive"         , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Front_Flipper"       : Motor_CANopen_Lib("Front_Flipper"       , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Rear_Flipper"        : Motor_CANopen_Lib("Rear_Flipper"        , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Turret"              : Motor_CANopen_Lib("Turret"              , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Left_Differential"   : Motor_CANopen_Lib("Left_Differential"   , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Right_Differential"  : Motor_CANopen_Lib("Right_Differential"  , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Telescopic"          : Motor_CANopen_Lib("Telescopic"          , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Wrist"               : Motor_CANopen_Lib("Wrist"               , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Gripper_360"         : Motor_CANopen_Lib("Gripper_360"         , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
        "Gripper"             : Motor_CANopen_Lib("Gripper"             , net["Carrier_CAN_Config"] ,      "motor_settings.xlsx"),
    }
# Set the network as operational (Start receiving feedback data) 
    for net_name in net:
        net[net_name].network_operational()
        time.sleep(0.1)

    # Alternetively you can turn them operational One by One
    net[Arm_CAN_Config].network_operational()
    net[Carrier_CAN_Config].network_operational() 
# Arm any motor (Mechanical break loose but holding torque applied, large power consumption during idle state) 
    for motor_name in motor: # Arms all motor
        motor[motor_name].control.ARM() 
    time.sleep(1.0) ## Wait atleast 100ms before giving motion command to let breaks disengage properly

    # Alternatively you can arm specific motor (only the ones you need to move, saves power)
    # Re-Arming an armed motor will disarm and rearm it, so it's basically a double action and break will be heard twice per motor
        motor["Left_Drive"].control.ARM()
        motor["Left_Differential"].control.ARM()
        motor["Wrist"].control.ARM()
# Operate any motor

    # Positive Maneuver Test
    motor["Left_Drive"].mode.velocity.RUN(v)
    motor["Right_Drive"].mode.velocity.RUN(v)
    motor["Front_Flipper"].mode.position.RUN(p,v) # in case of position mode, velocity is magnitude and must be positive
    motor["Rear_Flipper"].mode.position.RUN(p,v)
    motor["Turret"].mode.position.RUN(p,v)
    motor["Left_Differential"].mode.position.RUN(p,v)
    motor["Right_Differential"].mode.position.RUN(p,v)
    motor["Telescopic"].mode.position.RUN(p,v)
    motor["Wrist"].mode.position.RUN(p,v)
    motor["Gripper_360"].mode.velocity.RUN(v)
    motor["Gripper"].mode.torque.RUN(t)

    time.sleep(5.0)

    # Stop Manuever Test
    motor["Left_Drive"].mode.velocity.RUN(0)
    motor["Right_Drive"].mode.velocity.RUN(0)
    motor["Front_Flipper"].mode.position.RUN(motor["Front_Flipper"].telemetry.data.feedback.position,v) # Set current position feedback as destination position command-> Stops the motor right where it is at the moment
    motor["Rear_Flipper"].mode.position.RUN(motor["Rear_Flipper"].telemetry.data.feedback.position,v)
    motor["Turret"].mode.position.RUN(motor["Turret"].telemetry.data.feedback.position,v)
    motor["Left_Differential"].mode.position.RUN(motor["Left_Differential"].telemetry.data.feedback.position,v)
    motor["Right_Differential"].mode.position.RUN(motor["Right_Differential"].telemetry.data.feedback.position,v)
    motor["Telescopic"].mode.position.RUN(motor["Telescopic"].telemetry.data.feedback.position,v)
    motor["Wrist"].mode.position.RUN(motor["Wrist"].telemetry.data.feedback.position,v)
    motor["Gripper_360"].mode.velocity.RUN(0)
    motor["Gripper"].mode.torque.RUN(0)

# Disarm the motors after operation (Holding torque cancelled, but mechanical break applied, consumes little power) 
    # Make sure you have stopped the motor from moving by issueing a stop command
    for motor_name in motor:# disarms all motor
        motor[motor_name].control.DISARM()

    # Alternatively you can disarm specific motor (the ones you armed only, parhaps. Disarming a non-armed motor does nothing really, but not an issue)
    # Make sure you have stopped the motor from moving
        motor["Right_Drive"].control.DISARM()
        motor["Turret"].control.DISARM()
        motor["Gripper"].control.DISARM()
# Error Clear / Reset Alarm
## Same as Disarm
    # Even though motors will stop anyway when alarm is generated, clearing alarm will immediately resume motor motion with last recent command. This is risky because if the alarm is from overload or stuck at something,         after clearing the alarm, further motion from previous command will cause the motor to get stuck even more. You do not want to move the motor immediately after moving. So issue a stop command before clearing alarm. 
# Disconnect from CANOpen network (Not needed, only use experimentally)
    for net_name in net: 
        net[net_name].disconnect()

# Command
    what_is_Telescopic_position_command = motor["Telescopic"].telemetry.data.command.position
    what_is_Left_Drive_velocity_command = motor["Left_Drive"].telemetry.data.command.velocity
    what_is_Gripper_torque_command = motor["Gripper"].telemetry.data.command.torque
    what_is_Gripper_360_Controlword = motor["Gripper_360"].telemetry.data.controlword

# Feedback
## Telemetry
    what_is_Left_Drive_position_feedback = motor["Left_Drive"].telemetry.data.feedback.position
    what_is_Right_Drive_velocity_feedback = motor["Right_Drive"].telemetry.data.feedback.velocity
    what_is_Front_Flipper_torque_feedback = motor["Front_Flipper"].telemetry.data.feedback.torque
## Settings
    what_is_Rear_Flipper_operationmode = motor["Rear_Flipper"].telemetry.data.settings.operationmode
## Metadata
    what_is_Turret_circuittemperature = motor["Turret"].telemetry.data.metadata.circuittemperature
    what_is_Left_Differential_coiltemperature = motor["Left_Differential"].telemetry.data.metadata.coiltemperature
    what_is_Right_Differential_voltage = motor["Right_Differential"].telemetry.data.metadata.voltage
    what_is_Telescopic_current = motor["Telescopic"].telemetry.data.metadata.current
    what_is_Wrist_errorcode = motor["Wrist"].telemetry.data.metadata.errorcode
    what_is_Gripper_360_errorregister = motor["Gripper_360"].telemetry.data.metadata.errorregister
    what_is_Gripper_statusword = motor["Gripper"].telemetry.data.metadata.statusword
    what_is_Telescopic_state = motor["Telescopic"].telemetry.data.metadata.current

## Parse Metadata
    # understand the difference between statusword and state. There is a 16 bit statusword register. The entire 16 bit register value is statusword. Each bit is a single status. Multiple status bit combined define the           hardware state (User deducts state from the status bit, in his software, this is not read from hardware)
### Statusword (Parse)
    is_Left_Drive_ready_to_switch_on_true = motor["Left_Drive"].telemetry.data.metadata.statusword.sw.ready_to_switch_on
    is_Right_Drive_switched_on_true = motor["Right_Drive"].telemetry.data.metadata.statusword.sw.switched_on
    is_Gripper_status_operation_enabled_true = motor["Gripper"].telemetry.data.metadata.statusword.sw.operation_enabled
    is_Front_Flipper_fault_true = motor["Front_Flipper"].telemetry.data.metadata.statusword.sw.fault
    is_Rear_Flipper_voltage_enabled_true = motor["Rear_Flipper"].telemetry.data.metadata.statusword.sw.voltage_enabled
    is_Turret_quick_stop_true = motor["Turret"].telemetry.data.metadata.statusword.sw.quick_stop
    is_Left_Differential_switch_on_disabled_true = motor["Left_Differential"].telemetry.data.metadata.statusword.sw.switch_on_disabled
    is_Right_Differential_warning_true = motor["Right_Differential"].telemetry.data.metadata.statusword.sw.warning
    is_Telescopic_reserved_8_true = motor["Telescopic"].telemetry.data.metadata.statusword.sw.reserved_8
    is_Gripper_remote_true = motor["Gripper"].telemetry.data.metadata.statusword.sw.remote
    is_Gripper_360_target_reached_true = motor["Gripper_360"].telemetry.data.metadata.statusword.sw.target_reached
    is_Wrist_internal_limit_active_true = motor["Wrist"].telemetry.data.metadata.statusword.sw.internal_limit_active
    is_Wrist_set_point_acknowledge_true = motor["Wrist"].telemetry.data.metadata.statusword.sw.set_point_acknowledge
    is_Front_Flipper_following_error_true = motor["Front_Flipper"].telemetry.data.metadata.statusword.sw.following_error
    is_Rear_Flipper_homing_attained_true = motor["Rear_Flipper"].telemetry.data.metadata.statusword.sw.homing_attained
    is_Turret_homing_error_true = motor["Turret"].telemetry.data.metadata.statusword.sw.homing_error
### State (Decide)
    what_is_Left_Drive_state = motor["Left_Drive"].telemetry.data.metadata.statusword.state
    what_is_Right_Drive_state = motor["Right_Drive"].telemetry.data.metadata.statusword.state
    what_is_Front_Flipper_state = motor["Front_Flipper"].telemetry.data.metadata.statusword.state
    what_is_Rear_Flipper_state = motor["Rear_Flipper"].telemetry.data.metadata.statusword.state
    what_is_Turret_state = motor["Turret"].telemetry.data.metadata.statusword.state
    what_is_Left_Differential_state = motor["Left_Differential"].telemetry.data.metadata.statusword.state
    what_is_Right_Differential_state = motor["Right_Differential"].telemetry.data.metadata.statusword.state
    what_is_Telescopic_state = motor["Telescopic"].telemetry.data.metadata.statusword.state
### Error Register (Parse, Universal)
    does_Left_Drive_generic_error_exist = motor["Left_Drive"].telemetry.data.metadata.errorregister.parsed.generic
    does_Right_Drive_current_error_exist = motor["Right_Drive"].telemetry.data.metadata.errorregister.parsed.current
    does_Front_Flipper_voltage_error_exist = motor["Front_Flipper"].telemetry.data.metadata.errorregister.parsed.voltage
    does_Rear_Flipper_temperature_error_exist = motor["Rear_Flipper"].telemetry.data.metadata.errorregister.parsed.temperature
    does_Turret_communication_error_exist = motor["Turret"].telemetry.data.metadata.errorregister.parsed.communication
    does_Left_Differential_device_profile_error_exist = motor["Left_Differential"].telemetry.data.metadata.errorregister.parsed.device_profile
    does_Right_Differential_reserved_error_exist = motor["Right_Differential"].telemetry.data.metadata.errorregister.parsed.reserved
    does_Telescopic_manufacturer_error_exist = motor["Telescopic"].telemetry.data.metadata.errorregister.parsed.manufacturer
### Error Code (Parse, Vendor Specific)
    is_Left_Drive_software_error_flash = motor["Left_Drive"].telemetry.data.metadata.errorcode.parsed.software_error_flash
    is_Right_Drive_overvoltage = motor["Right_Drive"].telemetry.data.metadata.errorcode.parsed.overvoltage
    is_Front_Flipper_undervoltage = motor["Front_Flipper"].telemetry.data.metadata.errorcode.parsed.undervoltage
    is_Rear_Flipper_startuperror = motor["Rear_Flipper"].telemetry.data.metadata.errorcode.parsed.startuperror
    is_Turret_speedfeedbackerror = motor["Turret"].telemetry.data.metadata.errorcode.parsed.speedfeedbackerror
    is_Left_Differential_overflow = motor["Left_Differential"].telemetry.data.metadata.errorcode.parsed.overflow
    is_Right_Differential_encodercommunication = motor["Right_Differential"].telemetry.data.metadata.errorcode.parsed.encodercommunication
    is_Telescopic_motor_temp_high = motor["Telescopic"].telemetry.data.metadata.errorcode.parsed.motor_temp_high
    is_Wrist_board_temp_high = motor["Wrist"].telemetry.data.metadata.errorcode.parsed.board_temp_high
