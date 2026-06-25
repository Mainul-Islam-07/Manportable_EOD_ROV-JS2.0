# **One Time Setup**
    Perform after hardware purchase
## Hardware
### Host
    Ubunto 20 and above
### Adapter
    CAN-USB Adapter
    1. SocketCAN compatible
    2. Candlelight Firmware loaded
    3. CAN 2.0 A,B supported (Only A [11bit - identifier] is used)
### Node
    Avatarroboticparts TD seriese Robot Joint Actuator - CANOpen CiA301 & CiA402 1 Mbps
    Link: https://avatarroboticparts.com/
## Packages
### Linux
    sudo apt install can-utils
### Python
    # use virtual environment if systemwide package installation is prohibited
    pip install canopen, openpyxl