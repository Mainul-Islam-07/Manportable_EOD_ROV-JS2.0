# **Each Power Cycle Setup**
    perform after every time device is powered on
## Adapter Boot Up
### Boot Sequence Command
    # Run the following terminal commands to bring the CAN-USB adapter Online
    # may require password entry for sudo
    # Must run everytime after physical power-up / adapter reconnect / device unresponsive situation
    # It fresh starts the adapter
    # Original owner of this library "Fiaz Ahmed Tonmoy Khan" owns many adapters and their common CAN-USB protocols are the following

    gs_usb: OpenMoko, Inc. Geschwister Schneider CAN adapter
    peak_usb: PEAK System PCAN-USB
    no_name: Chicony Electronics Co., Ltd

    #### Chicony Electronics Co., Ltd (Does not use candlelight firmware, usage not recommanded for R&D; Don't ask why !) 

#### Command sequence (Copy paste in terminal and hit ENTER)
    sudo modprobe can
    sudo modprobe can_raw
    sudo modprobe can_dev
    sudo modprobe gs_usb
    sudo modprobe peak_usb
    sudo ip link set can0 down
    sudo ip link set can0 up type can bitrate 1000000 # Put your needed datarate
    # sudo ip link set can0 up type can bitrate 1000000 restart-ms 100 # Few device support this instead of previous line
    ip -details link show can0

### Boot Success Test
#### Boot Acknowledgement Message
    5: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
        link/can  promiscuity 0  allmulti 0 minmtu 0 maxmtu 0 
        can state ERROR-ACTIVE restart-ms 0 
        bitrate 1000000 sample-point 0.747
        tq 5 prop-seg 63 phase-seg1 63 phase-seg2 43 sjw 21 brp 1
        gs_usb: tseg1 2..256 tseg2 2..128 sjw 1..128 brp 1..512 brp_inc 1
        clock 170000000 numtxqueues 1 numrxqueues 1 gso_max_size 65536 gso_max_segs 65535 tso_max_size 65536 tso_max_segs 65535 gro_max_size 65536 parentbus usb parentdev 1-1:1.0 
#### Communication test (Optional)
    1. Dump CAN messages in another terminal with one of the the following command
        candump can0      # Show raw CAN frames from can0 without timestamps
        candump -ta can0  # Show CAN frames with absolute timestamp
        candump -tz can0  # Show CAN frames with zero-based relative timestamp from start of candump
    2. Send CAN packet into the CAN Wire. They will be printed in the terminal
        (1779636842.634870)  can0  608   [8]  40 01 10 00 00 00 00 00
        (1779636842.635131)  can0  588   [8]  4F 01 10 00 00 00 00 00

    Now you are sure that Host device is live in CAN communication context