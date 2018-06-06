import os
import time
import numpy as np
import dynamixel_functions as dynamixel
from scipy.spatial.distance import euclidean


class Motor_Disable:
    def __init__(self):
        pass

    def motor_disable(self, theta1_vp, theta2_vp):
        if os.name == 'nt':
            import msvcrt
            def getch():
                return msvcrt.getch().decode()
        else:
            import sys, tty, termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)

            def getch():
                try:
                    tty.setraw(sys.stdin.fileno())
                    ch = sys.stdin.read(1)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                return ch

        # Control table address
        ADDR_MX_TORQUE_ENABLE = 24  # 1 byte     RW                         # Control table address is different in Dynamixel model

        # Protocol version
        PROTOCOL_VERSION = 1  # See which protocol version is used in the Dynamixel

        # Default setting
        DXL_ID = [1, 2, 3, 4]

        BAUDRATE = 1000000
        DEVICENAME = "COM7".encode('utf-8')  # Check which port is being used on your controller
        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
        # Resolution is 0.088 degree / Position 0~4095 /
        TORQUE_ENABLE = 1  # Value for enabling the torque
        TORQUE_DISABLE = 0  # Value for disabling the torque
        DXL_MOVING_STATUS_THRESHOLD = 3  # Dynamixel moving status threshold

        ESC_ASCII_VALUE = 0x1b  # for ESC key to escape out from the operation

        COMM_SUCCESS = 0  # Communication Success result value
        COMM_TX_FAIL = -1001  # Communication Tx Failed

        # Initialize PortHandler Structs
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        port_num = dynamixel.portHandler(DEVICENAME)

        # Initialize PacketHandler Structs
        dynamixel.packetHandler()

        index = 0
        dxl_comm_result = COMM_TX_FAIL  # Communication result

        dxl_error = 0  # Dynamixel error
        dxl_present_position = 0  # Present position

        # Disable Dynamixel Torque
        for id in DXL_ID:
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

        dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
        dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))

        # Close port
        dynamixel.closePort(port_num)