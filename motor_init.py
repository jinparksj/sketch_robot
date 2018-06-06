import os
import time
import dynamixel_functions as dynamixel

class Motor_Init:
    def __init__(self):
        pass

    def motor_init(self):
        # Read a keypress and return the resulting character as a byte string.

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
        ADDR_MX_GOAL_POSITION = 30  # 2 bytes    RW
        ADDR_MX_PRESENT_POSITION = 36  # 2 bytes    R
        ADDR_MX_MOVING_SPEED = 32  # 2 bytes    RW
        ADDR_MX_PRESENT_SPEED = 38  # 2 bytes    R
        ADDR_MX_PRESENT_LOAD = 40  # 2 bytes    R
        ADDR_MX_PRESENT_VOLTAGE = 42  # 1 byte     R
        ADDR_MX_GOAL_ACCELERATION = 73  # 1 byte     RW
        ADDR_MX_MAX_TORQUE = 14 # 2 bytes RW

        # PID Controller Address
        ADDR_MX_DERIVATIVE_GAIN = 26  # 1 byte     RW
        ADDR_MX_INTEGRAL_GAIN = 27  # 1 byte     RW
        ADDR_MX_PROPORTIONAL_GAIN = 28  # 1 byte     RW

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
        DXL_MOVING_STATUS_THRESHOLD = 5 # Dynamixel moving status threshold

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


        #set position P gain, I gain, D gain - all 1 byte - P only 254 for unit communication
        # Motor 1
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_DERIVATIVE_GAIN, 254)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_INTEGRAL_GAIN, 30)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_PROPORTIONAL_GAIN, 65)

        # Motor 2
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[1], ADDR_MX_DERIVATIVE_GAIN, 254)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[1], ADDR_MX_INTEGRAL_GAIN, 50)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[1], ADDR_MX_PROPORTIONAL_GAIN, 120)

        # Motor 3
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[2], ADDR_MX_DERIVATIVE_GAIN, 254)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[2], ADDR_MX_INTEGRAL_GAIN, 50)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[2], ADDR_MX_PROPORTIONAL_GAIN, 120)

        # Motor 4
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3], ADDR_MX_DERIVATIVE_GAIN, 254)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3], ADDR_MX_INTEGRAL_GAIN, 50)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3], ADDR_MX_PROPORTIONAL_GAIN, 120)

        for id in DXL_ID:
            # Enable Dynamixel Torque - 1byte
            dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_MX_MAX_TORQUE, 1024)
            dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
            dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)

            # Initialize Goal Position 2048
            dxl_goal_position_init = 2048
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3], ADDR_MX_MAX_TORQUE, 1024)


        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3], ADDR_MX_GOAL_POSITION, 2500)
        time.sleep(5)

        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_GOAL_POSITION, 2048)
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[1], ADDR_MX_GOAL_POSITION, 2048)
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[2], ADDR_MX_GOAL_POSITION, 2048)

        time.sleep(3)


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