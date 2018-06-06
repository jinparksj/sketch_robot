import os
import time
import numpy as np
import dynamixel_functions as dynamixel
from scipy.spatial.distance import euclidean
import cv2
import ikine
import contour_paper

theta1_offset = 2147
theta2_offset = 1994
theta3_offset = 1960

dynamixel_offset_90 = 1024
dynamixel_offset_180 = 2048

init_tx = 42
init_ty = 90

def dist(x1, y1, x2, y2):
    distance = euclidean([x1, y1], [x2, y2])
    return distance


class Motor:
    def __init__(self):
        pass

    def motor_operate(self, theta1_vp, theta2_vp, e, tx, ty, div_pt_x_vp, div_pt_y_vp):
        # ta1_vp and theta2_vp from list to float type

        theta1_vp = theta1_vp.tolist()
        theta2_vp = theta2_vp.tolist()

        for i in range(len(theta1_vp)):
            theta1_vp[i] = float(theta1_vp[i][0])
            theta2_vp[i] = float(theta2_vp[i][0])


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

        theta1 = []


        for i in range(len(theta1_vp)):
            theta1.append(int((180 * theta1_vp[i]) / (0.088 * np.pi)) + theta1_offset - 2048 + dynamixel_offset_90)
        print('=====================theta1 =======================')
        print(theta1)


        theta2 = []

        for i in range(len(theta2_vp)):
            theta2.append(int((180 * theta2_vp[i]) / (0.088 * np.pi)) + theta2_offset - 2048 + 2048)

        print('=====================theta2=======================')
        print(theta2)

        theta3 = [theta3_offset] * len(theta2_vp)
        #theta4 = [2048]

        # Open port
        if dynamixel.openPort(port_num):
            print("Succeeded to open the port!")
        else:
            print("Failed to open the port!")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if dynamixel.setBaudRate(port_num, BAUDRATE):
            print("Succeeded to change the baudrate!")
        else:
            print("Failed to change the baudrate!")
            print("Press any key to terminate...")
            getch()
            quit()

        #set position P gain, I gain, D gain - all 1 byte - P only 254 for unit communication
        # Motor 1
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_DERIVATIVE_GAIN, 254)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_INTEGRAL_GAIN, 30)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_PROPORTIONAL_GAIN, 80)

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

        # Estimate time
        start_time = time.time()
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_GOAL_POSITION, 2200)
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[1], ADDR_MX_GOAL_POSITION, 2200)
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[2], ADDR_MX_GOAL_POSITION, 2048)
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3], ADDR_MX_GOAL_POSITION, 2500)
        time.sleep(2)

        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_GOAL_POSITION, theta1[0])
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[1], ADDR_MX_GOAL_POSITION, theta2[0])
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[2], ADDR_MX_GOAL_POSITION, theta3[0])
        time.sleep(5)
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3], ADDR_MX_GOAL_POSITION, 2038)
        time.sleep(4)

        while not (len(theta1) - index) == 0:
            # Write goal position

            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_GOAL_POSITION, theta1[index])
            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[1], ADDR_MX_GOAL_POSITION, theta2[index])
            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[2], ADDR_MX_GOAL_POSITION, theta3[index])
            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3], ADDR_MX_GOAL_POSITION, 2038)


            dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
            dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
            if dxl_comm_result != COMM_SUCCESS:
                print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
            elif dxl_error != 0:
                print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))



            # write moving speed
            for id in DXL_ID:
                dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_MX_MOVING_SPEED, 50)
            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3], ADDR_MX_MOVING_SPEED, 20)

            while 1:
                # Read present position
                dxl_present_position_theta1 = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0],
                                                               ADDR_MX_PRESENT_POSITION)
                dxl_present_position_theta2 = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[1],
                                                               ADDR_MX_PRESENT_POSITION)
                dxl_present_position_theta3 = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[2],
                                                               ADDR_MX_PRESENT_POSITION)
                dxl_present_position_theta4 = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3],
                                                                      ADDR_MX_PRESENT_POSITION)


                dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
                dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)

                if dxl_comm_result != COMM_SUCCESS:
                    print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
                elif dxl_error != 0:
                    print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))

                # read voltage
                dxl_present_voltage = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0],
                                                             ADDR_MX_PRESENT_VOLTAGE)

                # read load
                dxl_present_load = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_PRESENT_LOAD)

                # read present speed
                dxl_present_vel = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_PRESENT_SPEED)

                #print("[ID:%03d] GoalPos:%03d  PresPos:%03d  PresVolt:%03d  PresLoad:%03d PresVel:%03d" % (
                #DXL_ID[0], theta2[index], dxl_present_position_theta2,
                #dxl_present_voltage, dxl_present_load, dxl_present_vel))
                #print('e', e.is_set())

                if e.is_set() == 0:
                    pass

                elif e.is_set() != 0:
                    dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3], ADDR_MX_GOAL_POSITION, 2500)
                    time.sleep(5)
                    dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_GOAL_POSITION, 2200)
                    dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[1], ADDR_MX_GOAL_POSITION, 2200)
                    dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[2], ADDR_MX_GOAL_POSITION, 2048)

                    #contour
                    cam_paper = cv2.VideoCapture(0)
                    ret_paper, frame_paper = cam_paper.read()
                    frame_paper = cv2.flip(frame_paper, -1)
                    cv2.imshow('frame', frame_paper)
                    cv2.waitKey(1)
                    cont_paper = contour_paper.ContourPaper()
                    tx, ty = cont_paper.cont_paper(frame_paper)

                    print('translated tx and ty', tx, ty)


                    #paper move


                    #new space for current theta1 and theta2 to return IKINE
                    new_theta1 = np.zeros([len(theta1) - index, 1])
                    new_theta2 = np.zeros([len(theta2) - index, 1])

                    #new space of current theta1 and theta2 in degree
                    theta1_deg = np.zeros([])
                    theta2_deg = np.zeros([])
                    theta1_deg = np.delete(theta1_deg, 0)
                    theta2_deg = np.delete(theta2_deg, 0)

                    #current theta1 and theta2, remove past theta value
                    current_theta1 = theta1[index: ]
                    current_theta2 = theta2[index: ]
                    div_pt_x_vp = div_pt_x_vp[index: ]
                    div_pt_y_vp = div_pt_y_vp[index: ]

                    #print('x position, ', div_pt_x_vp)
                    #print('imhere1')
                    for i in range(len(current_theta1)):
                        theta1_deg = np.append(theta1_deg, ((current_theta1[i] - theta1_offset + 2048 - dynamixel_offset_90) * np.pi * 0.088 / 180))
                        theta2_deg = np.append(theta2_deg, ((current_theta2[i] - theta2_offset) * np.pi * 0.088 / 180))




                    IK = ikine.InverseKinematics()
                    for i in range(len(current_theta2)):
                        #print('theta1_deg', theta1_deg[i])
                        new_theta1[i], new_theta2[i] = IK.ikine(np.int(div_pt_x_vp[i]), np.int(div_pt_y_vp[i]), tx, ty)

                    #print('new_theta1, ', new_theta1)
                    new_theta1 = new_theta1.tolist()
                    new_theta2 = new_theta2.tolist()


                    for i in range(len(new_theta1)):
                        new_theta1[i] = float(new_theta1[i][0])
                        new_theta2[i] = float(new_theta2[i][0])

                    theta1 = []

                    for i in range(len(new_theta1)):
                        theta1.append(int((180 * new_theta1[i]) / (0.088 * np.pi)) + theta1_offset - 2048 + dynamixel_offset_90)
                    print('=====================theta1 =======================')
                    print(theta1)

                    theta2 = []

                    for i in range(len(new_theta2)):
                        theta2.append(int((180 * new_theta2[i]) / (0.088 * np.pi)) + theta2_offset - 2048 + 2048)

                    print('=====================theta2=======================')
                    print(theta2)

                    theta3 = [theta3_offset] * len(new_theta2)

                    index = 0

                    time.sleep(5)
                    break
                
                
                else:
                    pass


                if not ((abs(theta1[index] - dxl_present_position_theta1) > DXL_MOVING_STATUS_THRESHOLD) | ((abs(theta2[index] - dxl_present_position_theta2)) > DXL_MOVING_STATUS_THRESHOLD) | ((abs(theta3[index] - dxl_present_position_theta3)) > DXL_MOVING_STATUS_THRESHOLD)) :
                    break

                # check event
            #ABOUT TX and TY -> IKINE -> New theta1 and theta2
            init_tx = tx
            init_ty = ty
            if e.is_set() != 0:
                print('error')
                dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[0], ADDR_MX_GOAL_POSITION, theta1[0])
                dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[1], ADDR_MX_GOAL_POSITION, theta2[0])
                dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[2], ADDR_MX_GOAL_POSITION, theta3[0])
                time.sleep(5)
                dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3], ADDR_MX_GOAL_POSITION, 2038)
                time.sleep(3)
                e.clear()

            else:
                pass


            index += 1

        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[3], ADDR_MX_GOAL_POSITION, 2500)
        time.sleep(5)







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

        return tx, ty


