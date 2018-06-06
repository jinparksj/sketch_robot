import os
import time
import numpy as np
import intro_sketch
import cv2
import face
import sort_vp
import ikine
import motor_operator
from prop_paper import ProperPaper
import motor_disable
from multiprocessing import Process, Event
import time
import imutils
import contour_paper

start_time = time.time()

face = face.Face()

tx = 40.
ty = 95.
init_tx = 40
init_ty = 95

def dist(x1, y1, x2, y2):
    dist = np.sqrt(np.square(x1 - x2) + np.square(y1 - y2))
    return dist

letter_size = [279, 216]

ESC_ASCII_VALUE = 0x1b
ENTER_ASCII_VALUE = 0x0d

# 0. Top Camera Open
def Top_Camera(e):

    cam_paper = cv2.VideoCapture(0)
    ret_paper, frame_paper = cam_paper.read()
    frame_paper = cv2.flip(frame_paper, -1)
    h, w = frame_paper.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    video_write = cv2.VideoWriter('saved_out.avi', fourcc, 30.0, (w, h))

    #contour of paper -> x, y, w, h
    global tx, ty
    cont_paper = contour_paper.ContourPaper()
    tx, ty = cont_paper.cont_paper(frame_paper)
    global init_tx, init_ty
    init_tx = tx
    init_ty = ty
    print('init_tx:',init_tx, 'init_ty:,', init_ty)

    print("===================Video Recording===================")
    while (cam_paper.isOpened()):
        ret_paper, frame_paper = cam_paper.read()
        frame_paper = cv2.flip(frame_paper, -1)
        video_write.write(frame_paper)
        cv2.imshow('paper', frame_paper)
        keyboard_input = cv2.waitKey(1)

        #Detect Contour
        tx, ty = cont_paper.cont_paper(frame_paper)

        '''
        if dist(init_tx, init_ty, tx, ty) > 50: #50mm displacement, event generating for compensation
            print(dist(init_tx, init_ty, tx, ty))
            print(tx, ty)
            print('e.set')
            e.set()
            break

        elif dist(init_tx, init_ty, tx, ty <= 50):
            pass


        '''
        if keyboard_input & 0xFF == ord('q'):
            break


    cam_paper.release()
    video_write.release()
    cv2.destroyAllWindows()



def Robot_Arm(e):
    time.sleep(2)
    #1. Face Camera
    cam_intro = cv2.VideoCapture(1)
    intro = intro_sketch.IntroSketch()
    # intro.intro_sketch()
    #
    # intro.enter_esc()
    print("Press Enter key to choose sketching face! or press ESC to quit!")
    while (cam_intro.isOpened()):

        ret, frame_face_select = cam_intro.read()
        frame_face_select = cv2.flip(frame_face_select, 1)
        # cv2.rectangle(frame_face_select, (150, 100), (450, 400), (255, 255, 255))
        cv2.imshow('frame', frame_face_select)

        #frame_face_select = cv2.imread('santos.jpg')

        if cv2.waitKey(1) & 0xFF == 13:
            cv2.imwrite('C:/Code/sketchrobot/image/sketch_intro.jpg', frame_face_select)
            sketch_intro = cv2.imread('C:/Code/sketchrobot/image/sketch_intro.jpg')
            cv2.imshow('sketch_intro', sketch_intro)
            cv2.waitKey()
            break
        elif cv2.waitKey(1) & 0xFF == 27:
            exit(0)

    cam_intro.release()
    cv2.destroyAllWindows()

    #1. face recognition from camera 0 in laptop / output viapoints x-y plane- nparray
    print('-----------------------------1. FACE RECOGNITION PROCESSING-----------------------------')

    bgsketch, tx_face, ty_face = face.face_sketch()
    #print(bgsketch[172][200])


    # test
    #test_sketch = cv2.imread('C:/Code/sketchrobot/image/circle.png')
    #test_sketch = cv2.cvtColor(test_sketch, cv2.COLOR_BGR2GRAY)
    #test_sketch = cv2.resize(test_sketch, (int(letter_size[1] * 0.6), int(letter_size[0] * 0.6)))


    #1-1. Sort for order of drawing with via points by using Euclidean value: sketch[i:row][j:col]
    print('-----------------------------2. SKETCH AREA CREATING-----------------------------')
    sketch = sort_vp.Sorting()
    new_vp_space, new_sift_space, pt_x_vp, pt_y_vp, pt_x_key, pt_y_key = sketch.sort_vp(bgsketch)

    #print('length of points: ', len(new_vp_space))

    #2. ordering - sorted x and y
    div_pt_x_vp = np.array([], dtype=int)
    div_pt_y_vp = np.array([], dtype=int)

    comp_pt_x_vp = pt_x_vp[1:]
    comp_pt_y_vp = pt_y_vp[1:]
    new_pt_x_vp = pt_x_vp[:-1]
    new_pt_y_vp = pt_y_vp[:-1]




    while not len(new_pt_x_vp) == 0:
        while not len(new_pt_x_vp) == 0:
            distance = dist(new_pt_x_vp[0], new_pt_y_vp[0], comp_pt_x_vp[0], comp_pt_y_vp[0])
            div_pt_x_vp = np.append(div_pt_x_vp, new_pt_x_vp[0])
            div_pt_y_vp = np.append(div_pt_y_vp, new_pt_y_vp[0])
            new_pt_x_vp = np.delete(new_pt_x_vp, 0)
            new_pt_y_vp = np.delete(new_pt_y_vp, 0)
            comp_pt_x_vp = np.delete(comp_pt_x_vp, 0)
            comp_pt_y_vp = np.delete(comp_pt_y_vp, 0)
            if (distance >= 10):
                if len(div_pt_x_vp) >= 7:
                    break
                elif len(div_pt_x_vp) < 7:
                    div_pt_x_vp = np.array([], dtype=int)
                    div_pt_y_vp = np.array([], dtype=int)
                    continue



        #3. inverse kinematics / input x-y plane / output q_d
        print('-----------------------------3. Inverse Kinematics for Theta1 and 2-----------------------------')

        IK = ikine.InverseKinematics()

        theta1_vp = np.zeros([len(div_pt_x_vp),1])
        theta2_vp = np.zeros([len(div_pt_x_vp),1])
        global tx, ty, init_tx, init_ty
        print(tx, ty)
        #3-1. inverse kinematics based on the pt_y and pt_x
        for i in range(len(div_pt_x_vp)):
            theta1_vp[i], theta2_vp[i] = IK.ikine(np.int(div_pt_x_vp[i]), np.int(div_pt_y_vp[i]), tx, ty)


        print('===========================theta1=========================')
        print(theta1_vp * 180 / np.pi)
        print('===========================theta2=========================')
        print(theta2_vp * 180 / np.pi)





        #4. by using q_d theta1 theta2 main drawing, slightly changed theta4 100(bg) 255(keypoints)
        print('-----------------------------4. ROBOTIC ARM DRAWING-----------------------------')

        MOTOR = motor_operator.Motor()
        tx, ty = MOTOR.motor_operate(theta1_vp, theta2_vp, e, tx, ty, div_pt_x_vp, div_pt_y_vp)

        div_pt_x_vp = np.array([], dtype=int)
        div_pt_y_vp = np.array([], dtype=int)

        print(len(pt_x_vp))



    print('-----------------------------5. ROBOTIC ARM DRAWING-----------------------------')


if __name__ == '__main__':
    e = Event()

    proc_camera = Process(target=Top_Camera, args=(e, ))
    proc_robot_arm = Process(target=Robot_Arm, args=(e, ))

    #Multiprocessing Start
    proc_robot_arm.start()
    proc_camera.start()

    #Multiprocessing Close
    proc_robot_arm.join()
    proc_camera.join()









#motor_dis = motor_disable.Motor_Disable()
#motor_dis.motor_disable()


#2. check paper location and make translation matrix for compensation





#2-1. compensate x, y of viapoints by using translation















#theta1, theta2, speed medium


#5-1. pattern generating theta3
#THETA3 0~11 generating speed high maximum


#5-3. during drawing, check the paper changes rotation and translation
#NEED to MAKE
#5-3-1. after checking, re-compensate the x-y plane data (translation and rotation matrix)
#NEED to MAKE



#5-4 during drawing, check the face every 5 minutes -> 80% of number of change of sift add more??????
#NEED to MAKE


#6. tracking red marker with KCF tracker
#NEED to MAKE



#7. Sign 'ROMAS' at the right-bottom
#DO ROMAS CONTOUR AND SAVE VIAPOINT!

#Summary: 100 -> 255 -> Sign including 3 stops during drawing




