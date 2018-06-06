import cv2
import numpy as np



def translation_paper(x_old, y_old, x_new, y_new):
    tx = x_new - x_old
    ty = y_new - y_old

    return tx, ty



class Face:
    def __init__(self):
        pass

    def face_sketch(self):

        #face training
        face_cascade = cv2.CascadeClassifier('haarcascades/haarcascade_frontalface_default.xml')
        letter_size = [279, 216]
        test_face = cv2.imread('C:/Code/sketchrobot/image/sketch_intro.jpg')
        test_face = cv2.resize(test_face, (int(letter_size[0] * 0.70), int(letter_size[1] * 0.8)))
        gray_test_face = cv2.cvtColor(test_face, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray_test_face, 1.3, 5)

        roi_total = 255 * np.ones(gray_test_face.shape[::])


        for (x, y, w, h) in faces: #x: col, y: row, w: col length, h: row length
            row_face_init = int(0.55* y)
            row_face_end = int(1.1 * (y + h))

            column_face_init = int(0.55 * x)
            column_face_end = int(1.1 * (x + w))

            roi_face_gray = gray_test_face[row_face_init:row_face_end, column_face_init: column_face_end]

            row_shoulder_init = int(1 * (y+h))
            row_shoulder_end = gray_test_face.shape[0]

            column_shoulder_init = 0
            column_shoulder_end = gray_test_face.shape[1]

            roi_shoulder_gray = gray_test_face[row_shoulder_init: row_shoulder_end, column_shoulder_init: column_shoulder_end]

            tx_face, ty_face = translation_paper(int(x + (1/2) * w), int(y + (1/2)*h), int(column_shoulder_end / 2), int(row_shoulder_end / 2))

        #from the gray picture, remove up, left, right area
        gray_test_face[0: row_face_init - 5, 0:gray_test_face.shape[1]] = 255 #above head
        gray_test_face[row_face_init - 10: row_shoulder_init, 0: column_face_init - 10] = 255 #head left-side & above shoulder
        gray_test_face[row_face_init - 10: row_shoulder_init, column_face_end + 10: column_shoulder_end] = 255 #head right-side & above shoulder
        #gray_test_face[row_face_end: row_shoulder_end, column_shoulder_init: column_shoulder_end] = 255
        #gray_test_face[0: row_shoulder_end, 0: column_face_init] = 255
        #gray_test_face[0: row_shoulder_end, column_face_end: column_shoulder_end] = 255



        #cv2.imshow('roi', gray_test_face)
        #cv2.waitKey()

        cv2.imwrite('C:/Code/sketchrobot/image/croppedgray.png', gray_test_face)


        edges = cv2.Canny(gray_test_face, 10, 220)
        cv2.imwrite('C:/Code/sketchrobot/image/edges.png', edges)


        row_edge = edges.shape[0]
        col_edge = edges.shape[1]

        bgsketch = np.zeros(edges.shape[::])

        cv2.imwrite('C:/Code/sketchrobot/image/bgsketch.png', bgsketch)

        # background contour sketch
        for i in range(row_edge-3):
            for j in range(col_edge-3):
                if (edges[i, j] == 255) & (np.mod(j, 1) == 0):
                    bgsketch[i, j] = 100

        # remove bottom line point due to reduce via point
        bgsketch[row_shoulder_end - 15: row_shoulder_end, column_shoulder_init: column_shoulder_end] = 0

        # remove top line point due to reduce via point
        bgsketch[0: row_face_init - 2, 0: column_shoulder_end] = 0

        # remove left line point due to reduce via point
        bgsketch[0: row_shoulder_end, column_shoulder_init: column_shoulder_init + 10] = 0

        # remove right line point due to reduce via point
        bgsketch[0: row_shoulder_end, column_shoulder_end - 20: column_shoulder_end] = 0


        #ADD
        #remove top line point of face
        bgsketch[row_face_init - 1 : row_face_init, column_face_init - 1: column_face_end+1] = 0

        # remove left line point of face
        bgsketch[row_face_init - 1 : row_face_end + 1, column_face_init - 20: column_face_init + 5] = 0

        # remove right line point of face

        bgsketch[row_face_init - 1 : row_face_end + 1, column_face_end - 10: column_face_end + 20] = 0

        # remove top line point of shoulder
        bgsketch[row_shoulder_init - 15: row_shoulder_init + 15, column_shoulder_init: column_face_init + 1] = 0

        # remove top line point of shoulder
        bgsketch[row_shoulder_init - 15: row_shoulder_init + 15, column_face_end: column_shoulder_end] = 0

        cnt_bg_temp = 0

        for i in range(row_edge):
            for j in range(col_edge):
                if (bgsketch[i, j] == 100):
                    cnt_bg_temp += 1

        cv2.imwrite('C:/Code/sketchrobot/image/bgsketchafter.png', bgsketch)

        #shading

        sift = cv2.xfeatures2d.SIFT_create()
        kp, des = sift.detectAndCompute(gray_test_face, None)

        row_shade = np.zeros(len(kp))
        col_shade = np.zeros(len(kp))


        for i in range(len(kp)):
            col_shade[i] = int(kp[i].pt[0])
            row_shade[i] = int(kp[i].pt[1])


        for (i, j) in enumerate(kp):
            row_i = np.uint(row_shade[i])
            col_j = np.uint(col_shade[i])
            bgsketch[row_i, col_j] = 255

        # remove bottom line point due to reduce via point
        bgsketch[row_shoulder_end - 15: row_shoulder_end, column_shoulder_init: column_shoulder_end] = 0

        # remove top line point due to reduce via point
        bgsketch[0: row_face_init - 2, 0: column_shoulder_end] = 0

        # remove left line point due to reduce via point
        bgsketch[0: row_shoulder_end, column_shoulder_init: column_shoulder_init + 10] = 0

        # remove right line point due to reduce via point
        bgsketch[0: row_shoulder_end, column_shoulder_end - 20: column_shoulder_end] = 0


        #ADD
        #remove top line point of face
        bgsketch[row_face_init - 1 : row_face_init, column_face_init - 1: column_face_end+1] = 0

        # remove left line point of face
        bgsketch[row_face_init - 1 : row_face_end + 1, column_face_init - 20: column_face_init + 5] = 0

        # remove right line point of face

        bgsketch[row_face_init - 1 : row_face_end + 1, column_face_end - 10: column_face_end + 20] = 0

        # remove top line point of shoulder
        bgsketch[row_shoulder_init - 15: row_shoulder_init + 15, column_shoulder_init: column_face_init + 1] = 0

        # remove top line point of shoulder
        bgsketch[row_shoulder_init - 15: row_shoulder_init + 15, column_face_end: column_shoulder_end] = 0


        bgsketch = cv2.flip(bgsketch, 0)

        cv2.imwrite('C:/Code/sketchrobot/image/sketch_shade.png', bgsketch)

        #test
        #bgsketch = cv2.imread('C:/Code/sketchrobot/image/circle.png')
        #bgsketch = cv2.cvtColor(bgsketch, cv2.COLOR_BGR2GRAY)
        #bgsketch = cv2.resize(bgsketch, (int(letter_size[1] * 0.8), int(letter_size[0] * 0.8)))

        #print(bgsketch)
        #cv2.imshow('C:/Code/sketchrobot/image/bgsketch', bgsketch)

        #cv2.waitKey()

        return bgsketch, tx_face, ty_face


