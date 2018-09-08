import numpy as np
import cv2
import imutils

threshold_level = 150

class ContourPaper:
    def __init__(self):
        pass

    def cont_paper(self, frame_paper):
        frame = frame_paper

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 0)
        thresh = cv2.threshold(blurred, threshold_level, 255, cv2.THRESH_BINARY)[1]

        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        size = np.array([])
        count = 0

        for i in range(len(cnts)):
            size = np.append(size, len(cnts[i][:]))

        pick_index = size.argmax()
        cnts = cnts[pick_index][:]

        peri = cv2.arcLength(cnts, True)
        approx = cv2.approxPolyDP(cnts, 0.03 * peri, True)

        (x, y, w, h) = cv2.boundingRect(approx)


        ratio_w = 279.4 / w
        ratio_h = 215.9 / h

        gray = cv2.resize(gray, None, fx=ratio_w, fy=ratio_h, interpolation=cv2.INTER_AREA)

        blurred = cv2.GaussianBlur(gray, (9, 9), 0)
        thresh = cv2.threshold(blurred, threshold_level, 255, cv2.THRESH_BINARY)[1]

        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        size = np.array([])
        count = 0

        for i in range(len(cnts)):
            size = np.append(size, len(cnts[i][:]))

        pick_index = size.argmax()
        cnts = cnts[pick_index][:]


        peri = cv2.arcLength(cnts, True)
        approx = cv2.approxPolyDP(cnts, 0.04 * peri, True)

        (x, y, w, h) = cv2.boundingRect(approx)
        tx = x
        ty = y

        #print(tx, ty, tw, th)

        #cv2.drawContours(gray, [cnts], -1, (0, 255, 0), 2)
        #cv2.imshow('test', gray)
        #cv2.waitKey(0)

        return tx, ty
