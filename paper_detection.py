import cv2

class ShapeDetector:
    def __init__(self):
        pass

    def detect(self, c):
        #initialize the shape name and approximate the counter
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)

        # if the shape is a triangle, it will be have 3 vertices
        if len(approx) == 3:
            shape = "It is not drawing paper"

        #if the shape has 4 vertices, it is either a square or a rectangle
        elif len(approx) == 4:
            #compute the bounding box of the contour and use the bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w/float(h)

            shape = "It is not drawing paper" if ar >= 0.95 and ar <= 1.05 else "It is the drawing paper"

        elif len(approx) == 5:
            shape = "It is not drawing paper"

        else:
            shape = "It is not drawing paper"

        return shape

    #def FindPoints(self, c):




