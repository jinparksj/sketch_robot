import imutils

class RotationPaper:
    def __init__(self):
        pass

    # height = img.shape[0] , width = img.shape[1]
    def rotate_paper(self, img):
        if img.shape[0] > img.shape[1]:
            rotatedimg = imutils.rotate_bound(img, 270)

        else:
            pass

        return rotatedimg

