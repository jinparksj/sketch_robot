import numpy as np

class RotationAngle:
    def __init__(self):
        pass

    def rotation_angle(self, start_x, start_y, end_old_x, end_old_y, end_new_x, end_new_y):
        dist1 = np.sqrt(np.square(end_old_x - start_x) + np.square(end_old_y - start_y))
        dist2 = np.sqrt(np.square(end_new_x - start_x) + np.square(end_new_y - start_y))

        if dist1 < dist2:
            Bx = end_old_x
            By = end_old_y
            Ax = end_new_x
            Ay = end_new_y
        else:
            Ax = end_old_x
            Ay = end_old_y
            Bx = end_new_x
            By = end_new_y

        Qx = start_x - Ax
        Qy = start_y - Ay
        Px = Bx - Ax
        Py = By - Ay

        angle = np.arccos( (Px * Qx + Py * Qy) / (np.sqrt(np.square(Px) + np.square(Py)) * np.sqrt(np.square(Qx) + np.square(Qy))))

        angle = angle * 180 / np.pi

        return angle
