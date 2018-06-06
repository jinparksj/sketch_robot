import numpy as np

a1 = 86. + 72.6
a2 = 77.2 + 89.


class InverseKinematics:
    def __init__(self):
        pass

    def ikine(self, x, y, tx, ty):
        x = x + 110 - tx + 40
        y = y + 70 + ty - 95

        #print(x,',', y)
        r = np.sqrt(np.square(x) + np.square(y))



        #theta 2: Elbow up / down 45degree
        c2 = ( np.square(r) - np.square(a1) - np.square(a2)) / (2 * a1 * a2)

        #s2 = (np.square(r) - np.square(a1) - np.square(a2)) / (-2 * a1 * a2)

        #To Prevent Error of Return
        #if (c2 <= -1):
        #    c2 = -0.999999999999
        #elif (c2 >= 1):
        #    c2 = 0.9999999999999
        #else:
        #    pass

        #To Prevent Error of Return
        #if (s2 <= -1):
        #    s2 = -0.999999999999
        #elif (s2 >= 1):
        #    s2 = 0.9999999999999
        #else:
        #    pass

        theta2_up = np.arctan2(np.sqrt(1-np.square(c2)), c2)
        theta2_down = np.arctan2(-np.sqrt(1-np.square(c2)), c2)

        #print(x, y, c2)

        #theta2_1 = np.arctan2(s2, -np.sqrt(1 - np.square(s2)))
        #theta2_2 = np.arctan2(s2, np.sqrt(1 - np.square(s2)))

        beta = np.arctan2(y, x)
        calpha = (np.square(a1) + np.square(r) - np.square(a2)) / (2 * a1 * r)
        salpha = np.arctan2(np.sqrt(1-np.square(calpha)), calpha)
        alpha = np.arctan2(salpha, calpha)

        theta1 = beta + alpha

        #theta 1
        #theta1_down = np.arctan2(y, x) + np.arctan2((a2 * np.sin(theta2_down) / r), ((a1 + a2 * np.cos(theta2_down)) / r))
        #theta1_up = np.arctan2(y, x) + np.arctan2((a2 * np.sin(theta2_up) / r), ((a1 + a2 * np.cos(theta2_up)) / r))


        #if (theta1_down - np.pi/2) >= (np.pi/4):
        #    theta1 = theta1_down
        #    theta2 = theta2_down
        #elif (theta1_down - np.pi/2) < (np.pi/4):
        #    theta1 = theta1_up
        #    theta2 = theta2_up

        theta2 = theta2_down


        return theta1, theta2


