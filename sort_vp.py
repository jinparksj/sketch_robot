import numpy as np
import cv2
from scipy.spatial.distance import cdist

def nearest_point(pt, group):
    distances = cdist(pt, group)
    nearest_pt = group[distances.argmin()]
    for i, (row, col) in enumerate(group):
        if (row == nearest_pt[0]) & (col == nearest_pt[1]):
            nearest_index = i
    return nearest_pt, nearest_index

def vp_space_sorting(group, new_group, nearest_pt, nearest_index):
    new_group = np.append(new_group, nearest_pt)
    new_group = np.reshape(new_group, (-1, 2))
    group = np.delete(group, nearest_index, 0)
    return group, new_group


class Sorting:
    def __init__(self):
        pass

    def sort_vp(self, sketch):

        row_sketch = sketch.shape[0]
        col_sketch = sketch.shape[1]

        #1. Create the space for via points (returned)
        cnt_vp = 0

        for i in range(row_sketch):
            for j in range(col_sketch):
                if (sketch[i, j] == 100): #for test 255, real 100
                    cnt_vp += 1

        vpoint = np.zeros([cnt_vp, 2])
        vpoint_x = np.zeros([cnt_vp, 1])
        vpoint_y = np.zeros([cnt_vp, 1])


        #2. Extract X and Y Location about Via Points
        cnt_temp = 0

        for i in range(row_sketch):
            for j in range(col_sketch):
                if (sketch[i, j] == 100): #for test 255, real 100
                    vpoint[cnt_temp] = [i, j]
                    vpoint_y[cnt_temp] = i
                    vpoint_x[cnt_temp] = j
                    cnt_temp += 1



        #3. By using X-Y Via points, Sorting by Euclidean
        new_vp_space = np.array([[]], dtype=int)
        ref_point = vpoint[0]
        ref_point = np.expand_dims(ref_point, axis=0)
        new_vp_space = np.append(new_vp_space, ref_point)
        for i in range(len(vpoint)):
            nearest_pt, nearest_index = nearest_point(ref_point, vpoint)
            vpoint, new_vp_space = vp_space_sorting(vpoint, new_vp_space, nearest_pt, nearest_index)
            ref_point = np.expand_dims(new_vp_space[i+1], axis=0)


        #4. Create the space for sift key points (returned)
        cnt_sift = 0

        for i in range(row_sketch):
            for j in range(col_sketch):
                if (sketch[i, j] == 255):
                    cnt_sift += 1

        kpoint = np.zeros([cnt_sift, 2])
        kpoint_x = np.zeros([cnt_sift, 1])
        kpoint_y = np.zeros([cnt_sift, 1])


        #5. Extract X and Y Location about SIFT Points
        cnt_temp_sift = 0

        for i in range(row_sketch):
            for j in range(col_sketch):
                if (sketch[i, j] == 255):
                    kpoint[cnt_temp_sift] = [i, j]
                    kpoint_y[cnt_temp_sift] = i
                    kpoint_x[cnt_temp_sift] = j
                    cnt_temp_sift += 1

        #6. By using X-Y SIFT key points, Sorting by Euclidean
        new_key_space = np.array([[]], dtype=int)
        ref_key_point = kpoint[0]
        ref_key_point = np.expand_dims(ref_key_point, axis=0)
        new_key_space = np.append(new_key_space, ref_key_point)
        for i in range(len(kpoint)):
            nearest_key_pt, nearest_key_index = nearest_point(ref_key_point, kpoint)
            kpoint, new_key_space = vp_space_sorting(kpoint, new_key_space, nearest_key_pt, nearest_key_index)
            ref_key_point = np.expand_dims(new_key_space[i + 1], axis=0)

        pt_x_vp = new_vp_space[:,1]
        pt_y_vp = new_vp_space[:,0]

        #calibration for row, y position
        #for i in range(len(pt_x_vp)):
        #    pt_y_vp[i] = int(pt_y_vp[i] + (pt_x_vp[i] / max(pt_x_vp)) * (5/11))
            #if pt_y_vp[i] < 0:
            #    pt_y_vp[i] = 0

        #calibration for row, y length
        #for i in range(len(pt_x_vp)):
        #
        #     pt_y_vp[i] = int(pt_y_vp[i] * (max(pt_x_vp) - pt_x_vp[i]) * 7.4 / (max(pt_x_vp) * 9.8))

        pt_x_key = new_key_space[:,1]
        pt_y_key = new_vp_space[:,0]

        return new_vp_space, new_key_space, pt_x_vp, pt_y_vp, pt_x_key, pt_y_key


