# coding: utf-8

from __future__ import print_function
import numpy as np
import csv
import sys
from filterpy.kalman import KalmanFilter
from sklearn.neighbors import KNeighborsRegressor

if len(sys.argv) < 4:
    print("args: train_data.csv demo_data.csv output_data.csv")
    sys.exit(1)

_, train_file, demo_file, output_file = sys.argv

beacon_ids = ['cd', 'd7', '17', '51', '43', 'b8', '2a', 'f8', '3d', '62', '3']

def read_data(filename):
    with open(filename, 'rb') as f:
        reader = csv.reader(f)
        rssis = []
        positions = []
        last_17 = -70
        last_b8 = -70
        last_non_17_b8 = -70
        last_non_17_b8_i = None
        
        def rssi_row():
            res = np.zeros(11, dtype=np.float)
            res[2], res[5] = last_17, last_b8
            if last_non_17_b8_i is not None:
                res[last_non_17_b8_i] = last_non_17_b8
            return res
        
        for [b_id, rssi, x, y] in reader:
            rssi, x, y = int(rssi), (int(x) + 390.) / 3, (int(y) + 960.) / 3
            
            if b_id == '17':
                last_17 = rssi
                rssis.append(rssi_row())
            elif b_id == 'b8':
                last_b8 = rssi
                rssis.append(rssi_row())
            elif b_id in beacon_ids:
                last_non_17_b8 = rssi
                last_non_17_b8_i = beacon_ids.index(b_id)
                rssis.append(rssi_row())
        
            positions.append(np.array([x, y], dtype=np.float))

    return (np.array(rssis, dtype=np.float), np.array(positions, dtype=np.float))

def read_data_with_timestamp(filename):
    with open(filename, 'rb') as f:
        reader = csv.reader(f)
        rssis = []
        positions = []
        timestamps = []
        last_17 = -70
        last_b8 = -70
        last_non_17_b8 = -70
        last_non_17_b8_i = None
        
        def rssi_row():
            res = np.zeros(11, dtype=np.float)
            res[2], res[5] = last_17, last_b8
            if last_non_17_b8_i is not None:
                res[last_non_17_b8_i] = last_non_17_b8
            return res
        
        for [timestamp, b_id, rssi, x, y] in reader:
            timestamp, rssi, x, y = int(timestamp) / 1000, int(rssi), (int(x) + 390.) / 3, (int(y) + 960.) / 3
            
            if b_id == '17':
                last_17 = rssi
                rssis.append(rssi_row())
            elif b_id == 'b8':
                last_b8 = rssi
                rssis.append(rssi_row())
            elif b_id in beacon_ids:
                last_non_17_b8 = rssi
                last_non_17_b8_i = beacon_ids.index(b_id)
                rssis.append(rssi_row())
                
            positions.append(np.array([x, y], dtype=np.float))
            timestamps.append(timestamp)

    return (np.array(rssis, dtype=np.float), np.array(positions, dtype=np.float), np.array(timestamps, dtype=np.int64))

(rssis, positions) = read_data(train_file)
(rssis_test, positions_test, timestamps) = read_data_with_timestamp(demo_file)

ticks = [timestamps[0]]
tick_indices = [0]
for i, time in enumerate(timestamps):
    if time >= ticks[-1] + 3:
        ticks.append(time)
        tick_indices.append(i)

pixel_origin = np.array([125, 410])
global_origin = np.array([55.94455753546212,-3.1866420060396194])

coor_change_mat = np.linalg.inv(np.array([[-2.62187e6, 422601.], [-799717., -1.41637e6]]))

def coor_change(pixel):
    return coor_change_mat.dot(pixel - pixel_origin) + global_origin

rssi_kf = KalmanFilter(2, 2)
rssi_kf.x = np.array([-70, -70])
rssi_kf.P *= 400
rssi_kf.R = np.array([[ 28.86295524,   2.29415828], [  2.29415828,  33.17958867]])
rssi_kf.Q = np.eye(2) * 4  # TODO need to test on data with walking speed
rssi_kf.H = np.eye(2)

def kalman_rssi_f(rssis):
    kalman_rssi = rssi_kf.batch_filter(rssis[:,[2, 5]])[0]
    rssis[:, 2], rssis[:, 5] = kalman_rssi[:, 0], kalman_rssi[:, 1]
    
# kalman_rssi_f(rssis)
kalman_rssi_f(rssis_test)

rssi_train, positions_train = rssis, positions

neigh = KNeighborsRegressor(n_neighbors=5)
neigh.fit(rssi_train, positions_train)

def error(actual, predicted):
    return np.sqrt(np.sum((actual - predicted) ** 2) / actual.size)

raw_pred_pos = neigh.predict(rssis_test)

print("raw prediction error ", error(raw_pred_pos, positions_test))

div = 16
pos_r = np.cov(raw_pred_pos.T - positions_test.T)
pos_r_non_17_b8 = pos_r / div
pos_r_big_x = np.diag([pos_r[0, 0] / div, pos_r[1,1]])

rssis_test_non_17_b8 = rssis_test[:]
rssis_test_non_17_b8[:, [2, 5]] = 0
has_non_17_b8 = np.sum(rssis_test_non_17_b8, axis=1) != 0.
big_x = np.logical_or(raw_pred_pos[:,0] < 220, raw_pred_pos[:,0] > 380)
r_array = [None] * len(rssis_test)

for i in range(len(rssis_test)):
    if (big_x[i]):
        r_array[i] = pos_r_big_x
        
    if (has_non_17_b8[i]):
        r_array[i] = pos_r_non_17_b8

pos_kf = KalmanFilter(2, 2)
pos_kf.x = np.array([300, 600])
pos_kf.P *= 300 ** 2
pos_kf.R = pos_r
pos_kf.Q = np.eye(2) * 64
pos_kf.H = np.eye(2)

kalman_pos, kalman_cov, _, _ = pos_kf.batch_filter(raw_pred_pos, Rs=r_array)
kalman_pos, kalman_cov, _, _ = pos_kf.rts_smoother(kalman_pos, kalman_cov)

print("error after Kalman ", error(kalman_pos, positions_test))

pos_global = np.apply_along_axis(coor_change, 1, kalman_pos)
with open(output_file, 'w') as f:
    for i in tick_indices:
        print("{},{},{}".format(timestamps[i], pos_global[i, 0], pos_global[i, 1]), file=f)
