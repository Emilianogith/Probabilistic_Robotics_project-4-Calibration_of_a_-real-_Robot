import matplotlib.pyplot as plt
import numpy as np
import sys, os
sys.path.append(os.path.abspath("../04-Calibration"))
from view_traj import get_pos_sensor_traj

params = {}
with open("../build/calibrated_params.csv") as f:
    for line in f:
        if ":" in line:
            key, val = line.split(":", 1)
            params[key.strip()] = float(val.strip())

x_s         = params["x_s"]
y_s         = params["y_s"]
K_steer     = params["K_steer"]
K_traction  = params["K_traction"]
baseline    = params["baseline"]
steer_offset= params["steer_offset"]

print(x_s, y_s, K_steer, K_traction, baseline, steer_offset)

ticks_list = []
with open("../04-Calibration/dataset.txt") as f:
    for line in f:
        parts = line.strip().split()
        if "ticks:" in parts:
            idx = parts.index("ticks:")  
            steer_ticks = int(parts[idx + 1])
            traction_ticks = int(parts[idx + 2])
            ticks_list.append((steer_ticks, traction_ticks))

# start pos
x = 0.0
y = 0.0
theta = 0.0
phi = 0.0
pos_sensor = np.array([x_s, y_s])
tick_track_prev = ticks_list[0][1]
MOD = 2**32

h = []
for ticks_steer, tick_track_curr in ticks_list:
    delta = (tick_track_curr - tick_track_prev) % MOD
    if delta > MOD // 2:
        delta_ticks = delta - MOD
    else:
        delta_ticks = delta

    delta_traction = K_traction * delta_ticks

    # integrate the model
    x     += delta_traction * np.cos(theta) * np.cos(phi)
    y     += delta_traction * np.sin(theta) * np.cos(phi) 
    theta  = theta + delta_traction * np.sin(phi)/baseline
    phi    = K_steer * ticks_steer + steer_offset

    pos_robot = np.array([x, y])

    R = np.array([[np.cos(theta), -np.sin(theta)],
                [np.sin(theta), np.cos(theta)]])


    h.append(pos_robot + R @ pos_sensor)

    tick_track_prev = tick_track_curr


path = "../04-Calibration/dataset.txt"
x_s , y_s = get_pos_sensor_traj(path)

H = np.array(h)   
plt.figure()
plt.scatter(H[:, 0], H[:, 1], s=0.5, label = "Predicted sensor trajectory")
plt.xlabel("x_s [m]")
plt.ylabel("y_s [m]")
plt.scatter(x_s, y_s, s=0.5, label = "tracker sensor trajectory")
plt.legend()
plt.show()
