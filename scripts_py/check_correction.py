import matplotlib.pyplot as plt
import numpy as np
import sys, os
sys.path.append(os.path.abspath("../04-Calibration"))
from view_traj import get_pos_sensor_traj

def wrap(angle):
	return (angle + np.pi) % (2 * np.pi) - np.pi

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
tick_track_max = 5000
tick_steer_max = 8192

h = []
p_r = []
for ticks_steer, tick_track_curr in ticks_list:
    delta = (tick_track_curr - tick_track_prev) % MOD
    if delta > MOD // 2:
        delta_ticks = delta - MOD
    else:
        delta_ticks = delta

    delta_traction = (K_traction * delta_ticks) / tick_track_max

    # integrate the model
    phi    = K_steer * wrap(ticks_steer * (2 * np.pi / tick_steer_max)) + steer_offset
    x     += delta_traction * np.cos(theta) * np.cos(phi)
    y     += delta_traction * np.sin(theta) * np.cos(phi) 
    theta  = theta + delta_traction * np.sin(phi)/baseline

    pos_robot = np.array([x, y])

    R = np.array([[np.cos(theta), -np.sin(theta)],
                [np.sin(theta), np.cos(theta)]])


    h.append(pos_robot + R @ pos_sensor)
    p_r.append(np.array([x,y]))

    tick_track_prev = tick_track_curr


path = "../04-Calibration/dataset.txt"
x_s , y_s = get_pos_sensor_traj(path)

H = np.array(h)  
X_r = np.array(p_r) 

shift = -1.5 * np.ones_like(H[:,0], dtype=float)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
ax1.scatter(H[:, 0] + shift, H[:, 1], s=0.5, label = "Predicted sensor trajectory")
ax1.set_title("Grand truth vs Calibrated odometry ")
ax1.set_xlabel("x [m]")
ax1.set_ylabel("y [m]")
ax1.scatter(x_s, y_s, s=0.5, label = "Actual sensor trajectory")
ax1.legend()

ax2.set_title("Robot and sensor trajectory in robot local frame")
ax2.scatter(H[:, 0] , H[:, 1], s=0.5, label = "Predicted sensor trajectory", color="tab:green")
ax2.scatter(X_r[:, 0], X_r[:, 1], s=0.5, label = "Robot odometry", color="tab:red")
ax2.set_xlabel("x [m]")
ax2.set_ylabel("y [m]")
ax2.legend()
plt.show()
