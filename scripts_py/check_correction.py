import matplotlib.pyplot as plt
import numpy as np
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

    delta_traction = K_traction * float(delta_ticks)

    # integrate the model
    phi    = K_steer * float(ticks_steer) + steer_offset
    theta  = theta + delta_traction * np.sin(phi)/baseline
    x     += delta_traction * np.cos(theta) * np.cos(phi)
    y     += delta_traction * np.sin(theta) * np.cos(phi) 

    pos_robot = np.array([x, y])

    R = np.array([[np.cos(theta), -np.sin(theta)],
                [np.sin(theta), np.cos(theta)]])


    h.append(pos_robot + R @ pos_sensor)

    tick_track_prev = tick_track_curr


H = np.array(h)   
plt.figure()
plt.scatter(H[:, 0], H[:, 1], s=0.5)
plt.axis("equal")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Predicted sensor trajectory")
plt.show()
