import numpy as np
import matplotlib.pyplot as plt

def wrap(angle):
	return (angle + np.pi) % (2 * np.pi) - np.pi


def get_pos_sensor_traj(path):
	f = open(path)
	lines = f.read().splitlines()

	c = 0
	x = []
	y = []
	for l in lines:
		c += 1
		if(c < 10):
			continue
		tokens = l.split(":")
		tracker_pose = tokens[-1].strip()
		xy = tracker_pose.split(" ")
		x.append(float(xy[0]))
		y.append(float(xy[1]))
	return x,y

def get_pos_robot_traj(path):
	f = open(path)
	lines = f.read().splitlines()

	c = 0
	x = []
	y = []
	for l in lines:
		c += 1
		if(c < 10):
			continue
		tokens = l.split(":")
		tracker_pose = tokens[-2].strip()
		xy = list(filter(None, tracker_pose.split(" ")))
		x.append(float(xy[0]))
		y.append(float(xy[1]))
	return x,y

def odometry(path):
	f = open(path)
	lines = f.read().splitlines()

	c = 0
	x = []
	y = []

	x_r = 0
	y_r = 0
	theta_r = 0
	phi_r = 0
	tick_track_prev = float(lines[10].split(":")[-3].strip().split(" ")[1])
	MOD = 2**32

	#kinematic params
	K_steer = 0.1
	K_traction = 0.0106141 
	baseline = 1.4
	steer_offset = 0

	tick_track_max = 5000
	tick_steer_max = 8192

	for l in lines:
		c += 1
		if(c < 10):
			continue
		tokens = l.split(":")
		encoders_tick = tokens[-3].strip().split(" ")
		tick_steer = float(encoders_tick[0])
		tick_track_curr = float(encoders_tick[1])


		#integrate the model		
		delta = (tick_track_curr - tick_track_prev) % MOD
		if delta > MOD // 2:
			delta_ticks = delta - MOD
		else:
			delta_ticks = delta

		delta_traction = (K_traction * delta_ticks) / tick_track_max

		# integrate the model
		phi_r    = K_steer * wrap(tick_steer * (2 * np.pi / tick_steer_max)) + steer_offset 
		x_r     += delta_traction * np.cos(theta_r) * np.cos(phi_r)
		y_r     += delta_traction * np.sin(theta_r) * np.cos(phi_r) 
		theta_r  = theta_r + delta_traction * np.sin(phi_r)/baseline

		tick_track_prev = tick_track_curr

		x.append(float(x_r))
		y.append(float(y_r))

	return x,y

if __name__ == "__main__":
	path = "dataset.txt"
	x,y = get_pos_sensor_traj(path)
	x_np = np.asarray(x)
	y_np = np.asarray(y)
	# fig = plt.figure()
	# ax = fig.add_subplot(111)
	# ax.scatter(x, y, s=0.5)
	# ax.axis("equal")

	x_r,y_r = get_pos_robot_traj(path)
	x_odom,y_odom = odometry(path)
	
	fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
	ax1.set_title("Sensor trajectory in external tracking frame")
	ax1.scatter(x_np, y_np, s=0.5, label="Sensor trajectory")
	ax1.axis("equal")
	ax1.set_xlabel("x_s [m]")
	ax1.set_ylabel("y_s [m]")
	ax1.legend()

	# fig2, ax2 = plt.subplots()
	# ax2.scatter(x_r, y_r, s=0.5, label="Robot Trajectory", color="orange")
	# ax2.axis("equal")
	# ax2.legend()

	ax2.set_title("Robot odometry in robot local frame")
	ax2.scatter(x_odom, y_odom, s=0.5, label="Robot odometry (not calibrated)", color="green")
	ax2.axis("equal")
	ax2.set_xlabel("x [m]")
	ax2.set_ylabel("y [m]")
	ax2.legend()

	plt.show()