import numpy as np
import matplotlib.pyplot as plt


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

if __name__ == "__main__":
	path = "dataset.txt"
	x,y = get_pos_sensor_traj(path)
	x_np = np.asarray(x)
	y_np = np.asarray(y)
	fig = plt.figure()
	ax = fig.add_subplot(111)
	ax.scatter(x, y, s=0.5)
	ax.axis("equal")
	plt.show()
