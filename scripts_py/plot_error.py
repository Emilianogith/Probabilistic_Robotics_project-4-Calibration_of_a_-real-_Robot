import pandas as pd
import matplotlib.pyplot as plt

it = pd.read_csv("../build/error_log.csv")
plt.figure()
plt.plot(it["iter"], it[" error"], marker='o')
plt.xlabel("Iteration") 
plt.ylabel("mean ||e|| (Euclidean)")
plt.title("ICP mean error over iterations")
plt.grid(True)
plt.show()
