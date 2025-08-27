import pandas as pd
import matplotlib.pyplot as plt

it = pd.read_csv("../build/error_log.csv")


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
ax1.plot(it["iter"], it[" error"], marker='o')
ax1.set_xlabel("Iteration")
ax1.set_ylabel("Omega norm ||e|| error")
ax1.set_title("ICP error over iterations")
ax1.grid(True)

ax2.plot(it["iter"], it[" outliers"], marker='o')
ax2.set_xlabel("Iteration")
ax2.set_ylabel("Outliers")
ax2.set_title("Outliers over iterations")
ax2.grid(True)

plt.tight_layout()
plt.show()