import pandas as pd
import matplotlib.pyplot as plt

fileName = "data/sim_results_14_04_2021_14_54.csv"

tracking = pd.read_csv(fileName)
t = [x*1/(32*8) for x in range(0, len(tracking))]

plt.figure(figsize=(14, 10))
plt.plot(t, tracking["right_motor_delta"])
plt.plot(t, tracking["right_knee_angular_position"])
plt.plot(t, tracking["right_motor_torque"])
plt.ylim([-20, 8])
plt.xlabel("Time (s)")
plt.ylabel("Amplitudes/Angles")
plt.grid()
plt.legend([
    "Right motor's difference wrt set point",
    "Right motor's angular position",
    "Right motor's torque usage"
])
plt.yticks(range(-20, 8, 1))
plt.savefig(fileName.replace("data", "plots").replace("csv", "png"), format="png")
plt.show()