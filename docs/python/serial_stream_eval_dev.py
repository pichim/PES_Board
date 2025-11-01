# conda create --name rcrc-env python=3.11.4 numpy scipy matplotlib pyserial control ipykernel (optional: slycot pytest)

import time
import numpy as np
import matplotlib.pyplot as plt

from SerialStream import SerialStream

port = "/dev/ttyUSB0"
baudrate = int(2e6)

# Initialize the SerialStream object
try:
    serialStream.reset()
    print("Resetting existing serialStream object.")
except Exception as e:
    serialStream = SerialStream(port, baudrate)
    print("Creating new serialStream object.")

# Starting the stream
serialStream.start()
while serialStream.is_busy_flag():
    time.sleep(0.1)

# Accessing the data
try:
    data = serialStream.get_data()
except Exception as e:
    print("Data Stream not triggered.")
    exit()

# debug stuff
length_time = len(data["time"])
Ndata, Nsignals = data["values"].shape
print(length_time)
print(Ndata)
print(Nsignals)

time_diff = np.diff(data["time"]) * 1e6
values = data["values"]
print(time_diff[0:10])
print(values[0:10, :].T)
print(values.shape)

# --- Plotting the data

Ts = np.mean(np.diff(data["time"]))

plt.figure(1)
plt.plot(data["time"][:-1], np.diff(data["time"]) * 1e6)
plt.grid(True)
plt.title(f"Mean {np.mean(np.diff(data['time']) * 1e6):.0f} mus, " f"Std. {np.std(np.diff(data['time']) * 1e6):.0f} mus, " f"Med. dT = {np.median(np.diff(data['time']) * 1e6):.0f} mus")
plt.xlabel("Time (sec)")
plt.ylabel("dTime (mus)")
plt.xlim([0, data["time"][-2]])
plt.ylim([0, 1.2 * np.max(np.diff(data["time"]) * 1e6)])

for i in range(Nsignals):
    plt.figure(i + 2)
    plt.plot(data["time"], data["values"][:, i])
    plt.grid(True)

plt.show()
