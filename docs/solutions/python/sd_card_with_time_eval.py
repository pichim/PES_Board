# conda create --name pes-env python=3.11.4 numpy scipy matplotlib pyserial control ipykernel

import os
import numpy as np
import matplotlib.pyplot as plt


def read_sdcard_data_with_time(file_name):
    """
    1) Reads the first byte as the number of floats per record.
    2) Reads the remaining data as float32 and truncates to a multiple
        of num_of_floats.
    3) Reshapes into (num_records, num_of_floats).
    4) The first column is delta time in microseconds -> convert to
        cumulative time in seconds.
    5) The remaining columns are returned as measurements.

    Returns a dictionary with:
        {
          "time":    1D array of length num_records,
          "values":  2D array [num_records x (num_of_floats-1)]
        }
    """
    print("   --- read_sdcard_data_with_time ---")

    # Open file in binary mode
    with open(file_name, "rb") as f:
        # Read the first byte (uint8) to get num_of_floats
        num_of_floats_array = np.fromfile(f, dtype=np.uint8, count=1)
        if len(num_of_floats_array) == 0:
            raise ValueError("File is empty or not in the expected format.")
        num_of_floats = int(num_of_floats_array[0])
        print(f"   Number of floats: {num_of_floats}")

        # Read the remaining data as float32
        data_raw = np.fromfile(f, dtype=np.float32)
        print(f"   Raw data length: {len(data_raw)}")

    # Truncate to a multiple of num_of_floats
    length_adjusted = (len(data_raw) // num_of_floats) * num_of_floats
    data_raw = data_raw[:length_adjusted]

    # Reshape into (num_records, num_of_floats)
    data_values = data_raw.reshape(-1, num_of_floats)

    # We assume the first column is delta time in microseconds
    delta_time_us = data_values[:, 0]

    # Convert to cumulative time in seconds, shift to start at 0
    time = np.cumsum(delta_time_us) * 1e-6

    # The remaining columns are measurements
    data_values = data_values[:, 1:]

    print(f"   Data matrix: {data_values.shape[0]}x{data_values.shape[1]}\n")

    return {"time": time, "values": data_values}  # 1D array of length num_records, # 2D array of shape [num_records x (num_of_floats-1)]


# Read the file
file_name = os.path.join("docs", "solutions" ,"python", "002.bin")
data = read_sdcard_data_with_time(file_name)

# Evaluate the Data

# Average Sampling Time
Ts = np.mean(np.diff(data["time"]))

dtime_mus = np.diff(data["time"] * 1e6)  # in microseconds


plt.figure(1)
plt.plot(data["time"][:-1], dtime_mus)
plt.grid(True)
plt.title(f"Mean {np.mean(dtime_mus):0.0f} µs, " f"Std. {np.std(dtime_mus):0.0f} µs, " f"Med. dT = {np.median(dtime_mus):0.0f} µs")
plt.xlabel("Time (sec)")
plt.ylabel("dTime (µs)")
plt.xlim([0, data["time"][-2]])
plt.ylim([0, 1.2 * np.max(dtime_mus)])

plt.figure(2)
ax1 = plt.subplot(121)
ax1.plot(data["time"], data["values"][:, 0], label="Distance in mV")
ax1.grid(True)
ax1.set_xlabel("Time (sec)")
ax1.legend()
ax1.set_xlim([0, data["time"][-1]])
ax2 = plt.subplot(122, sharex=ax1)  # share the x-axis
ax2.plot(data["time"], data["values"][:, 1], label="Distance (cm)")
ax2.plot(data["time"], data["values"][:, 2], label="Distance Averaged (cm)")
ax2.grid(True)
ax2.set_xlabel("Time (sec)")
ax2.legend()
ax2.set_xlim([0, data["time"][-1]])

plt.tight_layout()
plt.show()
