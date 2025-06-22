# conda create --name pes-env python=3.11.4 numpy scipy matplotlib pyserial control ipykernel

import os
import numpy as np
import matplotlib.pyplot as plt


def read_sdcard_data(file_name):
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
          "values":  2D array [num_records x (num_of_floats-1)]
        }
    """
    print("   --- read_sdcard_data ---")

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
    print(f"   Data matrix: {data_values.shape[0]}x{data_values.shape[1]}\n")

    return {"values": data_values}


# Read the file
file_name = os.path.join("docs", "solutions", "python", "001.bin")
data = read_sdcard_data(file_name)

# Evaluate the Data

# // write data to the internal buffer of the sd card logger and send it to the sd card
# sd_logger.write(float(cntr)); // the logger only supports float, so we need to cast the counter to float
# sd_logger.send();
plt.figure()
plt.plot(data["values"])
plt.grid(True)
plt.xlabel("Time (sec)")

plt.tight_layout()
plt.show()
