import serial
import time
import math
import numpy as np


class SerialStream:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.SerialPort = serial.Serial(self.port, self.baudrate)
        self.data = np.zeros(int(1e8))  # preallocate big data array
        self.reset()

    def reset(self):
        self.data = np.zeros_like(self.data)
        self.timeout = 3.0
        self.is_waiting_for_first_measurement = True
        self.ind_end = 0
        self.num_of_floats = 0
        self.is_busy = True
        self.max_trigger_attempts = 5
        self.trigger_attempts = 0
        self.Timer = time.time()
        self.LoggingTimer = time.time()
        self.last_logging_print_time = 0

    def start(self):
        self.send_start_byte()

        while True:
            bytes_readable = self.SerialPort.in_waiting

            # First measurement: receive num_of_floats
            if self.is_waiting_for_first_measurement and bytes_readable > 0:

                self.is_waiting_for_first_measurement = False
                self.num_of_floats = int.from_bytes(self.SerialPort.read(1), "little")

                bytes_readable -= 1

                print(f"SerialStream started, logging {self.num_of_floats} signals")
                self.timeout = 0.3

                self.LoggingTimer = time.time()

                # TODO: After receiving num_of_floats, the serial buffer may contain
                # misaligned floats if the sender was running continuously.
                # To make the stream robust against this, you should align the stream here.
                #
                # --- FULL ALIGNMENT CODE TEMPLATE ---
                #
                # bytes_left = self.SerialPort.in_waiting
                # misalignment = bytes_left % (self.num_of_floats * 4)
                # if misalignment > 0:
                #     print(f"WARNING: SerialStream: Discarding {misalignment} bytes to align stream")
                #     self.SerialPort.read(misalignment)
                #
                # # Optionally: Read 1 full record immediately to force alignment and
                # # guarantee correct first time step.
                # if self.SerialPort.in_waiting >= self.num_of_floats * 4:
                #     data_raw = self.SerialPort.read(self.num_of_floats * 4)
                #     data_floats = np.frombuffer(data_raw, dtype=np.float32)
                #     ind_start = self.ind_end
                #     self.ind_end = ind_start + self.num_of_floats
                #     self.data[ind_start:self.ind_end] = data_floats
                #     self.Timer = time.time()
                #
                # --- END OF ALIGNMENT CODE TEMPLATE ---
                #
                # This prevents reshape errors and ensures that all data is read in complete records.
                # (See matching logic used in your SD card reader and Python SerialStream class.)
                #
                # CURRENT STATE: Alignment not implemented yet -> possible risk of corrupted data if stream is misaligned.

            # Normal operation: read data
            if bytes_readable >= 4:
                num_of_floats_readable = math.floor(bytes_readable / 4)
                if num_of_floats_readable > 0:
                    data_raw = self.SerialPort.read(int(num_of_floats_readable * 4))
                    data_floats = np.frombuffer(data_raw, dtype=np.float32)

                    ind_start = self.ind_end + 1  # Match MATLAB 1-based indexing logic
                    self.ind_end = ind_start + num_of_floats_readable - 1
                    self.data[ind_start - 1 : self.ind_end] = data_floats[:num_of_floats_readable]

                    self.Timer = time.time()

            # Logging print (every 2 sec)
            if not self.is_waiting_for_first_measurement:
                logging_time = time.time() - self.LoggingTimer
                while logging_time >= self.last_logging_print_time + 2.0:
                    print(f"             logging for {round(self.last_logging_print_time + 2.0):.2f} seconds...")
                    self.last_logging_print_time += 2.0

            # Timeout check:
            if time.time() - self.Timer > self.timeout:
                if self.is_waiting_for_first_measurement and self.trigger_attempts < self.max_trigger_attempts:
                    self.send_start_byte()
                else:
                    if self.is_waiting_for_first_measurement:
                        print(f"SerialStream timeout, logging not triggered after {self.max_trigger_attempts} attempts of waiting {self.timeout:.2f} seconds")
                    else:
                        print(f"SerialStream ended with {self.timeout:.2f} seconds timeout")
                        print(f"             logged for {round(logging_time):.2f} seconds")
                        print(f"             measured {self.ind_end} datapoints")
                    self.is_busy = False
                    break

            # Avoid burning CPU:
            if self.is_waiting_for_first_measurement:
                time.sleep(0.01)  # waiting for trigger -> 10 ms
            else:
                time.sleep(0.001)  # streaming -> 1 ms

    def is_busy_flag(self):
        return self.is_busy

    def get_data(self):
        # Only use complete rows:
        valid_length = (self.ind_end // self.num_of_floats) * self.num_of_floats
        values = self.data[0:valid_length].reshape((-1, self.num_of_floats))

        # Extract time (assumes first column is delta time):
        time_array = np.cumsum(values[:, 0]) * 1e-6
        time_array = time_array - time_array[0]

        # Remove delta time column:
        return {"time": time_array, "values": values[:, 1:]}

    def send_start_byte(self, start_byte=255):
        # Flush serial port to ensure no old data is left
        self.SerialPort.reset_input_buffer()
        self.Timer = time.time()
        self.SerialPort.write(bytes([start_byte]))
        self.trigger_attempts += 1
        print(f"SerialStream waiting for {self.timeout:.2f} seconds...")
