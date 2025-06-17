classdef SerialStream < handle
%%
    properties (Access = private)
        port
        baudrate
        SerialPort
        data
        timeout
        is_waiting_for_first_measurement
        ind_end
        num_of_floats
        is_busy
        max_trigger_attempts
        trigger_attempts
        Timer
        LoggingTimer
        last_logging_print_time
    end
%%
    methods

        function obj = SerialStream(port, baudrate)
            obj.port = port;
            obj.baudrate = baudrate;
            obj.SerialPort = serialport(obj.port, obj.baudrate);
            obj.data = zeros(1e8, 1); % preallocate big data array
            reset(obj);
        end

        function reset(obj)
            obj.data = zeros(size(obj.data));
            obj.timeout = 3.0;
            obj.is_waiting_for_first_measurement = true;
            obj.ind_end = 0;
            obj.num_of_floats = 0;
            obj.is_busy = true;
            obj.max_trigger_attempts = 5;
            obj.trigger_attempts = 0;
            obj.Timer = tic;
            obj.LoggingTimer = tic;
            obj.last_logging_print_time = 0;
        end

        function start(obj)
            obj.sendStartByte();

            while true
                bytes_readable = obj.SerialPort.NumBytesAvailable();

                % First measurement: receive num_of_floats
                if (obj.is_waiting_for_first_measurement && (bytes_readable > 0))

                    obj.is_waiting_for_first_measurement = false;
                    obj.num_of_floats = obj.SerialPort.read(1, 'uint8');

                    bytes_readable = bytes_readable - 1;

                    fprintf("SerialStream started, logging %d signals\n", obj.num_of_floats);
                    obj.timeout = 0.3;

                    obj.LoggingTimer = tic;

                    % TODO: After receiving num_of_floats, the serial buffer may contain
                    % misaligned floats if the sender was running continuously.
                    % To make the stream robust against this, you should align the stream here.
                    %
                    % --- FULL ALIGNMENT CODE TEMPLATE ---
                    %
                    % bytes_left = obj.SerialPort.NumBytesAvailable();
                    % misalignment = mod(bytes_left, obj.num_of_floats * 4);
                    % if misalignment > 0
                    %     warning('SerialStream: Discarding %d bytes to align stream', misalignment);
                    %     obj.SerialPort.read(misalignment, 'uint8');
                    % end
                    %
                    % % Optionally: Read 1 full record immediately to force alignment and
                    % % guarantee correct first time step.
                    % if obj.SerialPort.NumBytesAvailable() >= obj.num_of_floats * 4
                    %     data_raw = obj.SerialPort.read(obj.num_of_floats, 'single');
                    %     ind_start = obj.ind_end + 1;
                    %     obj.ind_end = ind_start + obj.num_of_floats - 1;
                    %     obj.data(ind_start:obj.ind_end) = double(data_raw);
                    %     obj.Timer = tic;
                    % end
                    %
                    % --- END OF ALIGNMENT CODE TEMPLATE ---
                    %
                    % This prevents reshape errors and ensures that all data is read in complete records.
                    % (See matching logic used in your SD card reader and Python SerialStream class.)
                    %
                    % CURRENT STATE: Alignment not implemented yet -> possible risk of corrupted data if stream is misaligned.
                end

                % Normal streaming: read data
                if (bytes_readable >= 4)
                    num_of_floats_readable = floor(bytes_readable / 4);
                    if (num_of_floats_readable > 0)
                        data_raw = obj.SerialPort.read(num_of_floats_readable, 'single');

                        ind_start = obj.ind_end + 1;
                        obj.ind_end = ind_start + num_of_floats_readable - 1;
                        obj.data(ind_start:obj.ind_end) = double(data_raw);

                        obj.Timer = tic;
                    end
                end

                % Logging print (every 2 sec)
                if (~obj.is_waiting_for_first_measurement)
                    logging_time = toc(obj.LoggingTimer);
                    while (logging_time >= obj.last_logging_print_time + 2.0)
                        fprintf("             logging for %0.2f seconds...\n", round(obj.last_logging_print_time + 2.0));
                        obj.last_logging_print_time = obj.last_logging_print_time + 2.0;
                    end
                end

                % Timeout check:
                if (toc(obj.Timer) > obj.timeout)

                    if (obj.is_waiting_for_first_measurement && (obj.trigger_attempts < obj.max_trigger_attempts))
                        obj.sendStartByte();
                    else
                        if (obj.is_waiting_for_first_measurement)
                            fprintf("SerialStream timeout, logging not triggered after %d attempts of waiting %0.2f seconds\n", ...
                                obj.max_trigger_attempts, obj.timeout);
                        else
                            fprintf("SerialStream ended with %0.2f seconds timeout\n", obj.timeout);
                            fprintf("             logged for %0.2f seconds\n", round(logging_time));
                            fprintf("             measured %d datapoints\n", obj.ind_end);
                        end
                        obj.is_busy = false;
                        break;
                    end
                end

                % Avoid burning CPU:
                if obj.is_waiting_for_first_measurement
                    pause(0.01);  % waiting for trigger -> 10 ms
                else
                    pause(0.001); % streaming -> 1 ms
                end
            end
        end

        function is_busy = isBusy(obj)
            is_busy = obj.is_busy;
        end

        function data = getData(obj)
            % Only use complete rows:
            valid_length = floor(obj.ind_end / obj.num_of_floats) * obj.num_of_floats;

            data.values = reshape(obj.data(1:valid_length), [obj.num_of_floats, valid_length/obj.num_of_floats]).';

            % Extract time (assumes first column is delta time):
            data.time = cumsum(data.values(:,1)) * 1e-6;
            data.time = data.time - data.time(1);

            % Remove delta time column:
            data.values = data.values(:,2:end);
        end
    end

%%
    methods (Access = private)

        function sendStartByte(obj, start_byte)
            if (~exist('start_byte', 'var') || isempty(start_byte))
                start_byte = 255;
            end
            % Flush serial port to ensure no old data is left
            obj.SerialPort.flush();
            obj.Timer = tic;
            obj.SerialPort.write(start_byte, 'uint8');
            obj.trigger_attempts = obj.trigger_attempts + 1;
            fprintf("SerialStream waiting for %0.2f seconds...\n", obj.timeout);
        end
    end
end
