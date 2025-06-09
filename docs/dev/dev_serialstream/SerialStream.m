classdef SerialStream < handle
%%
    properties (Access = private)
        port
        baudrate
        SerialPort
        data
        Timer
        is_waiting_for_fist_measurement
        ind_end
        timeout
        num_of_floats
        is_busy
        max_trigger_attempts
        trigger_attempts
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
            obj.data = 0 * obj.data;
            obj.timeout = 3.0;
            obj.is_waiting_for_fist_measurement = true;
            obj.ind_end = 0;
            obj.num_of_floats = 0;
            obj.is_busy = true;
            obj.max_trigger_attempts = 5;
            obj.trigger_attempts = 0;
        end

        function start(obj)

            obj.sendStartByte();

            while true

                bytes_readable = obj.SerialPort.NumBytesAvailable();

                if (obj.is_waiting_for_fist_measurement && (bytes_readable > 0))

                    obj.is_waiting_for_fist_measurement = false;
                    obj.num_of_floats = obj.SerialPort.read(1, 'uint8');

                    bytes_readable = bytes_readable - 1;

                    fprintf("SerialStream started, logging %d signals\n", obj.num_of_floats);
                    obj.timeout = 0.3;
                end

                if (bytes_readable >= 4)

                    num_of_floats_readable = ceil(bytes_readable / 4);
                    data_raw = obj.SerialPort.read(num_of_floats_readable, 'single');

                    ind_start = obj.ind_end + 1;
                    obj.ind_end = ind_start + num_of_floats_readable - 1;
                    obj.data(ind_start:obj.ind_end) = double(data_raw(1:obj.ind_end-ind_start+1));

                    obj.Timer = tic;
                end

                if (toc(obj.Timer) > obj.timeout)

                    if (obj.is_waiting_for_fist_measurement && (obj.trigger_attempts < obj.max_trigger_attempts))
                        obj.sendStartByte();
                    else

                        if (obj.is_waiting_for_fist_measurement)
                            fprintf("SerialStream timeout, logging not triggered after %d attempts of waiting %0.2f seconds\n", ...
                                obj.max_trigger_attempts, obj.timeout);
                        else
                            fprintf("SerialStream ended with %0.2f seconds timeout\n", obj.timeout);
                            fprintf("             measured %d datapoints\n", obj.ind_end);
                        end
                        obj.is_busy = false;
                        break;
                    end
                end
            end
        end

        function is_busy = isBusy(obj)
            is_busy = obj.is_busy;
        end

        function data = getData(obj)

            data.values = reshape(obj.data(1:obj.ind_end), [obj.num_of_floats, obj.ind_end/obj.num_of_floats]).';
            
            data.time = cumsum(data.values(:,1)) * 1e-6;
            data.time = data.time - data.time(1);

            data.values = data.values(:,2:end);

        end
    end

%%
    methods (Access = private)

        function sendStartByte(obj, start_byte)

            if (~exist('byte', 'var') || isempty(start_byte))
                start_byte = 255;
            end

            obj.SerialPort.flush();
            obj.Timer = tic;
            obj.SerialPort.write(start_byte, 'uint8');
            obj.trigger_attempts = obj.trigger_attempts + 1;
            fprintf("SerialStream waiting for %0.2f seconds...\n", obj.timeout);
        end
    end
end
