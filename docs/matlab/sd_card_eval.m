clc, clear variables
%% Read the file

file_name = '001.bin';
data = read_sdcard_data(file_name);


%% Evaluate the Data

% // write data to the internal buffer of the sd card logger and send it to the sd card
% sd_logger.write((float)(cntr)); // the logger only supports float, so we need to cast the counter to float
% sd_logger.send();

figure(1)
plot(data.values), grid on
xlabel('Time (sec)')
