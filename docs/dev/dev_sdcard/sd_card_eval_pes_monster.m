clc, clear all
%%

file_id = fopen('011.bin');

% file_id = fopen('023.bin'); % dc motor thread 2 kHz, pwm 20 kHz
% file_id = fopen('027.bin'); % dc motor thread 1 kHz, pwm 20 kHz
% file_id = fopen('032.bin'); % dc motor thread 1 kHz, pwm 1 kHz

num_of_floats = fread(file_id, 1, 'uint8')

data_raw = fread(file_id, 'single');
length(data_raw)

fclose(file_id);


%%

data_raw = data_raw(1:floor( length(data_raw)/num_of_floats ) * num_of_floats);

data.values = reshape(data_raw, [num_of_floats, length(data_raw)/num_of_floats]).';
            
data.time = cumsum(data.values(:,1)) * 1e-6;
data.time = data.time - data.time(1);

data.values = data.values(:,2:end);

size(data.values)


%%

Ts = mean(diff(data.time))

figure(1)
plot(data.time(1:end-1), diff(data.time * 1e6)), grid on
title( sprintf(['Mean %0.0f mus, ', ...
                'Std. %0.0f mus, ', ...
                'Med. dT = %0.0f mus'], ...
                mean(diff(data.time * 1e6)), ...
                std(diff(data.time * 1e6)), ...
                median(diff(data.time * 1e6))) )
xlabel('Time (sec)'), ylabel('dTime (mus)')
xlim([0 data.time(end-1)])
ylim([0 1.2*max(diff(data.time * 1e6))])

% dist: ir, us
figure(2)
plot(data.time, data.values(:,1:2)), grid on

% imu: rpy
figure(3)
plot(data.time, data.values(:,3:5)), grid on

% motor: rot. target, rot.
figure(4)
subplot(311)
plot(data.time, data.values(:,6:7)), grid on
subplot(312)
plot(data.time, data.values(:,8:9)), grid on
subplot(313)
plot(data.time, data.values(:,10:11)), grid on
