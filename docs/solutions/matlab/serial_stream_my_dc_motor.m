clc, clear all
%%

port = '/dev/ttyUSB0'; % port = 'COM12';
baudrate = 2e6;

% Initialize the SerialStream object
try
    serialStream.reset();
    fprintf("Resetting existing serialStream object.\n")
catch exception
    serialStream = SerialStream(port, baudrate);
    fprintf("Creating new serialStream object.\n")
end

% Starting the stream
serialStream.start()
while (serialStream.isBusy())
    pause(0.1);
end

% Accessing the data
try
    data = serialStream.getData();
catch exception
    fprintf("Data Stream not triggered.\n")
    return
end

% Save the data
file_name = 'data_my_dc_motor_00.mat';
save(file_name, 'data');

% Load the data
load(file_name)


%% Evaluate time

Ts = mean(diff(data.time));

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


%% Evaluate the data

% Defining the indices for the data columns
ind.counts    = 1;
ind.velocity  = 2;
ind.rotations = 3;
ind.voltage   = 4;
ind.velocity_setpoint = 5;
ind.velocity_target   = 6;

% Calculate smoothed acceleration
acceleration = [0; diff(data.values(:, ind.velocity))/Ts]; % prepend zero to match length
acceleration_smoothed = conv(acceleration, ones(1, 20)/20, 'same');

figure(2)
subplot(211)
plot(data.time, data.values(:, ind.voltage)), grid on
ylabel('Voltage (V)')
subplot(212)
plot(data.time, acceleration_smoothed), grid on
ylabel('Acceleration (RPS/sec)')
xlabel('Time (sec)')

figure(3)
subplot(311)
plot(data.time, data.values(:, ind.counts)), grid on
ylabel('Counts')
subplot(312)
plot(data.time, [data.values(:, ind.velocity_setpoint), ...
    data.values(:, ind.velocity_target), ...
    data.values(:, ind.velocity)]), grid on
legend('Setpoint', ...
    'Target', ...
    'Actual', ...
    'Location', 'best')
ylabel('Velocity (RPS)')
subplot(313)
plot(data.time, data.values(:, ind.rotations)), grid on
ylabel('Rotations')
xlabel('Time (sec)')
