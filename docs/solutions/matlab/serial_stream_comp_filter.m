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
file_name = 'data_comp_filter_00.mat';
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
ind.gyro = 1;
ind.acc  = 2:3;
ind.roll = 4;

figure(2)
subplot(211)
plot(data.time, data.values(:, ind.gyro)), grid on
ylabel('Gyro (rad/sec)')
xlim([0 data.time(end)])
subplot(212)
plot(data.time, data.values(:, ind.acc)), grid on
xlabel('Time (sec)')
ylabel('Acc (m²/sec)')
xlim([0 data.time(end)])

% Roll estimates based on individual measurements
roll_gyro = cumtrapz(data.values(:, ind.gyro)) * Ts;
roll_acc = atan2(data.values(:, ind.acc(1)), ...
    data.values(:, ind.acc(2)));

% Creating Low-Pass Filter
s = tf('s');
T = 0.2;
Glp_c = 1 / (T*s + 1);
Glp = c2d(Glp_c, Ts, 'tustin');
Blp = Glp.num{1};
Alp = Glp.den{1};

% Complementary Filter
roll_comp_filter = T * filter(Blp, Alp, data.values(:, ind.gyro)) + ...
    filter(Blp, Alp, atan2(data.values(:, ind.acc(1)), data.values(:, ind.acc(2))));

% 1-D Mahony Filter
Kp = 1/T;
N = length(data.values(:, ind.gyro));
roll_1d_mahony_i = 0;
roll_1d_mahony = zeros(N, 1)
for i = 1:N
    roll_1d_mahony_i = roll_1d_mahony_i + ...
        Ts * (data.values(i, ind.gyro) + Kp * (roll_acc(i) - roll_1d_mahony_i));
    roll_1d_mahony(i) = roll_1d_mahony_i;
end

figure(3)
plot(data.time, [data.values(:, ind.roll), ...
    roll_gyro, ...
    roll_acc, ...
    roll_comp_filter, ...
    roll_1d_mahony] * 180/pi), grid on
xlabel('Time (sec)')
ylabel('Roll (deg)')
legend('Mahony', ...
    'Int. Gyro', ...
    'Acc', ...
    'Comp. Filter', ...
    '1-D Mahony Filter', ...
    'Location', 'Best')
xlim([0 data.time(end)])
