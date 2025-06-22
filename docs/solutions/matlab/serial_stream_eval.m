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
file_name = 'data_00.mat';
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
ind.cntr_1 = 1;
ind.cntr_2 = 2;

figure(2)
plot(data.time, data.values(:, ind.cntr_1)), grid on, hold on
plot(data.time, data.values(:, ind.cntr_2)), hold off
xlabel('Time (sec)')
ylabel('Count')
legend('Counter 1', ...
    'Counter 2', ...
    'Location', 'Best')