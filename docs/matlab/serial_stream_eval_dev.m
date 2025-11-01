clc, clear all
%%

port = '/dev/ttyUSB0';
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

% debug stuff
lenth_time = length(data.time);
[Ndata, Nsignals] = size(data.values);
disp(lenth_time)
disp(Ndata)
disp(Nsignals)

time_diff = diff(data.time) * 1e6;
values = data.values;
disp(time_diff(1:10).')
disp(values(1:10,:).')
disp(size(values))


%% Plotting the data

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

for i = 1:Nsignals
    figure(i+1)
    plot(data.time, data.values(:,i)), grid on
end
