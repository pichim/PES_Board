clc, clear variables
%%

% 2 khz
max_num_of_floats = 2000000 / (4 * (8 + 2) * 2000)


% openlager
file_id = fopen('LOG004.TXT');
% file_id = fopen('LOG001.TXT');
% file_id = fopen('LOG002.TXT'); % PpmIn
% file_id = fopen('LOG003.TXT'); % SBus
% file_id = fopen('LOG095.TXT'); % SBus
% file_id = fopen('LOG096.TXT'); % PpmIn

num_of_floats = fread(file_id, 1, 'uint8')

data_raw = fread(file_id, 'single');
length(data_raw)

fclose(file_id);


%%

data.values = reshape(data_raw, [num_of_floats, length(data_raw)/num_of_floats]).';
            
data.time = cumsum(data.values(:,1)) * 1e-6;
data.time = data.time - data.time(1);

data.values = data.values(:,2:end);


%%

figure(1)
plot(data.time(1:end-1), diff(data.time * 1e6)), grid on
title( sprintf(['Mean dTime = %0.2f musec, ', ...
                'Std. dTime = %0.2f musec, ', ...
                'Median dTime = %0.2f musec'], ...
                mean(diff(data.time * 1e6)), ...
                std(diff(data.time * 1e6)), ...
                median(diff(data.time * 1e6))) )
xlabel('Time (sec)'), ylabel('dTime (musec)')
ylim([0 1.2*max(diff(data.time * 1e6))])
xlim([0 data.time(end-1)])

figure(2)
plot(data.time, data.values(:,1:4)), grid on
xlabel('Time (sec)')
xlim([0 data.time(end)])
ylim([-2 3])

figure(3)
ax(1) = subplot(311);
plot(data.time, data.values(:,5)), grid on
hold on
plot(data.time, data.values(:,8))
ax(2) = subplot(312);
plot(data.time, data.values(:,6)), grid on
hold on
plot(data.time, data.values(:,9))
ax(3) = subplot(313);
plot(data.time, data.values(:,7)), grid on
hold on
plot(data.time, data.values(:,10))
xlabel('Time (sec)')
linkaxes(ax, 'x'), clear ax
xlim([0 data.time(end)])

return


%%

% SBus elrs
load data_03.mat % save data_03 data
Ts = 500 * 1e-6;

w0 = 20 * 2*pi;
D = sqrt(3)/2;
Gf = c2d(tf(w0^2, [1 2*D*w0 w0^2]), Ts, 'tustin');

data_f = filter(Gf.num{1}, Gf.den{1}, data.values);

figure(1)
plot(data.time, [data.values, data_f]), grid on
xlabel('Time (sec)')
xlim([0 data.time(end)])
ylim([-2 3])

