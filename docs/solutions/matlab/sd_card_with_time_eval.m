clc, clear variables
%% Read the file

file_name = '002.bin';
data = read_sdcard_data_with_time(file_name);


%% Evaluate the Data

% Average Sampling Time
Ts = mean(diff(data.time));

dtime_mus = diff(data.time * 1e6);

figure(1)
plot(data.time(1:end-1), dtime_mus), grid on
title( sprintf(['Mean %0.0f mus, ', ...
                'Std. %0.0f mus, ', ...
                'Med. dT = %0.0f mus'], ...
                mean(dtime_mus), ...
                std(dtime_mus), ...
                median(dtime_mus)) )
xlabel('Time (sec)'), ylabel('dTime (mus)')
xlim([0 data.time(end-1)])
ylim([0 1.2*max(dtime_mus)])

% // write data to the internal buffer of the sd card logger and send it to the sd card
% sd_logger.write( dtime_us );        //  0 delta time in us
% sd_logger.write( ir_distance_mV );  //  1 ir distance in mV
% sd_logger.write( ir_distance_cm );  //  2 ir distance in cm
% sd_logger.write( ir_distance_avg ); //  3 ir distance averaged in cm
% sd_logger.send();

figure(2)
ax(1) = subplot(121);
plot(data.time, data.values(:,1)), grid on
xlabel('Time (sec)')
legend('Distance in mV')
ax(2) = subplot(122);
plot(data.time, [data.values(:,2), data.values(:,3)]), grid on
xlabel('Time (sec)')
legend('Distance (cm)', 'Distance Averaged (cm)')
linkaxes(ax, 'x'), clear ax
xlim([0 data.time(end)])
