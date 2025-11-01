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

% Save the data
save data_00.mat data

% Load the data
load data_00.mat


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
ind.u_e    = 1;
ind.u_c1   = 2;
ind.u_c2   = 3;
ind.sinarg = 4;

% Parameters
R1 = 4.7e3;  % Ohm
R2 = R1;
C1 = 470e-9; % F
C2 = C1;

a = R1*R2*C1*C2;
b = R1*C1 + R1*C2 + R2*C2;

% Transfer function
s = tf('s');
G_rcrc_mod = 1 / (a*s^2 + b*s + 1);

% Frequency response estimation
Nest     = round(2.0 / Ts);
koverlap = 0.5;
Noverlap = round(koverlap * Nest);
window   = hann(Nest);

% inp = diff( data.values(:,ind.u_e ) );
% out = diff( data.values(:,ind.u_c1) );
% [g, freq] = tfestimate(inp, out, window, Noverlap, Nest, 1/Ts);
% c         = mscohere(inp, out, window, Noverlap, Nest, 1/Ts);
% G1 = frd(g(2:end), freq(2:end), 'Units', 'Hz'); % frd(g, freq, Ts, 'Units', 'Hz');
% C1 = frd(c(2:end), freq(2:end), 'Units', 'Hz'); % frd(c, freq, Ts, 'Units', 'Hz');
% 
% inp = diff( data.values(:,ind.u_e ) );
% out = diff( data.values(:,ind.u_c2) );
% [g, freq] = tfestimate(inp, out, window, Noverlap, Nest, 1/Ts);
% c         = mscohere(inp, out, window, Noverlap, Nest, 1/Ts);
% G2 = frd(g(2:end), freq(2:end), 'Units', 'Hz'); % frd(g, freq, Ts, 'Units', 'Hz');
% C2 = frd(c(2:end), freq(2:end), 'Units', 'Hz'); % frd(c, freq, Ts, 'Units', 'Hz');

% rotating filter
Dlp = sqrt(3) / 2;
wlp = 2 * pi * 10;
Glp = c2d(tf(wlp^2, [1 2*Dlp*wlp wlp^2]), Ts, 'tustin');

inp = diff(apply_rotfiltfilt_via_fft(Glp, data.values(:,ind.sinarg), data.values(:,ind.u_e ), Ts));
out = diff(apply_rotfiltfilt_via_fft(Glp, data.values(:,ind.sinarg), data.values(:,ind.u_c1), Ts));
[G1, C1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = diff(apply_rotfiltfilt_via_fft(Glp, data.values(:,ind.sinarg), data.values(:,ind.u_e ), Ts));
out = diff(apply_rotfiltfilt_via_fft(Glp, data.values(:,ind.sinarg), data.values(:,ind.u_c2), Ts));
[G2, C2] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

figure(2)
bode(G1, G2, G_rcrc_mod, 2*pi*G1.Frequency), grid on
legend('G1', 'G2', 'Grcrc mod', 'Location', 'best')

opt = bodeoptions('cstprefs');
opt.MagUnits = 'abs';
opt.MagScale = 'linear';

figure(3)
bodemag(C1, C2, 2*pi*G1.Frequency, opt), grid on

% Step responses
f_max = 0.8 * 1/(2*Ts);
step_time = (0:Nest-1).'*Ts;
step_resp_1 = get_step_resp_from_frd(G1, f_max);
step_resp_2 = get_step_resp_from_frd(G2, f_max);

step_resp_mod = step(G_rcrc_mod, step_time);

figure(4)
plot(step_time, [step_resp_1, step_resp_2]), grid on, hold on
plot(step_time, step_resp_mod), hold off
legend('G1', 'G2', 'Grcrc mod', 'Location', 'best')
xlabel('Time (sec)'), ylabel('Voltage (V)')
xlim([0 0.05])
