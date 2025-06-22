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
file_name = 'data_pi_controller_00.mat';
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

% index
ind.counts    = 1;
ind.velocity  = 2;
ind.rotations = 3;
ind.voltage   = 4;
ind.sinarg    = 5;

figure(2)
subplot(211)
plot(data.time, data.values(:, ind.voltage)), grid on
ylabel('Voltage (V)')
subplot(212)
plot(data.time, data.values(:, ind.sinarg)), grid on
ylabel('Sinarg')
xlabel('Time (sec)')

figure(3)
subplot(311)
plot(data.time, data.values(:, ind.counts)), grid on
ylabel('Counts')
subplot(312)
plot(data.time, data.values(:, ind.velocity)), grid on
ylabel('Velocity (RPS)')
subplot(313)
plot(data.time, data.values(:, ind.rotations)), grid on
ylabel('Rotations')
xlabel('Time (sec)')


%%

% Estimate Frequency Response
Nest     = round(2.0 / Ts);
koverlap = 0.9;
Noverlap = round(koverlap * Nest);
window   = hann(Nest);
inp = diff( data.values(:,ind.voltage) );
out = diff( data.values(:,ind.velocity) );
[g, freq] = tfestimate(inp, out, window, Noverlap, Nest, 1/Ts);
c         = mscohere(inp, out, window, Noverlap, Nest, 1/Ts);
G = frd(g(2:end), freq(2:end), 'Units', 'Hz'); % frd(g, f, Ts, 'Units', 'Hz');
C = frd(c(2:end), freq(2:end), 'Units', 'Hz'); % frd(c, f, Ts, 'Units', 'Hz');

% Print the DC-Gain
fprintf('Measured DC-Gain: %0.2f dB\n', db(abs(g(2))));

% Model the plant as transfer function
s = tf('s');
% K = db2mag(0); % adjust the values here
% n = 1;
% T1 = 1.0 / (2*pi* 100);
% Tt = 0.1;
K = db2mag(-12.5);
n = 2;
T1 = 1.0 / (2*pi* 9.5);
Tt = 1e-3;
G_mod = K / (T1*s + 1.0)^n;

% Add dead time using 1st-order Pade approximation
[num, den] = pade(Tt, 1);
G_mod = G_mod * tf(num, den);

figure(4)
bode(G, G_mod, 2*pi*G.Frequency), grid on
legend('Measured', ...
    'Model', ...
    'Location', 'best')

opt = bodeoptions('cstprefs');
opt.MagUnits = 'abs';
opt.MagScale = 'linear';

figure(5)
bodemag(C, 2*pi*C.Frequency, opt), grid on


%%

% Chien-Hrones-Reswick PI tuning
% Reference: Chien, Hrones, Reswick (1952), Åström & Hägglund, Seborg et al.
% Assumed model: First-order plus dead time (FOPDT):
% 
%   G(s) = K / (T1*s + 1) * e^(-Tt*s)
% 
% Approximation of your actual plant:
% - The n-lag system is approximated to an equivalent FOPDT using:
% - These formulas are empirically derived from Åström & Hägglund, based on matching step response behavior.
%   Purpose: This allows the use of classical CHR PI tuning formulas based on FOPDT.
% 
%   T1_approx = T1 * (n + 1)/2
%   Tt_approx = T1 * (n - 1)/2 + Tt
% 
% Tuning for ~20% overshoot (faster tuning)
% 
%   Kp = 0.6 * T1 / (Tt * K)
%   Ti = T1
% 
% - Here the plant approximation (FOPDT) is used.

% Approximate equivalent dominant time constant
T1_approx = T1 * (n + 1) / 2;
% Approximate effective delay
Tt_approx = T1 * (n - 1) / 2 + Tt;

% PI Controller
Kp = 0.6 * T1_approx / (Tt_approx * K);
Ti = T1_approx;
Ki = Kp / Ti;
C = Kp + Ki/s;

% Controller Gains
fprintf('PI Controller P gain: %0.2f\n', Kp);
fprintf('PI Controller I gain: %0.2f\n', Ki);

% Closed-Loop System
T = feedback(C * G, 1);
T_mod = feedback(C * G_mod, 1);

% Step responses
f_max = 80;
step_time = (0:Nest-1).'*Ts;
step_resp = get_step_resp_from_frd(T, f_max);

step_resp_mod = step(T_mod, step_time);

figure(6)
plot(step_time, step_resp), grid on, hold on
plot(step_time, step_resp_mod), hold off
xlabel('Time (sec)'), ylabel('Velocity (RPS)')
legend('Calculated based of FRF', ...
    'Model', ...
    'Location', 'best')
xlim([0 0.2])
