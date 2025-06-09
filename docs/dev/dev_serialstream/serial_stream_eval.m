clc, clear all
%%

port = '/dev/ttyUSB0';
baudrate = 2e6;

if (~exist('serialStream', 'var'))
    serialStream = SerialStream(port, baudrate);
else
    serialStream.reset();
end

serialStream.start()
while (serialStream.isBusy())
    pause(0.1);
end

% access the data
data = serialStream.getData();


%%

% index
ind.counts    = 1;
ind.velocity  = 2;
ind.rotations = 3;
ind.voltage   = 4;
ind.sinarg    = 5;

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

figure(2)
subplot(311)
plot(data.time, data.values(:, ind.counts)), grid on
subplot(312)
plot(data.time, data.values(:, ind.velocity)), grid on
subplot(313)
plot(data.time, data.values(:, ind.rotations)), grid on

figure(3)
subplot(211)
plot(data.time, data.values(:, ind.voltage)), grid on
subplot(212)
plot(data.time, data.values(:, ind.sinarg)), grid on

%% 

% save chirp_excitation_fcut_15_Hz_00 data
% save chirp_excitation_fcut_30_Hz_00 data

% gpa_data = readmatrix('gpa_excitation_fcut_15_Hz_00.txt');
% U = gpa_data(:,2) + 1i*gpa_data(:,3);
% Y = gpa_data(:,4) + 1i*gpa_data(:,5);
% R = gpa_data(:,6) + 1i*gpa_data(:,7);
% P = frd(Y./U, gpa_data(:,1), Ts, 'Units', 'Hz'); % frd(Y./U, data(:,1), Ts, 'Units', 'Hz');
% T = frd(Y./R, gpa_data(:,1), Ts, 'Units', 'Hz'); % frd(Y./R, data(:,1), Ts, 'Units', 'Hz');


%%

% estimate Frequency Response
inp = diff( data.values(:,ind.voltage) );
out = diff( data.values(:,ind.velocity) );
Nest     = round(5.0 / Ts);
koverlap = 0.95;
Noverlap = round(koverlap * Nest);
window   = hann(Nest);
[g, f] = tfestimate(inp, out, window, Noverlap, Nest, 1/Ts);
c      = mscohere(inp, out, window, Noverlap, Nest, 1/Ts);
G = frd(g, f, 'Units', 'Hz'); % frd(g, f, Ts, 'Units', 'Hz');
C = frd(c, f, 'Units', 'Hz'); % frd(c, f, Ts, 'Units', 'Hz');

s = tf('s');
% K = db2mag(-3.24);
% T1 = 1.0 / (2*pi*  6.0);
% T2 = 1.0 / (2*pi* 15.0);
% Tt = 1e-3;
% G_mod = K * ...
%     1.0 / (T1*s + 1.0) * ...
%     1.0 / (T2*s + 1.0)^2;
% G_mod.InputDelay = Tt;

K = db2mag(-3.24);
n = 3;
T1 = 1.0 / (2*pi* 11.0);
Tt = 1e-3;
G_mod = K * ...
    1.0 / (T1*s + 1.0)^n;
G_mod.InputDelay = Tt;

f_max = inf;

figure(5)
bode(G,G_mod, 2*pi*G.Frequency(G.Frequency < f_max))

figure(6)
bodemag(C, 2*pi*C.Frequency(C.Frequency < f_max))

% % DCMotor: velocity controller parameters: kp = 1.6800, ki = 56.0000, kd = 0.0077
% Kp = 1.68;
% Ki = 56.0;


% Symmetrical Optimum (SO) PI tuning
% Reference: Kessler (1955), Åström & Hägglund "Advanced PID Control", Chapter 8.3
% Assumed model: n-lag system with dead time:
%   G(s) = K / (1 + T1 s)^n * e^(-Tt s)
% Assumption used directly matches your actual plant.
% Design principle: Place the PI zero at omega_PI = 1 / (4*T1), which gives Ti = 4*T1.
% The gain Kp is chosen empirically as ~1.25/K, providing good phase margin (~60 deg) for n=3.

Ti_SO = 4 * T1;                  % PI zero at 1/(4*T1)
Kp_SO = 1.25 / K;                % Empirical SO gain for n=3 identical lags
Ki_SO = Kp_SO / Ti_SO;           % Integral gain
C_SO = pid(Kp_SO, Ki_SO);        % PI controller

% Chien-Hrones-Reswick (CHR) PI tuning
% Reference: Chien, Hrones, Reswick (1952), Åström & Hägglund, Seborg et al.
% Assumed model: First-order plus dead time (FOPDT):
%   G(s) = K_proc / (T_approx s + 1) * e^(-L_approx s)
%
% Approximation of your actual plant:
% - The n-lag system is approximated to an equivalent FOPDT using:
%   T_approx = T1 * (n + 1)/2
%   L_approx = T1 * (n - 1)/2 + Tt
% - These formulas are empirically derived from Åström & Hägglund, based on matching step response behavior.
% Purpose: This allows the use of classical CHR PI tuning formulas based on FOPDT.

T_approx = T1 * (n + 1) / 2;     % Approximate equivalent dominant time constant
L_approx = T1 * (n - 1) / 2 + Tt;% Approximate effective delay
K_proc = K;                      % Process gain

% CHR PI tuning for 0% overshoot (safe, conservative tuning)
% Assumed closed-loop target: Critically damped response without overshoot.
% Tuning rules:
%   Kp = 0.35 * (T / L) * (1 / K)
%   Ti = 1.2 * T

Kp_CHR = 0.35 * (T_approx / L_approx) / K_proc;
Ti_CHR = 1.2 * T_approx;
Ki_CHR = Kp_CHR / Ti_CHR;
C_CHR_0 = pid(Kp_CHR, Ki_CHR);

% CHR PI tuning for ~20% overshoot (faster tuning)
% Assumed closed-loop target: Faster response with ~20% overshoot allowed.
% Tuning rules:
%   Kp = 0.6 * (T / L) * (1 / K)
%   Ti = T
% The same plant approximation (FOPDT) is used.

Kp_CHR = 0.6 * (T_approx / L_approx) / K_proc;
Ti_CHR = T_approx;
Ki_CHR = Kp_CHR / Ti_CHR;
C_CHR_20 = pid(Kp_CHR, Ki_CHR);

% Internal Model Control (IMC) PI tuning
% Reference: Rivera et al., AIChE Journal (1986), Skogestad & Postlethwaite
% Assumed model: FOPDT:
%   G(s) = K_proc / (T_approx s + 1) * e^(-L_approx s)
%
% Approximation of your actual plant: Same as for CHR → FOPDT with T_approx, L_approx.
% IMC principle:
% - Desired closed-loop time constant lambda is chosen by designer (tradeoff between speed and robustness).
% - Standard IMC PI tuning formulas:
%   Kp = T / [K * (lambda + L)]
%   Ti = T

lambda = 0.7*T_approx;   % Typical initial choice: lambda = T_approx (can be increased for more robustness)
Kp_IMC = T_approx / (K_proc * (lambda + L_approx));
Ti_IMC = T_approx;
Ki_IMC = Kp_IMC / Ti_IMC;
C_IMC = pid(Kp_IMC, Ki_IMC);


C = C_IMC;


L = C * G_mod;
T = feedback(L,1);

figure(7)
margin(L)

figure(8)
step(T)
