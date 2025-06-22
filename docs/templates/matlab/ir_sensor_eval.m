clc, clear variables
%% parameter fit

% Distance Data
dist_cm = [1:1:20, ...
    22:2:40, ...
    42:3:81].';

dist_mV = [1620, 2250, 2350, 2680, 3100, 3107, 3108, 2900, 2710, 2485, ...
    2205, 2060, 1930, 1803, 1700, 1595, 1520, 1410, 1350, 1310, ...
    1204, 1100, 1050, 965, 905, 845, 808, 770, 733, 697, ...
    678, 628, 610, 570, 560, 532, 500, 480, 470, 467, 450, 440, 414, 411].';

% Define a fit region: in the example we only want [7, 70]
ind_fit = dist_cm >= 7.0 & dist_cm <= 70;

% Model function: a / (x + b)
ft = fittype('a / (x + b)');

% Curve fitting
f1 = fit(dist_mV(ind_fit), dist_cm(ind_fit), ft);

fprintf("   Fitted parameters:\n")
fprintf("   a = %.4f,  b = %.4f\n", f1.a, f1.b);

figure(1)

% 1) dist_cm vs. dist_mV
subplot(131)
plot(dist_cm, dist_mV, 'x-'), grid on
axis([0 max(dist_cm) 0 max(dist_mV)])
xlabel('Distance (cm)'), ylabel('Voltage (mV)')

% 2) dist_mV vs. dist_cm with fitted model (over the fit region)
subplot(132)
plot(dist_mV, dist_cm, 'x-'), grid on, hold on
plot(dist_mV(ind_fit), f1(dist_mV(ind_fit)), 'x-'), hold off
ylabel('Distance (cm)'), xlabel('Voltage (mV)')
legend('Measured', 'Fitted Fcn.')
axis([0 max(dist_mV) 0 max(dist_cm)])

% 3) dist_cm vs. fitted_cm
subplot(133)
plot(dist_cm, dist_cm, 'x-'), grid on, hold on
plot(dist_cm, f1(dist_mV), 'x-'), grid on, hold off
axis([0  max(dist_cm) 0 max(dist_cm)])
xlabel('Distance (cm)'), ylabel('Fit (cm)')

% set(findall(gcf, 'type', 'line'), 'linewidth', 1.5)
