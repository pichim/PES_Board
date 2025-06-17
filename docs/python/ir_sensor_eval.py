# conda create --name pes-env python=3.11.4 numpy scipy matplotlib pyserial control ipykernel

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Distance Data
dist_cm = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 45, 48, 51, 54, 57, 60, 63, 66, 69, 72, 75, 78, 81])

dist_mV = np.array([1620, 2250, 2350, 2680, 3100, 3107, 3108, 2900, 2710, 2485, 2205, 2060, 1930, 1803, 1700, 1595, 1520, 1410, 1350, 1310, 1204, 1100, 1050, 965, 905, 845, 808, 770, 733, 697, 678, 628, 610, 570, 560, 532, 500, 480, 470, 467, 450, 440, 414, 411])

# Define a fit region: in the example we only want [7, 70]
ind_fit = (dist_cm >= 7.0) & (dist_cm <= 70.0)


# Model function: a / (x + b)
def model(x, a, b):
    return a / (x + b)


# Curve fitting
popt, pcov = curve_fit(model, dist_mV[ind_fit], dist_cm[ind_fit])
a_opt, b_opt = popt
print("   Fitted parameters:")
print(f"   a = {a_opt:.4f},  b = {b_opt:.4f}")

# Evaluate the model for plotting ---
fitted_cm = model(dist_mV, a_opt, b_opt)

# Plot (3 subplots)
fig, axs = plt.subplots(1, 3, figsize=(15, 5))

# 1) dist_cm vs. dist_mV
axs[0].plot(dist_cm, dist_mV, "x-")
axs[0].grid(True)
axs[0].set_xlabel("Distance (cm)")
axs[0].set_ylabel("Voltage (mV)")
axs[0].set_xlim([0, dist_cm.max()])
axs[0].set_ylim([0, dist_mV.max()])

# 2) dist_mV vs. dist_cm with fitted model (over the fit region)
axs[1].plot(dist_mV, dist_cm, "x-", label="Measured")
axs[1].plot(dist_mV[ind_fit], fitted_cm[ind_fit], "x-", label="Fitted Fcn.")
axs[1].grid(True)
axs[1].set_ylabel("Distance (cm)")
axs[1].set_xlabel("Voltage (mV)")
axs[1].set_xlim([0, dist_mV.max()])
axs[1].set_ylim([0, dist_cm.max()])
axs[1].legend()

# 3) dist_cm vs. fitted_cm
axs[2].plot(dist_cm, dist_cm, "x-")
axs[2].plot(dist_cm, fitted_cm, "x-")
axs[2].grid(True)
axs[2].set_xlabel("Distance (cm)")
axs[2].set_ylabel("Fit (cm)")
axs[2].set_xlim([0, dist_cm.max()])
axs[2].set_ylim([0, dist_cm.max()])

plt.tight_layout()
plt.show()
