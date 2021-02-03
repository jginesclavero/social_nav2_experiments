import matplotlib
import numpy as np
import math
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker

delta = 0.025
x = np.arange(-4.0, 4.0 , delta)
y = np.arange(-4.0, 4.0, delta)
var_h = 2.0
var_s = 0.7
var_r = 0.7
orientation = 0.0

X, Y = np.meshgrid(x, y)


def normalize_angle(rad):
    rad_mod = math.fmod(rad, 2. * np.pi )
    if rad_mod < -np.pi:
        return rad_mod + 2. * np.pi
    elif rad_mod > np.pi:
        return rad_mod - 2. * np.pi
    else:
        return rad_mod

def asymmetric_gaussian(x, y, angle, var_h, var_s, var_r):
    output = []
    x0 = 0.0
    y0 = 0.0
    for i in range(len(x[0])):
        y_values = []
        for j in range(len(y[0])):
            dx = x[0][i] - x0
            dy = y[j][0] - y0
            alpha_n = np.arctan2(dy,dx) - angle + np.pi / 2
            alpha_n = normalize_angle(alpha_n)
            if alpha_n <= 0.0:
                sigma = var_r
            else:
                sigma = var_h
            a = (np.power(np.cos(angle), 2.) / (2. * np.power(sigma, 2.))) + \
            (np.power(np.sin(angle), 2.) / (2. * np.power(var_s, 2.)))
            b = ((2. * np.sin(angle) * np.cos(angle)) / (4. * np.power(sigma, 2.))) - \
            ((2. * np.sin(angle) * np.cos(angle)) / (4. * np.power(var_s, 2.)))
            c = (np.power(np.sin(angle), 2.) / (2. * np.power(sigma, 2.))) + \
            (np.power(np.cos(angle), 2.) / (2. * np.power(var_s, 2.)))
            f1 = a * (np.power(x[0][i], 2.) + np.power(x0, 2.) - 2 * x[0][i] * x0)
            f2 = 2 * b * dx * dy
            f3 = c * (np.power(y[j][0], 2.) + np.power(y0, 2.) - 2. * y[j][0] * y0)
            y_values.append(np.exp(-(f1 + f2 + f3)))
        output.append(y_values)
    return output


fig, ax = plt.subplots(figsize=(8, 8))
CS = ax.contour(X, Y, asymmetric_gaussian(X, Y, orientation, var_h, var_s, var_r), cmap='gnuplot')
ax.clabel(CS, inline=1, fontsize=10)
plt.xlabel('Y (meters)')
plt.ylabel('X (meters)')
loc = plticker.MultipleLocator(base=1.0) # this locator puts ticks at regular intervals
ax.xaxis.set_major_locator(loc)
ax.yaxis.set_major_locator(loc)
plt.grid(True)
plt.show()