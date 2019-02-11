import matplotlib.pyplot as plt
import numpy as np
import scipy as sp
import scipy.integrate

tmax = 10.0

def u(t):
    if t < tmax / 2.0:
        return ((tmax / 2.0) - t) / (tmax / 2.0)
    else:
        return 1.0

def func(x, t, u):
    return - (x - u(t))

x0 = 0.8
t = np.linspace(0.0, tmax, 1000)
args = (u,)
y = sp.integrate.odeint(func, x0, t, args)

fig = plt.figure()
ax = fig.add_subplot(111)
h1, = ax.plot(t, y)
h2, = ax.plot(t, [u(s) for s in t])
ax.legend([h1, h2], ["y", "u"])
ax.set_xlabel("t")
ax.grid()
plt.show()