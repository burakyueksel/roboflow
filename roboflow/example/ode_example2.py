import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# function that returns dy/dt
def model(y,t,gain):
    dydt = -gain * y
    return dydt

#initial condition
y0 = 5

# time points
t = np.linspace(0,20)

# solve ODE
gain = 3
y = odeint(model,y0,t,args=(gain,))

# plot results
plt.plot(t,y)
plt.xlabel('time')
plt.ylabel('y(t)')
plt.show()
