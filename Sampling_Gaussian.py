# I need to sample A Gaussian Distribution & Graph it
import math
import matplotlib.pyplot as plt
import numpy as np

# Create the Standard normal Distribution Function
x = np.linspace(-3, 3, 1000)  # Need to make this into 1000 samples use np.linspace to do this

e = np.e
pi = np.pi
sqrt_pi = np.sqrt(pi)
half_gaussian_function = 1/(2*sqrt_pi)
power_on_e = (-0.5*np.power(x, 2))
other_half_gaussian = np.power(e, power_on_e)

Gaussian_function = half_gaussian_function * other_half_gaussian  # This is my Y output
y = Gaussian_function

fig, ax = plt.subplots(figsize=(5, 2.7), layout='constrained')

ax.plot(x, y, label='Gaussian Distribution')

ax.legend()

ax.set_title("The Standard Normal Distribution")

plt.show()

# Create a finite number of terms N which we will pass into the Standard Gaussian Function

# Then I need to plot this & I will label My axis
