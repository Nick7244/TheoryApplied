import matplotlib.pyplot as plt
import numpy as np

tf = 1000
ts = [t for t in np.arange(0, tf, 1)]

ys = []
p = 0.1
r = np.log((5+p)/p) / tf
for t in ts:
    ys.append(p*np.exp(r*t) - p)

plt.plot(ts, ys)
plt.show()