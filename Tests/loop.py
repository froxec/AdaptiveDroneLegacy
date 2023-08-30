import oclock
import numpy as np
timer = oclock.Timer(0.01)

while True:
    print(np.random.random())
    timer.checkpt()