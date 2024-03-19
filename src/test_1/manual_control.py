import numpy as np

velvec = np.array([0.011, #linear velocity 
                   0 #angular velocity
                   ])

np.save("src/test_1/velvec.npy", velvec)
velvec = np.load("src/test_1/velvec.npy")

print(velvec)