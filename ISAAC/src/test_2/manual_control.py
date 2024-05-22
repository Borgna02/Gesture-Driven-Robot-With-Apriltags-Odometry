import numpy as np

velvec = np.array([0, #linear velocity 
                   5 #angular velocity
                   ])

np.save("src/test_2/velvec.npy", velvec)
velvec = np.load("src/test_2/velvec.npy")

print(velvec)