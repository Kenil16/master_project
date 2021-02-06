import numpy as np
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
from tf.transformations import*

def eulerAnglesToRotationMatrix(theta):

    R_x = np.array([[1,         0,                  0                   ],
                    [0,         np.cos(theta[0]), -np.sin(theta[0]) ],
                    [0,         np.sin(theta[0]), np.cos(theta[0])  ]])

    R_y = np.array([[np.cos(theta[1]),    0,      np.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-np.sin(theta[1]),   0,      np.cos(theta[1])  ]])

    R_z = np.array([[np.cos(theta[2]),    -np.sin(theta[2]),    0],
                    [np.sin(theta[2]),    np.cos(theta[2]),     0],
                    [0,                     0,                      1]])

    R = np.matmul(R_z, np.matmul( R_y, R_x ))

    return R


#R_M = eulerAnglesToRotationMatrix([np.deg2rad(0), np.deg2rad(0), np.pi])
R = eulerAnglesToRotationMatrix([np.deg2rad(5), np.deg2rad(5), 0])


R = euler_matrix(np.deg2rad(5), np.deg2rad(-5), np.deg2rad(-90), 'rxyz')
acc = np.matmul(R, np.array([0.7, 0.7, 9.79, 1])) - np.matmul(R, np.array([0, 0, 9.79,1]))
print(acc)
