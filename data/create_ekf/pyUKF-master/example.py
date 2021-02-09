from ukf import UKF
import csv
import numpy as np
import math
import matplotlib.pyplot as plt



class ukf():
    
    def __init__(self):

        self.corr_acc_x = []
        self.corr_acc_y = []
        self.corr_acc_z = []
        self.data = []

        self.read_data('test1.txt')
        self.mx = np.array([item[0] for item in self.data])
        self.my = np.array([item[1] for item in self.data])
        self.mz = np.array([item[2] for item in self.data])

        self.macc_x = np.array([item[3] for item in self.data])
        self.macc_y = np.array([item[4] for item in self.data])
        self.macc_z = np.array([item[5] for item in self.data])

        self.mpsi = np.array([item[6] for item in self.data])
        self.mphi = np.array([item[7] for item in self.data])
        self.mtheta = np.array([item[8] for item in self.data])

        self.mgyro_x = np.array([item[9] for item in self.data])
        self.mgyro_y = np.array([item[10] for item in self.data])
        self.mgyro_z = np.array([item[11] for item in self.data])

        self.mtime = np.array([item[12] for item in self.data])
        self.old_x = 0.0

        self.x0 = [] #x
        self.x1 = [] #y
        self.x2 = [] #z
        self.x3 = [] #vel_x
        self.x4 = [] #vel_y
        self.x5 = [] #vel_z
        self.x6 = [] #acc_x
        self.x7 = [] #acc_y
        self.x8 = [] #acc_z
        self.x9 = [] #psi
        self.x10 = [] #phi
        self.x11 = [] #theta
        self.x12 = [] #gyro_x
        self.x13 = [] #gyro_y
        self.x14 = [] #gyro_z

        self.test = 0.0

    def read_data(self, file_name):

        with open(file_name,"r") as text_file:
            for line in text_file:
                items = line.split(' ')
                items[-1] = items[-1].strip()
                str_to_float = [float(item) for item in items]
                if not str_to_float[0] == 0:
                    self.data.append(str_to_float)

    def eulerAnglesToRotationMatrixYaw(self, theta):

            R_z = np.array([[np.cos(theta),    -np.sin(theta),    0],
                            [np.sin(theta),    np.cos(theta),     0],
                            [0,                     0,            1]])

            return R_z

    def eulerAnglesToRotationMatrix(self, theta):

            R_x = np.array([[1,         0,                  0                   ],
                            [0,         np.cos(theta[0]), -np.sin(theta[0]) ],
                            [0,         np.sin(theta[0]), np.cos(theta[0])  ]])

            R_y = np.array([[np.cos(theta[1]),    0,      np.sin(theta[1])  ],
                            [0,                     1,      0                   ],
                            [-np.sin(theta[1]),   0,      np.cos(theta[1])  ]])

            R_z = np.array([[np.cos(theta[2]),    -np.sin(theta[2]),    0],
                            [np.sin(theta[2]),    np.cos(theta[2]),     0],
                            [0,                     0,                      1]])

            
            R = np.matmul(R_x, np.matmul( R_y, R_z ))
            return R
    
    def iterate_x(self, x, dt, inputs):
        '''this function is based on the x_dot and can be nonlinear as needed'''
        
        ret = np.zeros(len(x))
        bias = [-0.190006656546, -0.174740895383]
        gravity = 9.79531049538

        #Rotation matrix to align angle to that of the acceleration of the drone. Hences the gravity can be subtracted 
        R1 = self.eulerAnglesToRotationMatrix([0, 0, 2*x[11] + np.pi/2])
        a = np.matmul(R1, np.array([x[9], x[10], x[11]]))

        #Correct for acceleration shift for the orientation of the drone and bias for x and y
        R2 = self.eulerAnglesToRotationMatrix([x[10], x[9], x[11]])
        acc = np.matmul(R2, np.array([x[6], x[7], 0]))
        acc_bias = np.matmul(R2, np.array([bias[0], bias[1], 0]))

        corrected_acc_x = acc[0] - acc_bias[0] + np.sin(a[0])*gravity
        corrected_acc_y = acc[1] - acc_bias[1] + np.sin(a[1])*gravity
        corrected_acc_z = x[8] - gravity

        #Rotation matrix to align gyro velocity to the orientation of the world (marker)
        R3 = self.eulerAnglesToRotationMatrix([0, 0, x[11]])
        corrected_gyro = np.matmul(R3, np.array([x[12], x[13], x[14]]))

        ret[0] = x[0] + x[3]*dt + 0.5*corrected_acc_x*dt**2
        ret[1] = x[1] + x[4]*dt + 0.5*corrected_acc_y*dt**2
        ret[2] = x[2] + x[5]*dt + 0.5*corrected_acc_z*dt**2
        ret[3] = x[3] + corrected_acc_x*dt
        ret[4] = x[4] + corrected_acc_y*dt
        ret[5] = x[5] + corrected_acc_z*dt
        ret[6] = x[6]
        ret[7] = x[7]
        ret[8] = x[8]
        ret[9] = x[9] + corrected_gyro[1]*dt #+ (np.cos(x[11])*x[12]*dt - np.sin(x[11])*x[13]*dt)
        ret[10] = x[10] + corrected_gyro[0]*dt # - (-np.sin(x[11])*x[12]*dt + np.cos(x[11])*x[13]*dt)
        ret[11] = x[11] + corrected_gyro[2]*dt #+ x[14]*dt
        ret[12] = x[12]
        ret[13] = x[13]
        ret[14] = x[14]

        #print(x)
        #self.test = self.test + x[14]*dt


        return ret
    
    def main(self):
        
        np.set_printoptions(precision=3)

        # Process Noise
        q = np.eye(15)
        q[0][0] = 0.009
        q[1][1] = 0.009
        q[2][2] = 0.009
        q[3][3] = 0.04
        q[4][4] = 0.04
        q[5][5] = 0.04
        q[6][6] = 0.05
        q[7][7] = 0.05
        q[8][8] = 0.05
        q[9][9] = 0.05
        q[10][10] = 0.05
        q[11][11] = 0.05
        q[12][12] = 0.005
        q[13][13] = 0.005
        q[14][14] = 0.005
        
        # Create measurement noise covariance matrices
        r_imu = np.zeros([6, 6])
        r_imu[0][0] = 200.1
        r_imu[1][1] = 200.1
        r_imu[2][2] = 200.0
        r_imu[3][3] = 0.02
        r_imu[4][4] = 0.02
        r_imu[5][5] = 0.001
        
        r_vision = np.zeros([6, 6])
        r_vision[0][0] = 0.1
        r_vision[1][1] = 0.1
        r_vision[2][2] = 0.1
        r_vision[3][3] = 0.01
        r_vision[4][4] = 0.01
        r_vision[5][5] = 0.01

        # pass all the parameters into the UKF!
        # number of state variables, process noise, initial state, initial coariance, three tuning paramters, and the iterate function
        measurements = np.vstack((self.mx, self.my, self.mz, self.macc_x, self.macc_y, self.macc_z, self.mpsi, self.mphi, self.mtheta, self.mgyro_x, self.mgyro_y, self.mgyro_z))
        m = measurements.shape[1]
        print(measurements.shape)
        i = measurements
        x_init = [i[0,0], i[1,0], i[2,0], 0 ,0 ,0, i[3,0], i[4,0], i[5,0], i[6,0], i[7,0], i[8,0], i[9,0], i[10,0], i[11,0]]
        state_estimator = UKF(15, q, x_init, 0.0001*np.eye(15), 0.04, 0.0, 2.0, self.iterate_x)
        print(x_init)
        dt = 0.0
        print(range(m))
        for j in range(m):
            
            row = [i[0,j], i[1,j], i[2,j], i[3,j], i[4,j], i[5,j], i[6,j], i[7,j], i[8,j], i[9,j], i[10,j], i[11,j]]
            #print(row)        
            if j:
                dt = self.mtime[j]-self.mtime[j-1]
            
            # create an array for the data from each sensor
            vision_x = row[0]
            vision_y = row[1]
            vision_z = row[2]
            vision_roll = row[6]
            vision_pitch = row[7]
            vision_yaw = row[8]

            imu_accX = row[3]
            imu_accY = row[4]
            imu_accZ = row[5]
            imu_rollV = row[9]
            imu_pitchV = row[10]
            imu_yawV = row[11]

            vision_data = np.array([vision_x, vision_y, vision_z, vision_roll, vision_pitch, vision_yaw])
            imu_data = np.array([imu_accX, imu_accY, imu_accZ, imu_rollV, imu_pitchV, imu_yawV])

            # prediction is pretty simple
            state_estimator.predict(dt)

            # updating isn't bad either
            # remember that the updated states should be zero-indexed
            # the states should also be in the order of the noise and data matrices
            if not self.old_x == row[0]:
                state_estimator.update([0, 1, 2, 9, 10, 11], vision_data, r_vision)
                
            state_estimator.update([6, 7, 8, 12, 13, 14], imu_data, r_imu)

            self.old_x = row[0]

            #print ("Estimated state: ", state_estimator.get_state())

            # Save states for Plotting
            
            x = state_estimator.get_state()

            self.x0.append(float(x[0]))
            self.x1.append(float(x[1]))
            self.x2.append(float(x[2]))
            self.x3.append(float(x[3]))
            self.x4.append(float(x[4]))
            self.x5.append(float(x[5]))
            self.x6.append(float(x[6]))
            self.x7.append(float(x[7]))
            self.x8.append(float(x[8]))
            self.x9.append(float(np.rad2deg(x[9])))
            self.x10.append(float(np.rad2deg(x[10])))
            self.x11.append(float(np.rad2deg(x[11])))

    def plot_state(self, title, x_label, y_label, fig_name, x, y):

        fig, ax = plt.subplots()
        ax.set_axisbelow(True)
        ax.set_facecolor('#E6E6E6')
        plt.grid(color='w', linestyle='solid')

        for spine in ax.spines.values():
            spine.set_visible(False)

        ax.xaxis.tick_bottom()
        ax.yaxis.tick_left()
        ax.tick_params(colors='gray', direction='out')

        for tick in ax.get_xticklabels():
            tick.set_color('gray')
        for tick in ax.get_yticklabels():
            tick.set_color('gray')

        for label, y in zip(labels, y):
            plt.plot(x, y, linewidth=0.5, label=label)

        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.title(title)
        plt.legend(loc='best')
        plt.savefig(fig_name)

    def plot_position(self):

        fig, ax = plt.subplots()
        ax.set_axisbelow(True)
        ax.set_facecolor('#E6E6E6')
        plt.grid(color='w', linestyle='solid')

        for spine in ax.spines.values():
            spine.set_visible(False)

        ax.xaxis.tick_bottom()
        ax.yaxis.tick_left()
        ax.tick_params(colors='gray', direction='out')

        for tick in ax.get_xticklabels():
            tick.set_color('gray')
        for tick in ax.get_yticklabels():
            tick.set_color('gray')

        plt.scatter(self.mx, self.my, s=10, label='GPS Measurements')
        plt.scatter(self.x0, self.x1, s=2, label='EKF Position', c='k')

        # Start/Goal
        plt.scatter(self.mx[0], self.my[0], s=60, label='Start', c='g')
        plt.scatter(self.mx[-1], self.my[-1], s=60, label='Goal', c='r')

        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Position')
        plt.legend(loc='best')
        plt.axis('equal')

        plt.savefig('Position.png')

if __name__ == "__main__":

    ukf = ukf()
    ukf.main()

    labels = ['x','y','z']
    ys = [ukf.x6, ukf.x7, ukf.x8]
    ukf.plot_state('Acceleration [IMU]', 'Time [s]', r'Acceleration [$(\frac{m}{s^2})$]', 'Acceleration.png', ukf.mtime, ys)

    labels = ['Velocity in x','Velocity in y','Velocity in z']
    ys = [ukf.x3, ukf.x4, ukf.x5]
    ukf.plot_state('Linear velocity [IMU]', 'Time [s]', r'Velocity [$(\frac{m}{s})$]', 'Linear velocity.png', ukf.mtime, ys)

    labels = ['Roll','Pitch']
    ys = [ukf.x9, ukf.x10, ukf.x11]
    ukf.plot_state('Angle [IMU and vision fusion]', 'Time [s]', r'Angle [Degress]', 'Orientation.png', ukf.mtime, ys)

    labels = ['x','y','z']
    ys = [ukf.mgyro_x, ukf.mgyro_y, ukf.mgyro_z]
    ukf.plot_state('Angular velocity [IMU]', 'Time [s]', r'Velocity [$(\frac{m}{s})$]', 'Velocity.png', ukf.mtime, ys)

    ukf.plot_position()

