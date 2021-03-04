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

        self.read_data('imu_vision_data.txt')
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

        self.x15 = [] #gyro_x
        self.x16 = [] #gyro_y
        self.x17= [] #gyro_z
        
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.test = 0

        self.new_route_x = []
        self.new_route_y = []
        self.new_route_z = []

        self.create_ground_truth()
        
        #Used in the GA (chromosome)
        self.covariance = [0.05, 0.05, 0.05, #Process noise x, y and z (pos)
                           0.04, 0.04, 0.04, #Process noise x, y and z (rate)
                           3.02, 3.02, 6.8, #Process noise x, y and z (acc)
                           1.5, 0.005, 0.05, #Process noise roll, pitch and yaw (angle)
                           0.9, 0.9, 0.5, #Process noise x, y and z (angle rate)
                           2.3, 2.3, 2.3, #Measurement noise x, y and z (pos)
                           0.03, 0.03, 0.08, #Measurement noise x, y and z (acc)
                           0.05, 0.05, 0.05, #Measurement noise x, y and z (angle)
                           1.5, 1.5, 0.5] #Measurement noise x, y and z (abgle rate)
        
        # From best individual using GA 
        #self.covariance = self.load_chromosome('ga_data/best_chromosome.txt')
    
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

        ret[0] = x[0] + x[3]*dt #+ 0.5*x[6]*dt**2
        ret[1] = x[1] + x[4]*dt #+ 0.5*x[7]*dt**2
        ret[2] = x[2] + x[5]*dt #+ 0.5*x[8]*dt**2
        ret[3] = x[3] + x[6]*dt
        ret[4] = x[4] + x[7]*dt
        ret[5] = x[5] + x[8]*dt
        ret[6] = x[6]
        ret[7] = x[7]
        ret[8] = x[8]
        ret[9] = x[9] + x[13]*dt# - x[15]
        ret[10] = x[10] + x[12]*dt# - x[16]
        ret[11] = x[11] + x[14]*dt
        ret[12] = x[12] 
        ret[13] = x[13] 
        ret[14] = x[14]

        return ret
    
    def main(self):
        
        np.set_printoptions(precision=3)
        
        # Process Noise
        q = np.eye(15)
        q[0][0] = self.covariance[0]
        q[1][1] = self.covariance[1]
        q[2][2] = self.covariance[2]
        q[3][3] = self.covariance[3]
        q[4][4] = self.covariance[4]
        q[5][5] = self.covariance[5]
        q[6][6] = self.covariance[6]
        q[7][7] = self.covariance[7]
        q[8][8] = self.covariance[8]
        q[9][9] = self.covariance[9]
        q[10][10] = self.covariance[10]
        q[11][11] = self.covariance[11]
        q[12][12] = self.covariance[12]
        q[13][13] = self.covariance[13]
        q[14][14] = self.covariance[14]
        
        # Create measurement noise covariance matrices
        r_imu_acc = np.zeros([3, 3])
        r_imu_gyro_v = np.zeros([3, 3])
        r_imu_acc[0][0] = self.covariance[15]
        r_imu_acc[1][1] = self.covariance[16]
        r_imu_acc[2][2] = self.covariance[17]
        r_imu_gyro_v[0][0] = self.covariance[18]
        r_imu_gyro_v[1][1] = self.covariance[19]
        r_imu_gyro_v[2][2] = self.covariance[20]
        
        r_vision_pos = np.zeros([3, 3])
        r_vision_ori = np.zeros([3, 3])
        r_vision_pos[0][0] = self.covariance[21]
        r_vision_pos[1][1] = self.covariance[22]
        r_vision_pos[2][2] = self.covariance[23]
        r_vision_ori[0][0] = self.covariance[24]
        r_vision_ori[1][1] = self.covariance[25]
        r_vision_ori[2][2] = self.covariance[26]
        
        # pass all the parameters into the UKF!
        # number of state variables, process noise, initial state, initial coariance, three tuning paramters, and the iterate function
        measurements = np.vstack((self.mx, self.my, self.mz, self.macc_x, self.macc_y, self.macc_z, self.mpsi, self.mphi, self.mtheta, self.mgyro_x, self.mgyro_y, self.mgyro_z))
        m = measurements.shape[1]
        #print(measurements.shape)
        i = measurements
        x_init = [i[0,0], i[1,0], i[2,0], 0, 0, 0, 0, 0, 0, i[6,0], i[7,0], i[8,0], 0 ,0 ,0]
        
        state_estimator = UKF(15, q, x_init, 0.0001*np.eye(15), 0.04, 15.0, 2.0, self.iterate_x)
        
        dt = 0.0
        for j in range(m):
                
            row = [i[0,j], i[1,j], i[2,j], i[3,j], i[4,j], i[5,j], i[6,j], i[7,j], i[8,j], i[9,j], i[10,j], i[11,j]]
            #print(row)        
            if j:
                dt = self.mtime[j]-self.mtime[j-1]
            
            # create an array for the data from each sensor
            x = state_estimator.get_state()
            bias = [-0.190006656546, -0.174740895383]
            gravity = 9.79531049538

            #Rotation matrix to align angle to that of the acceleration of the drone. Hences the gravity can be subtracted 
            R1 = self.eulerAnglesToRotationMatrix([0, 0, 2*x[11] + np.pi/2])
            a = np.matmul(R1, np.array([x[9], x[10], x[11]]))

            #Correct for acceleration shift for the orientation of the drone and bias for x and y
            R2 = self.eulerAnglesToRotationMatrix([x[10], x[9], x[11]])
            
            
            acc = np.matmul(R2, np.array([row[3], row[4], 0]))
            acc_bias = np.matmul(R2, np.array([bias[0], bias[1], 0]))
            """
            corrected_acc_x = acc[0] - acc_bias[0] + np.sin(a[0])*gravity
            corrected_acc_y = acc[1] - acc_bias[1] + np.sin(a[1])*gravity
            corrected_acc_z = row[5] - gravity
            
            """
            gravity = 9.79531049538
            
            gamma = [gravity*np.sin(a[0]), gravity*np.sin(a[1]), gravity*np.cos(a[0])*np.cos(a[1])]
            acc = np.matmul(R2, np.array([row[3], row[4], 0]))
            #acc_bias = np.matmul(R2, np.array([bias[0], bias[1], 0]))

            corrected_acc_x = acc[0] - acc_bias[0] #+ gamma[0]
            corrected_acc_y = acc[1] - acc_bias[1] #+ gamma[1]
            corrected_acc_z = row[5] - gamma[2]
            
            #Rotation matrix to align gyro velocity to the orientation of the world (marker)
            R3 = self.eulerAnglesToRotationMatrix([0, 0, x[11]-np.pi])
            corrected_gyro = np.matmul(R3, np.array([row[10], row[9], row[11]]))
 
            vision_x = row[0]
            vision_y = row[1]
            vision_z = row[2]
            vision_roll = row[6]
            vision_pitch = row[7]
            vision_yaw = row[8]

            imu_accX = corrected_acc_x #row[3]
            imu_accY = corrected_acc_y #row[4]
            imu_accZ = corrected_acc_z #row[5]
            imu_rollV = corrected_gyro[0] #row[9]
            imu_pitchV = corrected_gyro[1] #row[10]
            imu_yawV = corrected_gyro[2] #row[11]

            vision_data_pos = np.array([vision_x, vision_y, vision_z])
            vision_data_ori = np.array([vision_roll, vision_pitch, vision_yaw])
            imu_data_acc = np.array([imu_accX, imu_accY, imu_accZ])
            imu_data_gyro_v = np.array([imu_rollV, imu_pitchV, imu_yawV])

            # prediction is pretty simple
            state_estimator.predict(dt)

            # remember that the updated states should be zero-indexed
            # the states should also be in the order of the noise and data matrices
   
            if not self.old_x == row[0]:
                state_estimator.update([0, 1, 2], vision_data_pos, r_vision_pos)
                state_estimator.update([9, 10, 11], vision_data_ori, r_vision_ori)
                
            state_estimator.update([6, 7, 8], imu_data_acc, r_imu_acc)
            state_estimator.update([12, 13, 14], imu_data_gyro_v, r_imu_gyro_v)

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
            
            self.gyro_x = self.gyro_x + row[9]*dt
            self.gyro_y = self.gyro_y + row[10]*dt
            self.gyro_z = self.gyro_z + row[11]*dt
            
            self.x15.append(float(np.rad2deg(self.gyro_x)))
            self.x16.append(float(np.rad2deg(self.gyro_y)))
            self.x17.append(float(np.rad2deg(self.gyro_z)))

            #self.test = self.test + 1
            #print(self.test)
            
            
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

    def plot_ground_truth(self, x, y):

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

        plt.scatter(x, y, s=5, label='GPS Measurements', c='r')

        # Start/Goal
        plt.scatter(x[0], y[0], s=60, label='Start', c='g')
        plt.scatter(x[-1], y[-1], s=60, label='Goal', c='r')

        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('position_ground_truth')
        plt.legend(loc='best')
        plt.axis('equal')

        plt.savefig('position_ground_truth.png')

    def create_ground_truth(self):
        
        route = [[-3.65,-1.10,1.5], [-3.65,-4.25,1.5], [-3.65,-4.25,1.5], [-7.1,-4.25,1.5], [-7.1,-4.25,1.5], [-7.1,-7.1,1.5], [-7.1,-7.1, 1.0,]]
        waypoints = 335
        
        int_points = waypoints/(len(route)-1)
        if int(int_points) < int_points:
            int_points = int_points + 1
        
        #print(int_points)
        
        for i in range(1, len(route)):
            point1 = route[i-1]
            point2 = route[i]
    
            new_points = np.linspace(point1[0],point2[0],int(int_points))
            for i in new_points:    
                self.new_route_x.append(i)
        
            new_points = np.linspace(point1[1],point2[1],int(int_points))
            for i in new_points:    
                self.new_route_y.append(i)
            
            new_points = np.linspace(point1[2],point2[2],int(int_points))
            for i in new_points:    
                self.new_route_z.append(i)
        
        while len(self.new_route_x) > waypoints:
            self.new_route_x.pop()
            self.new_route_y.pop()
            self.new_route_z.pop()
        
        #print(len(self.new_route_x))
        #self.plot_ground_truth(self.new_route_x, self.new_route_y)
        
    def load_chromosome(self,file_name):
        chromosome = []
        with open(file_name,"r") as ga_file:
            for line in ga_file:
                items = line.split(' ')
                for item in items:
                    chromosome.append(item)
        return chromosome
  
        
if __name__ == "__main__":

    ukf = ukf()
    #ukf.create_ground_truth()
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
    ukf.plot_state('Angular velocity [IMU]', 'Time [s]', r'Velocity [$(\frac{r}{s})$]', 'Velocity.png', ukf.mtime, ys)

    ukf.plot_position()
    
    labels = ['Roll','Pitch', 'Yaw']
    ys = [ukf.x15, ukf.x16, ukf.x17]
    ukf.plot_state('Angle [IMU and vision fusion]', 'Time [s]', r'Angle [Degress]', 'Orientation_gyro.png', ukf.mtime, ys)
    
