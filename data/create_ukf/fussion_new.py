from ukf import UKF
import csv
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import


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
        
        self.mbaro = np.array([item[12] for item in self.data])
        
        self.mtime = np.array([item[13] for item in self.data])
        
        self.mvision_seq = np.array([item[14] for item in self.data])
        self.mimu_seq = np.array([item[15] for item in self.data])
        self.mbaro_seq = np.array([item[16] for item in self.data])

        self.ground_truth_mpsi = np.array([item[17] for item in self.data])
        self.ground_truth_mphi = np.array([item[18] for item in self.data])
        self.ground_truth_mtheta = np.array([item[19] for item in self.data])
        self.ground_truth_alt = np.array([item[20] for item in self.data])
        
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

        self.x18 = [] #ground truth
        self.x19 = [] #ground truth
        self.x20 = [] #ground truth
        
        self.x21 = [] #baro
        
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.test = 0

        self.new_route_x = []
        self.new_route_y = []
        self.new_route_z = []

        self.create_ground_truth()
        
        self.prev_acc_x = 0.0
        self.prev_acc_y = 0.0
        self.prev_acc_z = 0.0

        self.prev_acc_x_new = 0.0
        self.prev_acc_y_new = 0.0
        self.prev_acc_z_new = 0.0
        
        self.prev_vel_x = 0.0
        self.prev_vel_y = 0.0
        self.prev_vel_z = 0.0

        self.vision_seq = 0
        self.imu_seq = 0
        self.baro_seq = 0

        self.bias = [-0.190006656546, -0.174740895383]
        self.gravity = 9.79531049538

        self.activate_velocity_attenuation_x = [0, 0, 0]
        self.activate_velocity_attenuation_y = [0, 0, 0]
        self.activate_velocity_attenuation_z = [0, 0, 0]
        self.attenuation_vel_x = 1.0
        self.attenuation_vel_y = 1.0
        self.attenuation_vel_z = 1.0
        self.z_n = False
        self.z_p = False

        self.shift = []
        self.start = 0
        self.end = 0
        self.last_step = 0
        self.counter = 0
        
        self.inversions_x = []
        self.last_step_x = 0
        self.start_x = 0
        
        self.inversions_y = []
        self.last_step_y = 0
        self.start_y = 0
        
        self.inversions_z = []
        self.last_step_z = 0
        self.start_z = 0

        #Used in the GA (chromosome)
        self.covariance = [0.05, 0.05, 0.05, #Process noise x, y and z (pos)
                           0.5, .5, 0.5, #Process noise x, y and z (vel)
                           0.2, 0.2, 0.5, #Process noise x, y and z (acc)
                           1.5, 1.5, 0.05, #Process noise roll, pitch and yaw (angle)
                           0.9, 0.9, 0.5, #Process noise x, y and z (angle rate)
                           2.3, 2.3, 2.3, #Measurement noise x, y and z (pos)
                           0.03, 0.03, 2.08, #Measurement noise x, y and z (acc)
                           0.05, 0.05, 0.05, #Measurement noise x, y and z (angle)
                           1.5, 1.5, 0.5] #Measurement noise x, y and z (abgle rate)
        
        # From best individual using GA 
        #self.covariance = self.load_chromosome('ga_data/best_chromosome.txt')
    
    def read_data(self, file_name):
        init = True
        seq = 0
        with open(file_name,"r") as text_file:
            for line in text_file:
                items = line.split(' ')
                items[-1] = items[-1].strip()
                str_to_float = [float(item) for item in items]
                if init and not str_to_float[15] == seq:
                    init = False
                if not init:
                    error = False
                    for i in str_to_float:
                        if math.isnan(i):
                            error = True

                    if not error:
                        #print(str_to_float)
                        self.data.append(str_to_float)
                seq = str_to_float[15]

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
        
        ret = np.zeros(len(x))
        
        ret[0] = x[0] + 0.6*x[3]*dt
        ret[1] = x[1] + 0.6*x[4]*dt
        ret[2] = x[2]  # x[5]*dt
        ret[3] = x[6] #x[3]*self.attenuation_vel_x + x[6]*dt
        ret[4] = x[7] #x[4]*self.attenuation_vel_y + x[7]*dt
        ret[5] = x[5]*self.attenuation_vel_z + x[8]*dt
        ret[6] = x[6] 
        ret[7] = x[7] 
        ret[8] = x[8]
        ret[9] = x[9] + x[13]*dt # #+ np.arctan2(x[6], x[8]+off)
        ret[10] = x[10] + x[12]*dt #+ np.arctan2(-x[7], np.sqrt(x[6]**2 + x[8]**2)+off)
        ret[11] = x[11] + x[14]*dt
        ret[12] = x[12]
        ret[13] = x[13]
        ret[14] = x[14]
        ret[15] = x[15]
        return ret
    
    def main(self):
        
        np.set_printoptions(precision=3)
        
        # Process Noise
        q = np.eye(16)
        q[0][0] = 0.5 #x
        q[1][1] = 0.5 #y
        q[2][2] = 0.05 #z
        q[3][3] = 0.05 #vel x
        q[4][4] = 0.05 #vel y
        q[5][5] = 0.5 #vel z 
        q[6][6] = 0.05 #acc x
        q[7][7] = 0.05 #acc y
        q[8][8] = 0.5 #acc z
        q[9][9] = 0.05 #roll
        q[10][10] = 0.05#pitch
        q[11][11] = 0.05 #yaw
        q[12][12] = 0.05 #roll rate
        q[13][13] = 0.05 #pitch rate
        q[14][14] = 0.05 #yaw rate
        q[15][15] = 0.5 #baro rate
        
        
        # Create measurement noise covariance matrices
        r_imu_acc = np.zeros([3, 3])
        r_imu_gyro_v = np.zeros([3, 3])
        r_imu_acc[0][0] = 0.05 #acc x
        r_imu_acc[1][1] = 0.05 #acc y
        r_imu_acc[2][2] = 10.5 #acc z
        r_imu_gyro_v[0][0] = 0.5 #gyro roll
        r_imu_gyro_v[1][1] = 0.5 #gyro pitch
        r_imu_gyro_v[2][2] = 0.5 #gyro yaw
        
        r_vision_pos = np.zeros([3, 3])
        r_vision_ori = np.zeros([3, 3])
        r_vision_pos[0][0] = 0.5 #x
        r_vision_pos[1][1] = 0.5 #y
        r_vision_pos[2][2] = 0.05 #z
        r_vision_ori[0][0] = 0.5 #roll rate
        r_vision_ori[1][1] = 0.5 #pitch rate
        r_vision_ori[2][2] = 0.5 #yaw rate
        
        r_baro = np.zeros([1,1])
        r_baro[0][0] = 5.2

        r_baro_offset = np.zeros([1,1])
        r_baro_offset[0][0] = 0.02
        
        # pass all the parameters into the UKF!
        # number of state variables, process noise, initial state, initial coariance, three tuning paramters, and the iterate function
        measurements = np.vstack((self.mx, self.my, self.mz, self.macc_x, self.macc_y, self.macc_z, self.mpsi, self.mphi, self.mtheta, self.mgyro_x, self.mgyro_y, self.mgyro_z, 
            self.mvision_seq, self.mimu_seq, self.ground_truth_mpsi, self.ground_truth_mphi, self.ground_truth_mtheta, self.mbaro, self.mbaro_seq))
        m = measurements.shape[1]
        #print(measurements.shape)
        i = measurements
        #print(m)
        x_init = [i[0,0], i[1,0], i[2,0], 0, 0, 0, 0, 0, 0, i[6,0], i[7,0], i[8,0], 0 ,0 ,0, 0]
        
        state_estimator = UKF(16, q, x_init, 0.0001*np.eye(16), 0.04, 15.0, 2.0, self.iterate_x)
        time =0
        dt = 0.0
        for j in range(m):
                
            row = [i[0,j], i[1,j], i[2,j], i[3,j], i[4,j], i[5,j], i[6,j], i[7,j], i[8,j], i[9,j], i[10,j], i[11,j], i[12,j], i[13,j], i[14,j], i[15,j], i[16,j], i[17,j], i[18,j]]
            #print(row)        
            if j:
                dt = self.mtime[j]-self.mtime[j-1]
            
            
            # create an array for the data from each sensor
            x = state_estimator.get_state()
            bias = [-0.190006656546, -0.174740895383]
            gravity = 9.79531049538

            off = 0.000000001


            #Rotation matrix to align angle to that of the acceleration of the drone. Hences the gravity can be subtracted
            R1 = self.eulerAnglesToRotationMatrix([0, 0, 2*x[11] + np.pi/2])
            a = np.matmul(R1, np.array([x[9], x[10], x[11]]))

            pitch = row[15]
            roll = row[14]
            gravity_x =  np.sin(pitch)*gravity # row[15] 
            gravity_y =  -np.sin(roll)*gravity

            acc_x = row[3] #+ gravity_x  # + np.sin(np.arctan2(-row[3], np.sqrt(row[4]**2 + row[5]**2)+off))*self.gravity
            acc_y = row[4] #+ gravity_y  # - np.sin(np.arctan2(row[4], row[5]+off))*self.gravity            
            
            #Correct for acceleration shift for the orientation of the drone and bias for x and y
            R2 = self.eulerAnglesToRotationMatrix([x[10], x[9], x[11]])            
            acc = np.matmul(R2, np.array([acc_x, acc_y, 0]))
            acc_bias = np.matmul(R2, np.array([bias[0], bias[1], 0]))

            corrected_acc_x = -acc[0]
            corrected_acc_y = -acc[1]  
            corrected_acc_z = row[5] - gravity #*np.cos(x[9])*np.cos(x[10])
            self.acc_gravity = row[5]
            
            #Mechanical Filtering Window
            
            if corrected_acc_x < 0.10 and corrected_acc_x > -0.10:
                corrected_acc_x = 0

            if corrected_acc_y < 0.10 and corrected_acc_y > -0.10:
                corrected_acc_y = 0

            if corrected_acc_z < 0.25 and corrected_acc_z > -0.25:
                corrected_acc_z = 0
            
            time = time + dt
            
            """
            #X attenuation 
            zero_acc = False
            sign_shift = False
            if x[6] < 0.005 and x[6] > -0.005:
                zero_acc = True
            
            if not self.start_x and not zero_acc:
                self.start_x = True
                self.last_step_x = np.sign(x[6])
                print(self.last_step_x)
                print('start_x' + " " + str(time))
                self.inversions_x.append(self.last_step_x)

            if self.start_x and zero_acc:
                self.start_x = False
                print('End x' + " " + str(time))
            
            elif self.start_x and np.sign(x[6]) and self.last_step_x and not (self.last_step_x == np.sign(x[6])):
                self.start_x = False
                print('End' + " " + str(time))

            if len(self.inversions_x) > 1:
                if not self.inversions_x[-1] == self.inversions_x[-2]:
                    self.attenuation_vel_x = 0.95
                    self.inversions_x = []
            elif len(self.inversions_x) == 1:
                self.attenuation_vel_x = 1.00
            
            #Y attenuation 
            zero_acc = False
            sign_shift = False
            if x[7] < 0.005 and x[7] > -0.005:
                zero_acc = True
            
            if not self.start_y and not zero_acc:
                self.start_y = True
                self.last_step_y = np.sign(x[7])
                print(self.last_step_y)
                print('start y' + " " + str(time))
                self.inversions_y.append(self.last_step_y)

            if self.start_y and zero_acc:
                self.start_y = False
                print('End y' + " " + str(time))
            
            elif self.start_y and np.sign(x[7]) and self.last_step_y and not (self.last_step_y == np.sign(x[7])):
                self.start_y = False
                print('End y' + " " + str(time))

            if len(self.inversions_y) > 1:
                if not self.inversions_y[-1] == self.inversions_y[-2]:
                    self.attenuation_vel_y = 0.95
                    self.inversions_y = []
            elif len(self.inversions_y) == 1:
                self.attenuation_vel_y = 1.00
            
            """
            #Z attenuation 
            zero_acc = False
            sign_shift = False
            if x[8] < 0.005 and x[8] > -0.005:
                zero_acc = True
            
            if not self.start_z and not zero_acc:
                self.start_z = True
                self.last_step_z = np.sign(x[8])
                print(self.last_step_z)
                print('start z' + " " + str(time))
                self.inversions_z.append(self.last_step_z)

            if self.start_z and zero_acc:
                self.start_z = False
                print('End z' + " " + str(time))
            
            elif self.start_z and np.sign(x[8]) and self.last_step_z and not (self.last_step_z == np.sign(x[8])):
                self.start_z = False
                print('End z' + " " + str(time))

            if len(self.inversions_z) > 1:
                if not self.inversions_z[-1] == self.inversions_z[-2]:
                    self.attenuation_vel_z = 0.95
                    self.inversions_z = []
            elif len(self.inversions_z) == 1:
                self.attenuation_vel_z = 1.00
 
            #Rotation matrix to align gyro velocity to the orientation of the world (marker)
            R3 = self.eulerAnglesToRotationMatrix([0, 0, x[11]-np.pi])
            corrected_gyro = np.matmul(R3, np.array([row[10], row[9], row[11]]))
            
            vision_x = row[0]
            vision_y = row[1]
            vision_z = row[2]
            vision_roll = row[6]
            vision_pitch = row[7]
            vision_yaw = row[8]

            imu_accX =  corrected_acc_x
            imu_accY = corrected_acc_y
            imu_accZ = corrected_acc_z
            imu_rollV = corrected_gyro[0]
            imu_pitchV =corrected_gyro[1]
            imu_yawV = corrected_gyro[2]

            vision_data_pos = np.array([vision_x, vision_y, vision_z])
            vision_data_ori = np.array([vision_roll, vision_pitch, vision_yaw])
            imu_data_acc = np.array([imu_accX, imu_accY, imu_accZ])
            imu_data_gyro_v = np.array([imu_rollV, imu_pitchV, imu_yawV])
            
            baro_data = np.array([row[17]])
            baro_offset = vision_z - baro_data

            # prediction is pretty simple
            state_estimator.predict(dt, row)

            if not self.vision_seq == row[12]:
                state_estimator.update([0, 1, 2], vision_data_pos, r_vision_pos)
                state_estimator.update([9, 10, 11], vision_data_ori, r_vision_ori)
                state_estimator.update([15], baro_offset, r_baro_offset)
                self.vision_seq = row[12]
            
            if not self.imu_seq == row[13]:
                state_estimator.update([6, 7, 8], imu_data_acc, r_imu_acc)
                state_estimator.update([12, 13, 14], imu_data_gyro_v, r_imu_gyro_v)
                self.imu_seq = row[13]
            
            if not self.baro_seq == row[18]:
                state_estimator.update([2], baro_data + x[15], r_baro)
                self.baro_seq = row[18]
            
            self.old_x = row[0]

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
            self.x12.append(float(np.rad2deg(x[12])))
            self.x13.append(float(np.rad2deg(x[13])))
            self.x14.append(float(np.rad2deg(x[14])))
            
            self.gyro_x = self.gyro_x + row[9]*dt
            self.gyro_y = self.gyro_y + row[10]*dt
            self.gyro_z = self.gyro_z + row[11]*dt
            
            off = 0.000000001
            self.x15.append(float(np.rad2deg(self.gyro_x)))
            self.x16.append(float(np.rad2deg(self.gyro_y)))
            self.x17.append(float(np.rad2deg(self.gyro_z)))

            self.x18.append(float(np.rad2deg(row[14])))
            self.x19.append(float(np.rad2deg(row[15])))
            self.x20.append(float(np.rad2deg(row[16])))
            
            self.x21.append(float(baro_data+x[15]))
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
        plt.title('Position_xy')
        plt.legend(loc='best')
        plt.axis('equal')

        plt.savefig('Positionxy.png')
    def plot_position_2d(self):

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

        plt.scatter(self.mtime, self.x21, s=2, label='Corrected altimeter')
        plt.scatter(self.mtime, self.mbaro, s=2, label='Altimeter')
        plt.scatter(self.mtime, self.mz, s=2, label='Vision altitude')
        plt.scatter(self.mtime, self.ground_truth_alt, s=2, label='Ground truth')
        plt.scatter(self.mtime, self.x2, s=2, label='EKF altitude')

        # Start/Goal
        print(len(self.mtime))
        print(len(self.x2))
        #plt.scatter(self.mtime, self.mz[0], s=60, label='Start', c='g')
        #plt.scatter(self.mtime, self.mz[-1], s=60, label='Goal', c='r')
        
        plt.xlabel('Z [m]')
        plt.title('Positionz')
        plt.legend(loc='best')
        #plt.axis('equal')

        plt.ylim(ymax = 5, ymin = 0)
        plt.savefig('Positionz.png')

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
    ys = [ukf.x0, ukf.x1, ukf.x2]
    ukf.plot_state('Position', 'Time [s]', r'Position [$m$]', 'position.png', ukf.mtime, ys)

    labels = ['Velocity in x','Velocity in y','Velocity in z']
    ys = [ukf.x3, ukf.x4, ukf.x5]
    ukf.plot_state('Velocity [IMU]', 'Time [s]', r'Velocity [$(\frac{m}{s})$]', 'velocity.png', ukf.mtime, ys)
    
    labels = ['acc_x','acc_y','acc_z']
    ys = [ukf.x6, ukf.x7, ukf.x8]
    ukf.plot_state('Acceleration [IMU]', 'Time [s]', r'Acceleration [$(\frac{m}{s^2})$]', 'acceleration.png', ukf.mtime, ys)

    labels = ['Roll','Pitch']
    ys = [ukf.x9, ukf.x10, ukf.x11]
    ukf.plot_state('Angle [IMU and vision fusion]', 'Time [s]', r'Angle [Degress]', 'orientation.png', ukf.mtime, ys)

    labels = ['x','y','z']
    ys = [ukf.x12, ukf.x13, ukf.x14]
    ukf.plot_state('Angular velocity [IMU]', 'Time [s]', r'Angular velocity [$(\frac{r}{s})$]', 'angular_velocity.png', ukf.mtime, ys)
    
    ukf.plot_position()
    ukf.plot_position_2d()
    
    labels = ['Roll','Pitch', 'Yaw']
    ys = [ukf.x15, ukf.x16, ukf.x17]
    ukf.plot_state('Angle [IMU and vision fusion]', 'Time [s]', r'Angle [Degress]', 'orientation_gyro.png', ukf.mtime, ys)
    
    labels = ['Roll','Pitch', 'Yaw']
    ys = [ukf.x18, ukf.x19, ukf.x20]
    ukf.plot_state('Angle [Ground truth]', 'Time [s]', r'Angle [Degress]', 'orientation_ground_truth.png', ukf.mtime, ys)
