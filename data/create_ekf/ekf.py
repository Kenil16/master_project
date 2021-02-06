import numpy as np
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
from scipy.stats import norm
from sympy import Symbol, symbols, Matrix, sin, cos
from sympy.interactive import printing
from mpl_toolkits.axes_grid1 import make_axes_locatable
printing.init_printing(use_latex=True)
#from tf.transformations import*

class ekf():

    def __init__(self):

        self.data = [] 
        self.read_data('test12.txt')
        self.mx = np.array([item[0] for item in self.data])
        self.my = np.array([item[1] for item in self.data])
        self.mz = np.array([item[2] for item in self.data])
        
        self.macc_x = np.array([item[3] for item in self.data])
        self.macc_y = np.array([item[4] for item in self.data])
        self.macc_z = np.array([item[5] for item in self.data])
        
        self.corr_acc_x = []
        self.corr_acc_y = []
        self.corr_acc_z = []

        self.mpsi = np.array([item[6] for item in self.data])
        self.mphi = np.array([item[7] for item in self.data])
        self.mtheta = np.array([item[8] for item in self.data])
        
        self.mgyro_x = np.array([item[9] for item in self.data])
        self.mgyro_y = np.array([item[10] for item in self.data])
        self.mgyro_z = np.array([item[11] for item in self.data])
        
        self.mtime = np.array([item[12] for item in self.data])

        #Just need one of the x, y, z, psi, phi and theta to see if a new vision estimation has arrived 
        self.old_x = 0.0
        
        self.numstates = 15 #States
        self.dt = 1.0/50.0 # Sample Rate of the Measurements is 50Hz
        self.dtVision = 1.0/10.0 # Sample Rate of GPS is 10Hz
 
        
        xs, ys, zs, xv, yv, zv, xa, ya, za, psi, phi, theta, psi_v, phi_v, theta_v, dt = symbols('x y z x_v, y_v, z_v, x_a y_a a_z psi phi theta psi_v phi_v theta_v T')
        
        self.gs = Matrix([
            [xs + xv*dt + 0.5*xa*dt**2],
            [ys + yv*dt + 0.5*ya*dt**2],
            [zs + zv*dt + 0.5*za*dt**2],
            [xv + xa*dt],
            [yv + ya*dt],
            [zv + za*dt],
            [xa],
            [ya],
            [za],
            [psi + cos(theta+np.pi)*(psi_v*dt) - sin(theta+np.pi)*(phi_v*dt)],
            [phi + sin(theta+np.pi)*(psi_v*dt) + cos(theta+np.pi)*(phi_v*dt)],
            [theta + theta_v*dt],
            [psi_v],
            [phi_v],
            [theta_v]])
        
        
        #self.state = Matrix([xs, ys, zs, xv, yv, zv, xa, ya, za, psi, phi, theta, psi_v, phi_v, theta_v])
        self.P = np.diag([1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0])
        self.Q = np.diag([1.0**2, 1.0**2, 0.5**2, 7.5**2, 7.5**2, 1.0**2, 0.5**2, 0.5**2, 0.5**2, 0.2**2, 0.2**2, 0.2**2, 0.2**2, 0.2**2, 0.2**2]) 
        self.R = np.diag([15.5**2, 15.5**2, 0.5**2, 2.0**2, 2.0**2, 1.0**2, 0.1**2, 0.1**2, 0.5**2, 0.02**2, 0.02**2, 0.02**2])


        self.hs = Matrix([
            [xs],
            [ys],
            [zs],
            [xa], 
            [ya], 
            [za],
            [psi],
            [phi],
            [theta],
            [psi_v], 
            [phi_v], 
            [theta_v]])

        #self.JHs=self.hs.jacobian(self.state)
        
        self.x = np.matrix([[0, 0, 0, 0, 0, 0 ,0, 0, 0, 0 ,0 ,0]]).T
        self.I = np.eye(self.numstates)

        #print(self.gs.jacobian(self.state))
        #print(self.JHs)

        
        # Preallocation for Plotting
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
    
    def read_data(self, file_name):
        
        with open(file_name,"r") as text_file:
            self.data = []
            for line in text_file:
                items = line.split(' ')
                items[-1] = items[-1].strip()
                str_to_float = [float(item) for item in items]
                if not str_to_float[0] == 0:
                    self.data.append(str_to_float)

    def plot_vision_pos(self):
        
        fig = plt.figure(figsize=(9,4))

        plt.plot(self.x, self.y, label='Vision position', linewidth=5, alpha=0.8)
        plt.scatter(self.x,self.y, s=50, label='Vision measurements')
        
        plt.xlabel('X [m]')
        #plt.xlim(80, 120)
        plt.ylabel('Y [m]')
        #plt.ylim(160, 180)
        plt.title('Position')
        plt.legend(loc='best');
        plt.savefig('EKF-Position.png', dpi=150)

    def plot(self):

        P = np.diag([1000.0, 1000.0, 1000.0, 1000.0, 1000.0])
        print(P, P.shape)
        fig = plt.figure()#figsize=(5, 5))
        plt.title('Initial Covariance Matrix $P$')
        ylocs, ylabels = plt.yticks()
        # set the locations of the yticks
        plt.yticks(np.arange(6))
        # set the locations and labels of the yticks
        plt.yticks(np.arange(5),('$x$', '$y$', '$\psi$', '$v$', '$\dot \psi$'), fontsize=22)

        xlocs, xlabels = plt.xticks()
        # set the locations of the yticks
        plt.xticks(np.arange(6))
        # set the locations and labels of the yticks
        plt.xticks(np.arange(5),('$x$', '$y$', '$\psi$', '$v$', '$\dot \psi$'), fontsize=22)

        plt.xlim([-0.5,4.5])
        plt.ylim([4.5, -0.5])
        
        plt.minorticks_on()
        plt.grid(b=True, which='major', color='#666666', linestyle='-')
        plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
        im = plt.imshow(P, interpolation="none", cmap=plt.get_cmap('binary'))
        
        divider = make_axes_locatable(plt.gca())
        cax = divider.append_axes("right", "5%", pad="3%")
        plt.colorbar(im, cax=cax)
        #plt.tight_layout()
        
        plt.gca().patch.set_facecolor('0.8')
        plt.savefig('EKF-Position.png', dpi=150)


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

        plt.savefig('Extended-Kalman-Filter-CTRA-Position.png')


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

    def eulerAnglesToRotationMatrixYaw(self,theta):
	
        R_z = np.array([[np.cos(theta),    -np.sin(theta),    0],
                        [np.sin(theta),    np.cos(theta),     0],
                        [0,                     0,            1]])
        
        return R_z
    
    def eulerAnglesToRotationMatrix(self,theta):
	
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

if __name__ == "__main__":

    ekf = ekf()
    #ekf.plot_vision_pos()
    #ekf.plot()

    measurements = np.vstack((ekf.mx, ekf.my, ekf.mz, ekf.macc_x, ekf.macc_y, ekf.macc_z, ekf.mpsi, ekf.mphi, ekf.mtheta, ekf.mgyro_x, ekf.mgyro_y, ekf.mgyro_z))
    m = measurements.shape[1]
    print(measurements.shape)
    i = measurements
    x = np.matrix([[i[0,0], i[1,0], i[2,0], 0.0, 0.0, 0.0, i[3,0], i[4,0], i[5,0], i[6,0], i[7,0], i[8,0], i[9,0], i[10,0], i[11,0]]]).T
    
    for filterstep in range(m):

        if filterstep:
            ekf.dt = ekf.mtime[filterstep]-ekf.mtime[filterstep-1]
        print(ekf.dt)

        #Time Update (Prediction)

        #Rotation matrix to align angle to that of the acceleration of the drone. Hences the gravity can be subtracted 
        R1 = ekf.eulerAnglesToRotationMatrix([0, 0, x[11,0]+np.pi/2])
        R2 = ekf.eulerAnglesToRotationMatrix([0, 0, x[11,0]])
        a = np.matmul(np.matmul(R1,R2), np.array([x[9,0], x[10,0], x[11,0]]))
        
        #Correct for acceleration shift for the orientation of the drone and bias for x and y
        R = ekf.eulerAnglesToRotationMatrixYaw(x[11,0])
        acc = np.matmul(R, np.array([x[6,0], x[7,0], 0]))
        acc_bias = np.matmul(R, np.array([-0.190006656546, -0.174740895383, 0]))
        
        corrected_acc_x = acc[0] - acc_bias[0] + np.sin(a[0]*9.79531049538)
        corrected_acc_y = acc[1] - acc_bias[1] + np.sin(a[1]*9.79531049538)
        corrected_acc_z = x[8,0] - 9.79531049538
        
        ekf.corr_acc_x.append(corrected_acc_x)
        ekf.corr_acc_y.append(corrected_acc_y)
        ekf.corr_acc_z.append(corrected_acc_z)
        
        x[0] = x[0] + x[3]*ekf.dt #+ 0.5*corrected_acc_x*ekf.dt**2 #(np.cos(x[8]+np.pi)*(0.5*x[3]*ekf.dt) - np.sin(x[8]+np.pi)*(0.5*x[4]*ekf.dt))
        x[1] = x[1] + x[4]*ekf.dt #+ 0.5*corrected_acc_y*ekf.dt**2 #(np.sin(x[8]+np.pi)*(0.5*x[3]*ekf.dt) + np.cos(x[8]+np.pi)*(0.5*x[4]*ekf.dt))
        x[2] = x[2] + 0.5*x[5]*ekf.dt #+ 0.5*corrected_acc_z*ekf.dt**2
        x[3] = x[3] + corrected_acc_x*ekf.dt
        x[4] = x[4] + corrected_acc_y*ekf.dt
        x[5] = x[5] + corrected_acc_z*ekf.dt
        x[6] = x[6]
        x[7] = x[7]
        x[8] = x[8]
        x[9] = x[9] - (np.cos(x[11,0])*x[12,0]*ekf.dt + np.sin(x[11,0])*x[13,0]*ekf.dt) # x[12]*ekf.dt
        x[10] = x[10] - (-np.sin(x[11,0])*x[12,0]*ekf.dt + np.cos(x[11,0])*x[13,0]*ekf.dt) # x[13]*ekf.dt
        x[11] = x[11] + x[14]*ekf.dt
        x[12] = x[12]
        x[13] = x[13]
        x[14] = x[14]
        #dstate.append(1)

        #print((np.sin(x[8]+np.pi)*(0.5*x[3]*ekf.dt) + np.cos(x[8]+np.pi)*(0.5*x[4]*ekf.dt)))
        print(x[10])

        # Calculate the Jacobian of the Dynamic Matrix A
        # see "Calculate the Jacobian of the Dynamic Matrix with respect to the state vector"
        
        a09_11 = float(-ekf.dt*x[12]*np.sin(x[11]) - ekf.dt*x[13]*np.cos(x[11])) 
        a09_12 = float(ekf.dt*np.cos(x[11])) 
        a09_13 = float(-ekf.dt*np.sin(x[11])) 
        
        a10_11 = float(-ekf.dt*x[12]*np.sin(x[11]) + ekf.dt*x[13]*np.cos(x[11])) 
        a10_12 = float(ekf.dt*np.sin(x[11])) 
        a10_13 = float(ekf.dt*np.cos(x[11])) 
        
        JA = np.matrix([[1.0, 0.0, 0.0, ekf.dt, 0.0, 0.0, 0.5*ekf.dt**2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0, ekf.dt, 0.0, 0.0, 0.5*ekf.dt**2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0, 0.0, 0.0, ekf.dt, 0.0, 0.0, 0.5*ekf.dt**2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, ekf.dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, ekf.dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, ekf.dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, a09_11, a09_12, a09_13, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, a10_11, a10_12, a10_13, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, ekf.dt],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

        #Error covariance
        ekf.P = JA*ekf.P*JA.T + ekf.Q

        # Measurement Update (Correction)
        
        # Measurement Function
        hx = np.matrix([[float(x[0])],
                        [float(x[1])],
                        [float(x[2])],
                        [float(x[6])],
                        [float(x[7])],
                        [float(x[8])],
                        [float(x[9])],
                        [float(x[10])],
                        [float(x[11])],
                        [float(x[12])],
                        [float(x[13])],
                        [float(x[14])]])
        
        if not ekf.old_x == measurements[0,filterstep]: # with 10Hz, every 5th step
            print('Vision update')
            ekf.JH = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
        else: # every other step
            print('NO vision')
            ekf.JH = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
            
        S = ekf.JH*ekf.P*ekf.JH.T + ekf.R
        K = (ekf.P*ekf.JH.T) * np.linalg.inv(S) #np.linalg.pinv(S)
        
        # Update the estimate via
        Z = measurements[:,filterstep].reshape(ekf.JH.shape[0],1)
        y = Z - (hx) # Innovation or Residual
        x = x + (K*y)

        # Update the error covariance
        ekf.P = (ekf.I - (K*ekf.JH))*ekf.P

        ekf.old_x = measurements[0,filterstep]

        # Save states for Plotting
        ekf.x0.append(float(x[0]))
        ekf.x1.append(float(x[1]))
        ekf.x2.append(float(x[2]))
        ekf.x3.append(float(x[3]))
        ekf.x4.append(float(x[4]))
        ekf.x5.append(float(x[5]))
        ekf.x9.append(float(np.rad2deg(x[9])))
        ekf.x10.append(float(np.rad2deg(x[10])))
        ekf.x11.append(float(np.rad2deg(x[11])))
    
    ekf.plot_position()
    
    labels = ['x','y','z']
    ys = [ekf.corr_acc_x, ekf.corr_acc_y, ekf.corr_acc_z]
    ekf.plot_state('Acceleration [IMU]', 'Time [s]', r'Acceleration [$(\frac{m}{s^2})$]', 'Acceleration.png', ekf.mtime, ys)
    
    labels = ['Velocity in x','Velocity in y','Velocity in z']
    ys = [ekf.x3, ekf.x4, ekf.x5]
    ekf.plot_state('Linear velocity [IMU]', 'Time [s]', r'Velocity [$(\frac{m}{s})$]', 'Linear velocity.png', ekf.mtime, ys)
    
    labels = ['Roll','Pitch']
    ys = [ekf.x9, ekf.x10, ekf.x11]
    ekf.plot_state('Angle [IMU and vision fusion]', 'Time [s]', r'Angle [Degress]', 'Orientation.png', ekf.mtime, ys)
    
    labels = ['x','y','z']
    ys = [ekf.mgyro_x, ekf.mgyro_y, ekf.mgyro_z]
    ekf.plot_state('Angular velocity [IMU]', 'Time [s]', r'Velocity [$(\frac{m}{s})$]', 'Velocity.png', ekf.mtime, ys)

    """
    labels = ['x','y','z']
    ys = [ekf.mx, ekf.my, ekf.mz]
    ekf.plot_state('Position [Vision]', 'Time [s]', r'Position [$(\frac{m}{s^2})$]', 'Position.png', ekf.mtime, ys)
    """
