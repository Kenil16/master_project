import numpy as np
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
from scipy.stats import norm
from sympy import Symbol, symbols, Matrix, sin, cos
from sympy.interactive import printing
from mpl_toolkits.axes_grid1 import make_axes_locatable
printing.init_printing(use_latex=True)

class ekf():

    def __init__(self):

        self.data = [] 
        self.read_data('test1.txt')
        
        self.x = np.array([item[0] for item in self.data])
        self.acc_x = np.array([item[1] for item in self.data])
        
        self.y = np.array([item[2] for item in self.data])
        self.acc_y = np.array([item[3] for item in self.data])
        
        self.z = np.array([item[4] for item in self.data])
        self.acc_z = np.array([item[5] for item in self.data])
        
        self.alpha = np.array([item[6] for item in self.data])
        self.gyro_x = np.array([item[7] for item in self.data])
        
        self.beta = np.array([item[8] for item in self.data])
        self.gyro_y = np.array([item[8] for item in self.data])
        
        self.gamma = np.array([item[9] for item in self.data])
        self.gyro_z = np.array([item[10] for item in self.data])
        
        self.time = np.array([item[11] for item in self.data])
        
        """
        self.numstates = 12 #States
        self.dt = 1.0/50.0 # Sample Rate of the Measurements is 50Hz
        self.dtVision = 1.0/10.0 # Sample Rate of GPS is 10Hz
        xs, ys, zs, dts, xs, ys, lats, lons = symbols('x y z \dotx \doty \dot \alpha \beta \gamma ')
        
        self.gs = Matrix([[xs+(vs/dpsis)*(sin(psis+dpsis*dts)-sin(psis))],
            [ys+(vs/dpsis)*(-cos(psis+dpsis*dts)+cos(psis))],
            [psis+dpsis*dts],
            [vs],
            [dpsis]])
        
        state = Matrix([xs,ys,psis,vs,dpsis])
        """
    def read_data(self, file_name):
        
        with open(file_name,"r") as text_file:
            self.data = []
            for line in text_file:
                items = line.split(' ')
                items[-1] = items[-1].strip()
                str_to_float = [float(item) for item in items]
                self.data.append(str_to_float)

    def plot_vision_pos(self):
        
        fig = plt.figure(figsize=(9,4))

        # EKF State
        #plt.quiver(x0,x1,np.cos(x2), np.sin(x2), color='#94C600', units='xy', width=0.01, scale=0.2, label='Driving Direction')
        plt.plot(self.x, self.y, label='Vision position', linewidth=5, alpha=0.8)

        # Measurements
        plt.scatter(self.x,self.y, s=50, label='Vision measurements')
        #cbar=plt.colorbar(ticks=np.arange(20))
        #cbar.ax.set_ylabel(u'EPE', rotation=270)
        #cbar.ax.set_xlabel(u'm')

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

if __name__ == "__main__":

    ekf = ekf()
    ekf.plot_vision_pos()
    ekf.plot()

