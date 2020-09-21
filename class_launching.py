from class_drone import Drone
from class_order import Order
import matplotlib.pyplot as plt
import numpy as np


class LaunchingBase():
    def __init__(self, dict_setup):
        self.x = dict_setup["x_base"]
        self.y = dict_setup["y_base"]
        self.x_max = dict_setup["x_max"]
        self.y_max = dict_setup["y_max"]
        self.list_drone = [Drone(dict_setup) for _ in range(dict_setup["nb_drone"])]
        self.list_order = []
        self.list_order_flying = []
        self.list_order_done = []
        self.dt = dict_setup["dt"]
        self.time_wait_base = dict_setup["time_wait_base"]
        self.recharge_power = dict_setup["recharge_power"]
        self.energy_tot_consumed = dict_setup["cap_bat"]*dict_setup["nb_drone"]
        self.granularity_noise = dict_setup["granularity_noise"]

        plt.show()
        self.axes = plt.subplot(121)
        self.axes.set_xlim(0, self.x_max)
        self.axes.set_ylim(0, self.y_max)
        self.axes.plot(self.x, self.y, 'r+', markersize=40)
        img = plt.imread('lyon.PNG')
        self.axes.imshow(img, alpha=0.5, extent=[0, self.x_max, 0, self.y_max])
        self.line, = self.axes.plot([], [], '*')

        self.axes_noise = plt.subplot(122)
        self.axes_noise.set_xlim(0, self.x_max)
        self.axes_noise.set_ylim(0, self.y_max)
        self.line_noise = self.axes_noise.imshow(np.zeros((self.y_max//self.granularity_noise, self.x_max//self.granularity_noise)), extent=[0, self.x_max, 0, self.y_max], cmap=plt.cm.RdBu, vmin=0, vmax=80)
        plt.colorbar(self.line_noise)

    def launch_order(self):
        boolean = True
        while boolean and len(self.list_order) != 0:
            order = self.list_order[0]
            boolean = False
            for drone in self.list_drone:
                if drone.state == 'Base' and (drone.time_waiting_base is None or drone.time_waiting_base <= 0):
                    # Check if there is a free drone to begin the delivery
                    drone.begin_order(order)
                    del self.list_order[0]
                    self.list_order_flying.append(order)
                    boolean = True
                    break

    def new_order(self, dest_x, dest_y, weight):
        self.list_order.append(Order(dest_x, dest_y, weight))

    def check_arrived_order(self):
        for drone in self.list_drone:
            if drone.state == 'Back' and drone.order is not None:
                # The order is delivered
                ind_order = self.list_order_flying.index(drone.order)
                self.list_order_done.append(self.list_order_flying[ind_order])
                del self.list_order_flying[ind_order]
                drone.order = None
            elif drone.state == 'Back to Base':
                # Back to base, we calcul the time to recharge the drone
                drone.time_waiting_base = self.time_wait_base + (drone.cap_bat - drone.cap_bat_act)/self.recharge_power*3600
                self.energy_tot_consumed += drone.cap_bat - drone.cap_bat_act
                drone.cap_bat_act = drone.cap_bat
                drone.state = 'Base'

    def move_all(self):
        for drone in self.list_drone:
            drone.move(self.dt)

        for order_waiting in self.list_order:
            order_waiting.waiting_order(self.dt)

    def plot_drone(self, time):
        time = time%86400
        self.axes.set_title(f'{time//3600}h{(time%3600)//60}min{time%60}s \n {len(self.list_order)} orders waiting \n {len(self.list_order_flying)} orders flying \n {len(self.list_order_done)} orders done \n {sum(1 for elem in self.list_drone if elem.state == "Base" and (elem.time_waiting_base is None or elem.time_waiting_base <= 0))} drones available')
        xdata = np.array([drone.x for drone in self.list_drone])
        ydata = np.array([drone.y for drone in self.list_drone])
        self.line.set_xdata(xdata)
        self.line.set_ydata(ydata)
        plt.draw()
        plt.pause(1e-17)

    def drone_noise(self):
        noise = np.zeros((self.y_max//self.granularity_noise, self.x_max//self.granularity_noise))
        for i in range(self.y_max//self.granularity_noise):
            for j in range(self.x_max//self.granularity_noise):
                noise[i, j] = 0
                x_obs = j*self.granularity_noise
                y_obs = (self.y_max//self.granularity_noise - i)*self.granularity_noise
                z_obs = 0
                for drone in self.list_drone:
                    x = drone.x - x_obs
                    y = drone.y - y_obs
                    z = drone.height
                    if (drone.state == 'Go' or drone.state == 'Back') and np.sqrt(x*x + y*y + z*z) < 1000:
                        noise[i, j] += drone.noise(x_obs, y_obs, z_obs)
        noise = 20*np.log10(noise/2e-5)
        return noise

    def plot_noise(self):
        noise = self.drone_noise()
        self.line_noise.set_data(noise)
        self.axes_noise.set_title(f'Max noise : {int(np.max(noise))}dB')
        plt.draw()
        plt.pause(1e-17)

    def plot_waiting_order(self):
        list_waiting = [elem.time_delivery for elem in self.list_order_done]
        plt.figure()
        plt.hist(list_waiting, bins=50)
        plt.show()
