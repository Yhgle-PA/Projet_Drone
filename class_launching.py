from class_drone import Drone
from class_order import Order
import matplotlib.pyplot as plt
import numpy as np


class LaunchingBase():
    def __init__(self, dict_setup):
        self.x = dict_setup["x_base"]
        self.y = dict_setup["y_base"]
        self.list_drone = [Drone(dict_setup) for _ in range(dict_setup["nb_drone"])]
        self.list_order = []
        self.list_order_flying = []
        self.list_order_done = []
        self.dt = dict_setup["dt"]
        self.time_wait_base = dict_setup["time_wait_base"]
        self.recharge_power = dict_setup["recharge_power"]
        self.energy_tot_consumed = dict_setup["cap_bat"]*dict_setup["nb_drone"]

        plt.show()
        self.axes = plt.gca()
        self.axes.set_xlim(0, dict_setup["x_max"])
        self.axes.set_ylim(0, dict_setup["y_max"])
        self.axes.plot(self.x, self.y, 'r+', markersize=40)
        img = plt.imread('lyon.PNG')
        self.axes.imshow(img, alpha=0.5, extent=[0, dict_setup["x_max"], 0, dict_setup["y_max"]])
        self.line, = self.axes.plot([], [], '*')

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

    def new_order(self, dest_x, dest_y):
        self.list_order.append(Order(dest_x, dest_y))

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

    def plot_drone(self, nb_seconds):
        self.axes.set_title(f'{nb_seconds} seconds \n {len(self.list_order)} orders waiting \n {len(self.list_order_flying)} orders flying \n {len(self.list_order_done)} orders done')
        xdata = np.array([drone.x for drone in self.list_drone])
        ydata = np.array([drone.y for drone in self.list_drone])
        self.line.set_xdata(xdata)
        self.line.set_ydata(ydata)
        plt.draw()
        plt.pause(1e-17)

    def plot_waiting_order(self):
        list_waiting = [elem.time_delivery for elem in self.list_order_done]
        plt.figure()
        plt.hist(list_waiting, bins=50)
        plt.show()
