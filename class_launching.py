from class_drone import Drone
from class_order import Order
import matplotlib.pyplot as plt
import numpy as np


class LaunchingBase():
    def __init__(self, nb_drone, x, y, speed, high, x_max, y_max, delivery_time, dt, time_wait_base):
        self.x = x
        self.y = y
        self.list_drone = [Drone(speed, high, x, y, delivery_time) for _ in range(nb_drone)]
        self.list_order = []
        self.list_order_flying = []
        self.list_order_done = []
        self.dt = dt
        self.time_wait_base = time_wait_base

        plt.show()
        self.axes = plt.gca()
        self.axes.set_xlim(0, x_max)
        self.axes.set_ylim(0, y_max)
        self.title = self.axes.text(1, 1, '', ha='right', va='top', fontsize=24)
        self.axes.plot(x, y, 'r+', markersize=40)
        self.line, = self.axes.plot([], [], '*')

    def launch_order(self):
        boolean = True
        while boolean and len(self.list_order) != 0:
            order = self.list_order[0]
            boolean = False
            for drone in self.list_drone:
                if drone.state == 'Base' and (drone.time_waiting_base is None or drone.time_waiting_base <= 0):
                    drone.begin_order(order)
                    del self.list_order[0]
                    self.list_order_flying.append(order)
                    boolean = True
                    break

    def new_command(self, dest_x, dest_y):
        self.list_order.append(Order(dest_x, dest_y))

    def check_arrived_order(self):
        for drone in self.list_drone:
            if drone.state == 'Base' and drone.order is not None:
                ind_order = self.list_order_flying.index(drone.order)
                self.list_order_done.append(self.list_order_flying[ind_order])
                del self.list_order_flying[ind_order]
                drone.order = None
                drone.time_waiting_base = self.time_wait_base

    def move_all(self):
        for drone in self.list_drone:
            drone.move(self.dt)

        for order_waiting in self.list_order:
            order_waiting.waiting_order(self.dt)

    def plot(self, nb_seconds):
        self.axes.set_title(f'{nb_seconds} seconds \n {len(self.list_order)} orders waiting \n {len(self.list_order_flying)} orders flying \n {len(self.list_order_done)} orders done')
        xdata = np.array([drone.x for drone in self.list_drone])
        ydata = np.array([drone.y for drone in self.list_drone])
        self.line.set_xdata(xdata)
        self.line.set_ydata(ydata)
        plt.draw()
        plt.pause(1e-17)
