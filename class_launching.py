from class_drone import Drone
from class_order import Order
import matplotlib.pyplot as plt
import numpy as np
import json


class LaunchingBase():
    def __init__(self, dict_setup):
        self.x = dict_setup["x_base"]
        self.y = dict_setup["y_base"]
        self.x_max = dict_setup["x_max"]
        self.y_max = dict_setup["y_max"]
        self.list_drone = [Drone(dict_setup) for _ in range(dict_setup["nb_drone"])]
        noise = self.list_drone[0].calculate_noise()
        for elem in self.list_drone:
            elem.base_noise = noise

        self.list_order = []
        self.list_order_flying = []
        self.list_order_done = []
        self.dt = dict_setup["dt"]
        self.time_wait_base = dict_setup["time_wait_base"]
        self.recharge_power = dict_setup["recharge_power"]
        self.energy_tot_consumed = dict_setup["cap_bat"]*dict_setup["nb_drone"]
        self.energy = [self.energy_tot_consumed]
        self.granularity_noise = dict_setup["granularity_noise"]
        self.dist_calc_noise = dict_setup["dist_calc_noise"]

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
        self.line_noise = self.axes_noise.imshow(np.zeros((self.y_max//self.granularity_noise, self.x_max//self.granularity_noise)), extent=[0, self.x_max, 0, self.y_max], cmap=plt.cm.Blues, vmin=30, vmax=75)
        plt.colorbar(self.line_noise)
        
        self.average_noise = np.zeros((self.y_max//self.granularity_noise, self.x_max//self.granularity_noise))
        self.iter = (dict_setup["time_simu"][1] - dict_setup["time_simu"][0])*3600//dict_setup["dt"]

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
        self.energy.append(self.energy_tot_consumed)

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
        for drone in self.list_drone:
            if (drone.state == 'Go' or drone.state == 'Back'):
                noise += drone.noise(self.x_max//self.granularity_noise, self.y_max//self.granularity_noise, self.granularity_noise, self.dist_calc_noise)

        self.average_noise += noise/self.iter

        noise = 10*np.log10(noise) + 120
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
        plt.hist(list_waiting, bins=50, density=True)
        plt.xlabel("Temps de livraison (s)")
        plt.ylabel("Proportion")
        plt.show()
        
    def plot_energy(self):
        delta = int(30*60/self.dt)
        E = [(self.energy[k] - self.energy[k-1])/0.5 for k in range(1, len(self.energy), delta)]
        t = [7 + 0.5*k for k in range(len(E))]
        plt.figure()
        plt.bar(t, np.array(E)/1000, align='edge')
        plt.xlim(7,24)
        plt.xlabel("Heure")
        plt.ylabel("Puissance moyenne (kW)")
        plt.show()
        
        # plt.figure()
        # E = [(self.energy[k] - self.energy[k-1])*3600/30/1000 for k in range(1, len(self.energy))]
        # t = [7 + 30/3600*k for k in range(len(E))]
        # plt.plot(t, E)
        # plt.xlabel("Heure")
        # plt.xlim(7,24)
        # plt.ylabel("Puissance électrique (kW)")
        # plt.show()
        
        plt.figure()
        t = [7 + 30/3600*k for k in range(len(self.energy))]
        plt.plot(t, np.array(self.energy)/1000)
        plt.xlabel("Heure")
        plt.xlim(7,24)
        plt.ylabel("Consommation electrique cumulée (kWh)")
        plt.show()

    def to_dict(self):
        return {
            'drones': [drone.to_dict() for drone in self.list_drone],
            'orders': [order.to_dict() for order in self.list_order]
            }

    def save_json(self, path):
        np.save('avg_noise', 10*np.log10(self.average_noise) + 120)
        json_text = json.dumps(self.to_dict())

        with open(path, 'w') as file:
            file.write(json_text)
