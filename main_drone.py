from class_launching import LaunchingBase
from random_destination import RandomDestination
from random import uniform
from nb_order import number_order, check_proba_order
import json
import numpy as np


def main_drone(dict_setup):
    check_proba_order(dict_setup)
    base = LaunchingBase(dict_setup)
    destinations = RandomDestination(dict_setup["x_max"], dict_setup["y_max"])
    time_tot_simu = (dict_setup["time_simu"][1] - dict_setup["time_simu"][0])*3600

    for i in range(time_tot_simu//dict_setup["dt"]):
        t = i*dict_setup["dt"] + dict_setup["time_simu"][0]*3600
        for _ in range(number_order(dict_setup, t)):
            x, y = destinations.random()
            while abs(x - dict_setup["x_base"]) < 5 and abs(y - dict_setup["y_max"]) < 5:
                x, y = destinations.random()

            weight = np.around(uniform(dict_setup["range_weight"][0], dict_setup["range_weight"][1]), 2)
            base.new_order(x, y, weight)

        base.launch_order()
        base.move_all()
        base.check_arrived_order()
        base.plot_drone(t)

    print('Temps total de vol ', sum([elem.flying_time_tot for elem in base.list_drone])/3600, ' heures')
    print('Energy Total ', base.energy_tot_consumed/1000, ' kWh')

    base.plot_waiting_order()


if __name__ == '__main__':
    with open('setup.json') as file:
        dict_setup = json.load(file)
    main_drone(dict_setup)
