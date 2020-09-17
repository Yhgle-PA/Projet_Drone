from class_launching import LaunchingBase
from random import randint, random
import json


def main_drone(dict_setup):
    base = LaunchingBase(dict_setup)

    for i in range(dict_setup["time_tot_simu"]//dict_setup["dt"]):
        if random() < 1:
            base.new_order(randint(0, dict_setup["x_max"]), randint(0, dict_setup["y_max"]))
        base.launch_order()
        base.move_all()
        base.check_arrived_order()
        base.plot_drone(i*dict_setup["dt"])

    print('Temps total de vol ', sum([elem.flying_time_tot for elem in base.list_drone])/3600, ' heures')
    print('Energy Total ', base.energy_tot_consumed/1000, ' kWh')

    base.plot_waiting_order()


if __name__ == '__main__':
    with open('setup.json') as file:
        dict_setup = json.load(file)
    main_drone(dict_setup)
