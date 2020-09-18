from class_launching import LaunchingBase
from random_destination import RandomDestination
from random import randint, random


def main_drone(x_base, y_base, x_max, y_max, nb_drone, speed_drone, high_drone, dt, delivery_time, time_wait_base, time_tot_simu):
    base = LaunchingBase(nb_drone, x_base, y_base, speed_drone, high_drone, x_max, y_max, delivery_time, dt, time_wait_base)
    destinations = RandomDestination(x_max, y_max)

    for i in range(time_tot_simu//dt):
        if random() < 1:
            #base.new_order(randint(0, x_max), randint(0, y_max))
            x, y = destinations.random()
            base.new_order(x, y)
        base.launch_order()
        base.move_all()
        base.check_arrived_order()
        base.plot_drone(i*dt)

    base.plot_waiting_order()

    print('Temps maximum d attente ', max([elem.time_delivery for elem in base.list_order_done]), ' secondes')


if __name__ == '__main__':
    x_base = 2500
    y_base = 2000
    x_max = 16000
    y_max = 11500
    nb_drone = 100
    speed_drone = 5  # m/s
    high_drone = 10  # m
    dt = 30  # s
    delivery_time = 180  # s
    time_wait_base = 300  # s
    time_tot_simu = 7200  # s
    main_drone(x_base, y_base, x_max, y_max, nb_drone, speed_drone, high_drone, dt, delivery_time, time_wait_base, time_tot_simu)
