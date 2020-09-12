from class_drone import Drone


class LaunchingBase():
    def __init__(self, nb_drone, x, y, speed, high):
        self.x = x
        self.y = y
        self.list_drone = [Drone(speed, high, x, y) for _ in range(nb_drone)]
