class Drone():
    def ___init__(self, speed, high, x, y):
        self.x = x
        self.y = y
        self.base_x = x
        self.base_y = y
        self.speed = speed
        self.high = high
        self.dest_x = None
        self.dest_y = None
        self.state = 'Base'


