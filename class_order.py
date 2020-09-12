class Order():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.time_delivery = 0

    def waiting_order(self, dt):
        self.time_delivery += dt
