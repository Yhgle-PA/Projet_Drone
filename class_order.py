class Order():
    def __init__(self, x, y, weight):
        self.x = x
        self.y = y
        self.time_delivery = 0
        self.weight = weight

    def waiting_order(self, dt):
        self.time_delivery += dt
