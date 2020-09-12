import numpy as np


class Drone():
    def __init__(self, speed, high, x, y, delivery_time):
        self.x = x
        self.y = y
        self.base_x = x
        self.base_y = y
        self.speed = speed
        self.high = high
        self.order = None
        self.dest_x = None
        self.dest_y = None
        self.state = 'Base'
        self.delivery_time = delivery_time
        self.timer_delivery = None
        self.flying_time_tot = 0
        self.time_waiting_base = None

    def move(self, dt):
        if self.state == 'Deliver':
            self.timer_delivery -= dt
            self.flying_time += dt
            self.flying_time_tot += dt
            if self.timer_delivery <= 0:
                self.state = 'Move'
                self.dest_x = self.base_x
                self.dest_y = self.base_y
                self.order.time_delivery += self.flying_time

        elif self.state == 'Move':
            self.flying_time += dt
            self.flying_time_tot += dt
            dx = self.dest_x - self.x
            dy = self.dest_y - self.y
            if dy == 0:
                dt_x = dt
                dt_y = 0
            else:
                dt_x = dt * abs(dx/dy)/(1+abs(dx/dy))
                dt_y = dt - dt_x

            move_x = np.sign(dx) * min(abs(dx), self.speed*dt_x)
            self.x += move_x

            move_y = np.sign(dy) * min(abs(dy), self.speed*dt_y)
            self.y += move_y

            if abs(self.x - self.dest_x) <= 0.01 and abs(self.y - self.dest_y) <= 0.01:
                self.x = self.dest_x
                self.y = self.dest_y
                if self.dest_x == self.base_x and self.dest_y == self.base_y:
                    self.state = 'Base'
                else:
                    self.state = 'Deliver'
                    self.timer_delivery = self.delivery_time

        elif self.state == 'Base' and self.time_waiting_base is not None:
            self.time_waiting_base -= dt

    def begin_order(self, order):
        self.order = order
        self.dest_x = order.x
        self.dest_y = order.y
        self.state = 'Move'
        self.flying_time = 0
