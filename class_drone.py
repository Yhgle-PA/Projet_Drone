import numpy as np


class Drone():
    def __init__(self, dict_setup):
        # Position
        self.x = dict_setup["x_base"]
        self.y = dict_setup["y_base"]
        self.height = 0
        self.base_x = dict_setup["x_base"]
        self.base_y = dict_setup["y_base"]

        # Drone characteristics
        self.horizontal_speed = dict_setup["horizontal_speed"]
        self.vertical_speed = dict_setup["vertical_speed"]
        self.height_flight = dict_setup["height_flight"]
        self.state = 'Base'
        self.delivery_time = dict_setup["delivery_time"]
        self.timer_delivery = None
        self.flying_time_tot = 0
        self.time_waiting_base = None

        # Battery
        self.cap_bat = dict_setup["cap_bat"]
        self.cap_bat_act = self.cap_bat
        self.horizontal_power = dict_setup["horizontal_power"]
        self.vertical_power = dict_setup["horizontal_power"]

        # Order infos
        self.order = None
        self.dest_x = None
        self.dest_y = None

    def move(self, dt):
        if self.cap_bat < 0:
            raise Exception('No more battery for a drone')

        if self.state == 'Deliver':
            # The drone has landed, it waits to deliver the order
            self.timer_delivery -= dt
            if self.timer_delivery <= 0:
                self.state = 'Back'
                self.dest_x = self.base_x
                self.dest_y = self.base_y
                self.order.time_delivery += self.flying_time + self.delivery_time

        elif self.state == 'Go' or self.state == 'Back':
            self.flying_time += dt
            self.flying_time_tot += dt
            if self.height == self.height_flight and self.x != self.dest_x and self.y != self.dest_y:
                # Horizontal Move
                self.cap_bat_act -= self.horizontal_power*dt/3600
                dx = self.dest_x - self.x
                dy = self.dest_y - self.y
                if dy == 0:
                    dt_x = dt
                    dt_y = 0
                else:
                    dt_x = dt * abs(dx/dy)/(1+abs(dx/dy))
                    dt_y = dt - dt_x

                move_x = np.sign(dx) * min(abs(dx), self.horizontal_speed*dt_x)
                self.x += move_x

                move_y = np.sign(dy) * min(abs(dy), self.horizontal_speed*dt_y)
                self.y += move_y

            elif self.x == self.dest_x and self.y == self.dest_y and self.height != 0:
                # Vertical landing
                self.cap_bat_act -= self.vertical_power*dt/3600
                dh = -self.height
                move_h = min(dh, self.vertical_speed*dt)
                self.height += move_h

            elif self.height != self.height_flight:
                # Vertical lift off
                self.cap_bat_act -= self.vertical_power*dt/3600
                dh = self.height_flight - self.height
                move_h = min(dh, self.vertical_speed*dt)
                self.height += move_h
            else:
                print(self.height, self.height_flight, self.x, self.y, self.dest_x, self.dest_y)
                raise Exception('No cases should be there')

            if self.x == self.dest_x and self.y == self.dest_y and self.height == 0 and (self.state == 'Go' or self.state == 'Back'):
                # Check if the drone arrived to the delivery place or to the base
                if self.dest_x == self.base_x and self.dest_y == self.base_y:
                    self.state = 'Back to Base'
                else:
                    self.state = 'Deliver'
                    self.timer_delivery = self.delivery_time

        elif self.state == 'Base' and self.time_waiting_base is not None:
            # Wait in the base the time needed
            self.time_waiting_base -= dt

    def begin_order(self, order):
        self.order = order
        self.dest_x = order.x
        self.dest_y = order.y
        self.state = 'Go'
        self.flying_time = 0
        
    
    def to_dict(self):
        order = self.order.to_dict() if self.order is not None else None
        return {
            'x': self.x,
            'y': self.y,
            'state': self.state,
            'order': order
            }
