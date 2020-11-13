import numpy as np
import scipy.special


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
        self.weight = dict_setup["weight_drone"]
        self.state = 'Base'
        self.delivery_time = dict_setup["delivery_time"]
        self.nb_pale = dict_setup["nb_pale"]
        self.vit_rotor = dict_setup["vit_rotor"]
        self.ray_eq_drone = dict_setup["ray_eq_drone"]
        self.corde_pale = dict_setup["corde_pale"]
        self.dict_mod_acoust = dict_setup["dict_mod_acoust"]
        self.timer_delivery = None
        self.flying_time_tot = 0
        self.time_waiting_base = None

        # Battery
        self.cap_bat = dict_setup["cap_bat"]
        self.cap_bat_act = self.cap_bat
        self.horizontal_power = dict_setup["horizontal_power"]
        self.vertical_power = dict_setup["vertical_power"]

        # Order infos
        self.order = None
        self.dest_x = None
        self.dest_y = None
        self.weight_order = 0

        # Noise drone
        self.base_noise = None

    def move(self, dt):
        if self.cap_bat_act < 0:
            raise Exception('No more battery for a drone')

        if self.state == 'Deliver':
            # The drone has landed, it waits to deliver the order
            self.timer_delivery -= dt
            if self.timer_delivery <= 0:
                self.state = 'Back'
                self.dest_x = self.base_x
                self.dest_y = self.base_y
                self.order.time_delivery += self.flying_time + self.delivery_time
                self.weight = 0

        elif self.state == 'Go' or self.state == 'Back':
            self.flying_time += dt
            self.flying_time_tot += dt
            if self.height == self.height_flight and self.x != self.dest_x and self.y != self.dest_y:
                # Horizontal Move
                self.cap_bat_act -= self.horizontal_power*dt/3600
                dx = self.dest_x - self.x
                dy = self.dest_y - self.y
                dxy = np.sqrt(dx*dx + dy*dy)
                move_xy = min(dxy, self.horizontal_speed*dt)
                factor_move = move_xy/dxy
                move_x = dx*factor_move
                self.x += move_x

                move_y = dy*factor_move
                self.y += move_y

                if abs(self.x - self.dest_x) <= 0.001 and abs(self.y - self.dest_y) <= 0.001:
                    # If it's at less than 1cm of the destination
                    self.x = self.dest_x
                    self.y = self.dest_y

            elif self.x == self.dest_x and self.y == self.dest_y and self.height != 0:
                # Vertical landing
                self.cap_bat_act -= self.vertical_power*dt/3600
                dh = -self.height
                move_h = min(dh, self.vertical_speed*dt)
                self.height += move_h
                if self.height <= 0.001:
                    # If it's almost landed
                    self.height = 0

            elif self.height != self.height_flight:
                # Vertical lift off
                self.cap_bat_act -= self.vertical_power*dt/3600
                dh = self.height_flight - self.height
                move_h = min(dh, self.vertical_speed*dt)
                self.height += move_h
                if self.height_flight - self.height <= 0.001:
                    # If it's almost at the good height flight
                    self.height = self.height_flight

            else:
                print(self.height, self.height_flight, self.x, self.y, self.dest_x, self.dest_y)
                raise Exception('No cases should be there')

            if self.x == self.dest_x and self.y == self.dest_y and self.height == 0 and (self.state == 'Go' or self.state == 'Back'):
                # Check if the drone is arrived to the delivery place or to the base
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
        self.weight_order = order.weight
        self.state = 'Go'
        self.flying_time = 0

    def calculate_noise(self):
        # CLROT         Programme de calcul du bruit de raies rayonné
        #               par un rotor (supposé libre) de drone.

        # Ce module restitue la pression sonore complexe en champ lointain pour
        # un harmonique donné de la fréquence de passage des pales. Il nécessite
        # la définition préalable d'un ensemble d'harmoniques de charge utilisés
        # comme données d'entrée.
        # Le calcul est fait pour un rayon equivalent unique.

        # INITIALISATIONS

        R = 1                  # distance de l'observateur au centre du rotor
        rho = 1.2              # densité de l'air
        c0 = 340               # célérité du son(m/s)

        #                       DISCRETISATION DE LA SPHERE D'ECOUTE
        #   On définit une sphère d'écoute de rayon R centrée sur le stator.
        #
        N = 360                 # nombre de positions angulaires en élévation(par rapport à l'axe)
        N1 = 360                # nombre de positions angulaires en azimut(autour de l'axe)
        theo = np.zeros((N, 1))
        phio = np.zeros((1, N1+1))
        dthe = np.pi/N
        dphi = 2*np.pi/N1

        S = theo*phio          # matrice du maillage de la surface
        #   les éléments de la matrice S donnent les aires des surface élémentaires
        #  dans le système des coordonnées sphériques(utile pour intégrer une
        #  intensité sonore et déterminer ainsi la puissance totale rayonnée si
        #  nécessaire).
        for j in range(N):
            theo[j, 0] = (j+1-0.5)*dthe
            S[j, :] = R*R*abs(np.sin(theo[j]))*dthe*dphi*np.ones(N1+1)

        for k in range(N1+1):
            phio[0, k] = (k+1-0.5)*dphi

        #                  INITIALISATION - DEFINITION SUCCINCTE DU ROTOR
        #                  **********************************************
        B = self.nb_pale   # nombre de pales du rotor
        Gom = self.vit_rotor    # vitesse de rotation du rotor en tours par minute
        Gom = Gom*2*np.pi/60    # pulsation associée
        R0 = self.ray_eq_drone  # rayon équivalent
        M = Gom*R0/c0      # nombre de Mach tangentiel en R0
        gam = 45           # calage des pales en degrés
        gam = np.pi*gam/180
        c = self.corde_pale     # corde des pales(en R0)

        #   BOUCLE SUR LES HARMONIQUES DE BRUIT
        #   ***********************************

        Nraies = 20
        Int0 = np.zeros(S.shape)

        for jraie in range(Nraies):
            #   Détermination des bornes pour la troncature modale
            #   **************************************************
            #   On se fonde sur les valeurs de la fonction de Bessel. Quand cette
            #   dernière est proche de zero, les harmoniques de charge correspondants
            #   ne rayonnent pas. Avec le critère suivant, on est large...
            m = jraie + 1
            s1 = round(m*B*(1-M)-7)-10         # borne inf
            s2 = round(m*B*(1+M)+7)+10         # borne sup
            ssig = np.zeros(S.shape, dtype='complex128')

            cst = 1j*m*B*B*Gom*np.exp(1j*m*B*Gom*R/c0)/(4*np.pi*c0*R)

            #   Définition des harmoniques de charge
            #   ************************************

            #   Modèle simplifié d'harmoniques de charge pour une interaction
            #   potentielle avec un bras cylindrique. 3 paramètres ajustables:
            ai = self.dict_mod_acoust["ai"]
            ar = self.dict_mod_acoust["ar"]
            b = self.dict_mod_acoust["b"]

            # Attention: ce modèle vaut pour un arbre de transmission aligné sur
            # l'axe des X négatifs.
            for ss in range(s1, s2+1):
                if ss == 0:
                    Fs = 5    # pour la charge stationnaire
                else:
                    Fs = (ar-1j*ai)*(-1)**ss*ss*np.exp(-ss/b)

                arg = 0.5*c*ss/R0
                Sears = np.exp(1j*arg*(1-0.5*np.pi*np.pi/(1+2*np.pi*arg)))/np.sqrt(complex(1+2*np.pi*arg, 0))
                Fs = np.pi*rho*c*Gom*R0*Fs*Sears    # Formule simplifiée donnant la force en fonction des expresions du test précédent

                bes = (scipy.special.jv(m*B-ss, m*B*M*np.sin(theo)))*np.ones(N1+1)
                croc = np.cos(gam)*np.cos(theo)-(m*B-ss)*np.sin(gam)/(m*M*B)
                cro = croc*np.exp(1j*(m*B-ss)*(phio-np.pi/2))
                ssig += Fs*cro*bes

            Pac = cst*ssig  # pression acoustique locale

            Int0 += abs(Pac)*abs(Pac)/(rho*c0)*function_dBA((jraie+1)*Gom*B/2/np.pi)  # Intensité sonore locale avec pondération dBA

        return Int0

    def noise(self, size_x, size_y, granularity, dist_calc_noise):
        # Create the coordonates coordinates of the map
        _, y_mat = np.meshgrid(np.zeros(size_x), np.arange(size_y-1, -1, -1))
        y_mat = y_mat*granularity + granularity/2
        x_mat, _ = np.meshgrid(np.arange(0, size_x), np.zeros(size_y))
        x_mat = x_mat*granularity + granularity/2

        # Calculate just a part of the map, defined by the dist_calc_noise
        ind_x_min = max(0, int((self.x - dist_calc_noise/2)//granularity))
        ind_x_max = min(size_x-1, int((self.x + dist_calc_noise/2)//granularity))

        ind_y_min = size_y - min(size_y, int((self.y + dist_calc_noise/2)//granularity))
        ind_y_max = size_y - max(0, int((self.y - dist_calc_noise/2)//granularity))

        x_mat = x_mat[ind_y_min:ind_y_max, ind_x_min:ind_x_max]
        y_mat = y_mat[ind_y_min:ind_y_max, ind_x_min:ind_x_max]

        y_mat = y_mat.flatten() - self.y
        x_mat = x_mat.flatten() - self.x
        vec = np.vstack((x_mat, y_mat))

        # Change the reference with a translation and a rotation
        X_dir_drone = self.dest_x - self.x
        Y_dir_drone = self.dest_y - self.y
        N_drone = np.sqrt(X_dir_drone*X_dir_drone+Y_dir_drone*Y_dir_drone)
        if N_drone == 0:
            angle = 0
        else:
            C = X_dir_drone/N_drone
            S = -Y_dir_drone
            angle = np.sign(S)*np.arccos(C)

        mat_r = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])

        vec_trans = np.dot(mat_r, vec)
        x = vec_trans[0].reshape((ind_y_max - ind_y_min, ind_x_max - ind_x_min))
        y = vec_trans[1].reshape((ind_y_max - ind_y_min, ind_x_max - ind_x_min))

        # Transform the new euclidian coordinates into spherical coordinates
        z = np.zeros((ind_y_max - ind_y_min, ind_x_max - ind_x_min)) - self.height
        r = np.sqrt(x*x + y*y + z*z)
        x = np.where(x == 0, 1e-7, x)
        r = np.where(r == 0, 1e-7, r)
        theta = np.arccos(z/r)
        phi = np.arctan(y/x)

        # Calculate the noise for each point
        noise_loc = np.array([[self.base_noise[int(p), int(t)] for p, t in zip(line_phi, line_theta)] for line_phi, line_theta in zip(phi, theta)])
        noise_loc = noise_loc/r/r
        noise = np.ones((size_y, size_x))*1e-15
        noise[ind_y_min:ind_y_max, ind_x_min:ind_x_max] = noise_loc

        return noise

    def to_dict(self):
        order = self.order.to_dict() if self.order is not None else None
        return {
            'x': self.x,
            'y': self.y,
            'state': self.state,
            'order': order
            }


def function_dBA(freq):
    root = np.sqrt((freq**2 + 107.7**2)*(freq**2 + 737.9**2))
    num = 12194**2*freq**4
    denom = (freq**2 + 20.6**2)*(freq**2 + 12194**2)*root
    A_1000 = 0.7943411
    return num/denom/A_1000
