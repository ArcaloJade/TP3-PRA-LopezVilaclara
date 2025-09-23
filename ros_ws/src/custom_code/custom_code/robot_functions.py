import numpy as np
import random
import copy

class particle():

    def __init__(self):
        self.x = (random.random()-0.5)*2  # initial x position
        self.y = (random.random()-0.5)*2 # initial y position
        self.orientation = random.uniform(-np.pi,np.pi) # initial orientation
        self.weight = 1.0

    def set(self, new_x, new_y, new_orientation):
        '''
        set: sets a robot coordinate, including x, y and orientation
        '''
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def move_odom(self, odom, noise):
        '''
        move_odom: Takes in Odometry data and moves the robot based on the odometry data
        
        Devuelve una particula (del robot) actualizada
        '''      
        delta_rot1 = odom['r1']
        delta_rot2 = odom['r2']
        delta_trans = odom['t']
        alpha1, alpha2, alpha3, alpha4 = noise

        delta_rot1_hat = delta_rot1 + np.random.normal(0, np.sqrt(alpha1*delta_rot1**2 + alpha2*delta_trans**2))
        delta_trans_hat = delta_trans + np.random.normal(0, np.sqrt(alpha3*delta_trans**2 + alpha4*(delta_rot1**2 + delta_rot2**2)))
        delta_rot2_hat = delta_rot2 + np.random.normal(0, np.sqrt(alpha1*delta_rot2**2 + alpha2*delta_trans**2))

        x_new = self.x + delta_trans_hat * np.cos(self.orientation + delta_rot1_hat)
        y_new = self.y + delta_trans_hat * np.sin(self.orientation + delta_rot1_hat)
        theta_new = self.orientation + delta_rot1_hat + delta_rot2_hat

        theta_new = (theta_new + np.pi) % (2 * np.pi) - np.pi

        self.set(x_new, y_new, theta_new)

    def set_weight(self, weight):
        '''
        set_weights: sets the weight of the particles
        '''
        #noise parameters
        self.weight  = float(weight)

class RobotFunctions:

    def __init__(self, num_particles=0):
        if num_particles != 0:
            self.num_particles = num_particles
            self.particles = []
            for _ in range(self.num_particles):
                self.particles.append(particle())

            self.weights = np.ones(self.num_particles) / self.num_particles

    def get_weights(self,):
        return self.weights
    
    def get_particle_states(self,):
        samples = np.array([[p.x, p.y, p.orientation] for p in self.particles])
        return samples
    
    def move_particles(self, deltas):
        for part in self.particles:
            part.move_odom(deltas, [10.0, 10.0, 0.1, 0.1])
    
    def get_selected_state(self,):
        '''
        Esta funcion debe devolver lo que ustedes consideran como la posición del robot segun las particulas.
        Queda a su criterio como la obtienen en base a las particulas.
        '''
        xs = np.array([p.x for p in self.particles])
        ys = np.array([p.y for p in self.particles])
        thetas = np.array([p.orientation for p in self.particles])
        weights = np.array([p.weight for p in self.particles])

        if np.sum(weights) > 0:
            weights = weights / np.sum(weights)
        else:
            weights = np.ones(len(weights)) / len(weights)

        x_est = np.average(xs, weights=weights)
        y_est = np.average(ys, weights=weights)

        sin_sum = np.average(np.sin(thetas), weights=weights)
        cos_sum = np.average(np.cos(thetas), weights=weights)
        theta_est = np.arctan2(sin_sum, cos_sum)

        return [x_est, y_est, theta_est]

    def update_particles(self, data, map_data, grid):
        '''
        La funcion update_particles se llamará cada vez que se recibe data del LIDAR
        Esta funcion toma:
            data: datos del lidar en formato scan (LaserScan).
                Se usa scan_refererence para convertir los datos crudos en
                posiciones globales calculadas
            map_data: mensaje crudo del mapa de likelihood (OccupancyGrid).
            grid: representación como matriz de numpy del mapa de likelihood. 
                Importante:
                    - La grilla se indexa como grid[y, x]
                    - La celda (0,0) corresponde a la esquina inferior izquierda del mapa.
        
        Esta funcion debe actualizar el valor de probabilidad (weight) de cada partícula
        y luego resamplear las partículas.
        '''

        resolution = map_data.info.resolution
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y
        width = map_data.info.width
        height = map_data.info.height

        new_weights = []

        for part in self.particles:
            points_map = self.scan_refererence(
                data.ranges,
                data.range_min,
                data.range_max,
                data.angle_min,
                data.angle_max,
                data.angle_increment,
                [part.x, part.y, part.orientation]
            )

            xs, ys = points_map
            log_prob = 0.0

            for x_z, y_z in zip(xs, ys):
                x_cell = int((x_z - origin_x) / resolution)
                y_cell = int((y_z - origin_y) / resolution)

                if 0 <= x_cell < width and 0 <= y_cell < height:
                    val = grid[y_cell, x_cell]
                    if val >= 0:
                        p = val / 100.0
                    else:
                        p = 0.01
                else:
                    p = 0.01

                log_prob += np.log(max(p, 1e-6))

            part.weight = np.exp(log_prob)
            new_weights.append(part.weight)

        new_weights = np.array(new_weights)
        if np.sum(new_weights) > 0:
            new_weights /= np.sum(new_weights)
        else:
            new_weights = np.ones(len(new_weights)) / len(new_weights)

        for i, part in enumerate(self.particles):
            part.weight = new_weights[i]

        indices = np.random.choice(
            range(len(self.particles)),
            size=len(self.particles),
            p=new_weights
        )
        new_particles = [copy.deepcopy(self.particles[i]) for i in indices]
        self.particles = new_particles

        self.weights = new_weights



    def scan_refererence(self, ranges, range_min, range_max, angle_min, angle_max, angle_increment, last_odom):
        '''
        Scan Reference recibe:
            - ranges: lista rangos del escáner láser
            - range_min: rango mínimo del escáner
            - range_max: rango máximo del escáner
            - angle_min: ángulo mínimo del escáner
            - angle_max: ángulo máximo del escáner
            - angle_increment: incremento de ángulo del escáner
            - last_odom: última odometría [tx, ty, theta]
        Devuelve puntos en el mapa transformados a coordenadas globales donde 
            - points_map[0]: coordenadas x
            - points_map[1]: coordenadas y
        '''
        # xs = []
        # ys = []
        # angle = angle_min
        # tx, ty, theta = last_odom
        # for r in ranges:
        #     if range_min < r < range_max:
        #         x_local = r * np.cos(angle)
        #         y_local = r * np.sin(angle)
        #         x_global = tx + np.cos(theta) * x_local - np.sin(theta) * y_local
        #         y_global = ty + np.sin(theta) * x_local + np.cos(theta) * y_local
        #         xs.append(x_global)
        #         ys.append(y_global)
        #     angle += angle_increment
        # points_map = np.array([xs, ys])
        # return points_map
        xs = []
        ys = []
        tx, ty, theta = last_odom
        angle = angle_min
        for r in ranges:
            if range_min < r < range_max:
                ang_global = theta + angle
                x_global = tx + r * np.cos(ang_global)
                y_global = ty + r * np.sin(ang_global)
                xs.append(x_global)
                ys.append(y_global)
            angle += angle_increment
        return np.array([xs, ys])
