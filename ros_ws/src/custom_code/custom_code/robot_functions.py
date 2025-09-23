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
        # TODO: Completar
        return [0,0,0]

    def update_particles(self, data, map_data, grid):
        '''
        La funcion update_particles se llamará cada vez que se recibe data del LIDAR
        Esta funcion toma:
            data: datos del lidar en formato scan (Ver documentacion de ROS sobre tipo de dato LaserScan).
                  Pueden aprovechar la funcion scan_refererence del TP1 para convertir los datos crudos en
                  posiciones globales calculadas
            map_data: Es el mensaje crudo del mapa de likelihood. Pueden consultar la documentacion de ROS
                      sobre tipos de dato OccupancyGrid.
            grid: Es la representación como matriz de numpy del mapa de likelihood. 
                  Importante:
                    - La grilla se indexa como grid[y, x], primero fila (eje Y) y luego columna (eje X).
                    - La celda (0,0) corresponde a la esquina inferior izquierda del mapa en coordenadas de ROS.
        
        Esta funcion debe tomar toda esta data y actualizar el valor de probabilidad (weight) de cada partícula
        En base a eso debe resamplear las partículas. Tenga cuidado al resamplear de hacer un deepcopy para que 
        no sean el mismo objeto de python
        '''
        # TODO: Implementar la actualización de pesos y resampling
        pass


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
        
        # Valores para que no falle el codigo
        points_map = np.array([[0, 0, 0], [0, 0, 0]])


        return points_map