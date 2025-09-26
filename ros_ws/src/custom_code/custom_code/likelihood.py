import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np


class LikelihoodMapPublisher(Node):
    def __init__(self):
        super().__init__('likelihood_map_publisher')
        qos = rclpy.qos.QoSProfile(depth=1)
        qos.durability = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(OccupancyGrid, '/likelihood_map', qos)
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos
        )
    
    def map_callback(self, msg):
        prob_msg = OccupancyGrid()
        prob_msg.header = msg.header
        prob_msg.info = msg.info

        # Obtener dimensiones del mapa
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # Convertir msg.data a numpy array 2D para facilidad de cálculo
        map_data = np.array(msg.data, dtype=int).reshape((height, width))

        # Crear una matriz vacía para el likelihood
        likelihood = np.zeros((height, width), dtype=np.float32)

        # Obtener coordenadas de celdas ocupadas
        occupied_coords = np.argwhere(map_data == 100)

        if len(occupied_coords) == 0:
            # Si no hay celdas ocupadas, asignamos probabilidad mínima (0)
            likelihood.fill(0)
        else:
            # Para cada celda libre, calcular la distancia mínima a una celda ocupada
            for i in range(height):
                for j in range(width):
                    if map_data[i, j] == 0:  # solo celdas libres
                        # Calcular distancias a todas las celdas ocupadas
                        distances = np.sqrt((occupied_coords[:,0] - i)**2 + (occupied_coords[:,1] - j)**2)
                        min_dist = np.min(distances) * resolution  # multiplicar por resolución del mapa

                        # Aplicar función gaussiana: prob = exp(-(d^2)/(2*sigma^2))
                        sigma = 0.5  # ajustar según conveniencia (metros)
                        prob = np.exp(-(min_dist**2) / (2 * sigma**2))

                        # Convertir a escala 0-100
                        likelihood[i, j] = int(prob * 100)
                    elif map_data[i, j] == 100:
                        likelihood[i, j] = 100  # ocupadas
                    else:
                        likelihood[i, j] = 0  # desconocidas

        # Aplanar a un array unidimensional y convertir a int8
        prob_msg.data = likelihood.astype(np.uint8).flatten().tolist()

        self.pub.publish(prob_msg)
        self.get_logger().info("Published likelihood map")

def main(args=None):
    rclpy.init(args=args)
    node = LikelihoodMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
