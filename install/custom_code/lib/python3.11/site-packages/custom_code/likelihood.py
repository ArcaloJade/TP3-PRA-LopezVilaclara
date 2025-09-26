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

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # Paso 1: convertir a arreglo 2D
        data = np.array(msg.data, dtype=np.int16).reshape((height, width))

        # Inicializar campo de verosimilitud
        likelihood = np.zeros_like(data, dtype=np.float32)

        # Parámetros de la gaussiana
        sigma = 3.0  # en celdas
        var = sigma**2
        kernel_size = int(3 * sigma)  # radio para calcular gaussiana
        x = np.arange(-kernel_size, kernel_size + 1)
        y = np.arange(-kernel_size, kernel_size + 1)
        xx, yy = np.meshgrid(x, y)
        gaussian_kernel = np.exp(-(xx**2 + yy**2) / (2 * var))

        # Paso 2: encontrar celdas ocupadas
        occupied_y, occupied_x = np.where(data == 100)

        # Paso 3: sumar gaussianas en cada celda ocupada
        for (cy, cx) in zip(occupied_y, occupied_x):
            y_min = max(0, cy - kernel_size)
            y_max = min(height, cy + kernel_size + 1)
            x_min = max(0, cx - kernel_size)
            x_max = min(width, cx + kernel_size + 1)

            ky_min = y_min - (cy - kernel_size)
            ky_max = ky_min + (y_max - y_min)
            kx_min = x_min - (cx - kernel_size)
            kx_max = kx_min + (x_max - x_min)

            likelihood[y_min:y_max, x_min:x_max] += gaussian_kernel[ky_min:ky_max, kx_min:kx_max]

        # Paso 4: normalizar a rango [0, 100]
        likelihood = likelihood / likelihood.max() * 100.0
        likelihood = likelihood.astype(np.int8)

        # Paso 5: aplanar
        prob_msg.data = likelihood.flatten().tolist()

        self.pub.publish(prob_msg)
        self.get_logger().info("Published likelihood map")

def main(args=None):
    rclpy.init(args=args)
    node = LikelihoodMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
