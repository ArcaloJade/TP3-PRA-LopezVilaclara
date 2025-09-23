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

        occupancy = np.array(msg.data, dtype=np.int8).reshape((height, width))
        # occupancy = np.flipud(occupancy)  # Descomentar si no funciona. Corrige para que (0,0) esté abajo

        occupied_mask = occupancy == 100

        occupied_coords = np.argwhere(occupied_mask)

        likelihood_field = np.zeros((height, width), dtype=np.float32)

        sigma_hit = 0.2
        z_random = 0.01
        z_hit = 0.9

        if len(occupied_coords) > 0:
            for i in range(height):
                for j in range(width):
                    if occupancy[i, j] == -1:
                        likelihood_field[i, j] = -1
                        continue
                    dist = np.min(np.linalg.norm(occupied_coords - np.array([i, j]), axis=1) * resolution)
                    p_hit = np.exp(-0.5 * (dist / sigma_hit) ** 2)
                    likelihood_field[i, j] = z_hit * p_hit + z_random

        likelihood_field_scaled = np.clip(likelihood_field * 100, 0, 100).astype(np.int8)

        likelihood_field_scaled[occupancy == -1] = -1

        prob_msg.data = likelihood_field_scaled.flatten().tolist()

        self.pub.publish(prob_msg)
        self.get_logger().info("Published likelihood map")

def main(args=None):
    rclpy.init(args=args)
    node = LikelihoodMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
