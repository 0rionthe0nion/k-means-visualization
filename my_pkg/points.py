import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class PointsPublisher(Node):
    def __init__(self):
        super().__init__('points_publisher')
        self.pub = self.create_publisher(Float32MultiArray, 'points_array', 10)

        # Generate your points here
        num = 300
        num_clusters = 10
        means = np.linspace(0, num_clusters-1, num_clusters)
        points_per_cluster = num // num_clusters
        X = np.zeros((num, 4))
        idx = 0
        for k in range(num_clusters):
            for _ in range(points_per_cluster):
                x, y, z = np.random.normal(loc=means[k], size=3)
                X[idx] = [x, y, z, k]
                idx += 1
        self.points = X

        # publish once every second
        self.timer = self.create_timer(1.0, self.publish_points)

    def publish_points(self):
        msg = Float32MultiArray()
        # Flatten the points array for the message
        msg.data = self.points.flatten().tolist()
        self.pub.publish(msg)

        # reshape for logging to see [x, y, z, k] per point
        reshaped = np.array(msg.data).reshape(-1, 4)
        self.get_logger().info(f'Published points:\n{reshaped.tolist()}')


def main(args=None):
    rclpy.init(args=args)
    node = PointsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()