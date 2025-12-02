import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import time

class ClusterPublisher(Node):
    def __init__(self):
        super().__init__('cluster_publisher')

        self.publisher = self.create_publisher(PointCloud2, '/point_cloud', 10)

        timer_period = 1.0   # publish every second
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Cluster PointCloud2 publisher started~")

    def timer_callback(self):
        msg = self.generate_pointcloud_msg()
        self.publisher.publish(msg)
        self.get_logger().info("Published /point_cloud")

    # --------------------------
    #     POINTCLOUD BUILDER
    # --------------------------
    def generate_pointcloud_msg(self):
        num = 300
        num_clusters = 10

        X = np.zeros((num, 4), dtype=np.float32)

        means = [i for i in range(0, num, int(num / num_clusters))]
        j = 0

        for i in range(num):
            X[i, 0:3] = np.random.normal(loc=means[j], size=3)
            X[i, 3] = j  # k-value
            if i % (num / num_clusters) == 0 and i != 0:
                j += 1

        # Create PointCloud2
        msg = PointCloud2()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.height = 1
        msg.width = num

        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='k', offset=12, datatype=PointField.UINT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 16          # 4 floats/uints × 4 bytes
        msg.row_step = msg.point_step * num
        msg.is_dense = True

        # Pack the binary buffer
        buffer = []
        for p in X:
            x, y, z, k = p
            buffer.append(struct.pack('<fffI', x, y, z, int(k)))

        msg.data = b"".join(buffer)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ClusterPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
