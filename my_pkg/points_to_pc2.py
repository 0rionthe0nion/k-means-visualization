import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np

class PointsToPC2(Node):
    def __init__(self):
        super().__init__('points_to_pc2')
        self.sub = self.create_subscription(Float32MultiArray, 'clustered_cloud', self.callback, 100)
        self.pub = self.create_publisher(PointCloud2, 'point_cloud', 100)

    def callback(self, msg):
        points = np.array(msg.data).reshape(-1, 4)
        self.get_logger().info(f'Received {points.shape[0]} points')

        pc2_msg = self.create_pc2(points)
        self.pub.publish(pc2_msg)
        self.get_logger().info('Published PointCloud2')

    def create_pc2(self, points):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = points.shape[0]

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='k', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 16  # 4 floats × 4 bytes
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True

        buffer = []
        for p in points:
            buffer.append(struct.pack('ffff', p[0], p[1], p[2], p[3]))
        msg.data = b''.join(buffer)

        return msg

def main(args=None):
    rclpy.init(args=args)
    node = PointsToPC2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
