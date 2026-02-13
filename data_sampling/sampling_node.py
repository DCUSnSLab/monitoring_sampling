import rclpy
import time
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, CompressedImage, PointCloud2
from hunter_msgs.msg import HunterStatus
from sensor_msgs_py import point_cloud2


class SamplingNode(Node):
    def __init__(self):
        super().__init__('data_sampling')

        self.periods = {
            'gps': 1.0,
            'vehicle_status': 1.0,
            'camera' : 1.0,
            'pcd' : 1.0,
        }

        self.last_sent_time = {
            'gps': 0.0,
            'vehicle_status': 0.0,
            'camera' : 0.0,
            'pcd' : 0.0,
        }

        gps_topics = [
            '/ublox_gps_node/fix',
            '/ublox_gps/fix',
            '/ublox/fix',
        ]

        self.gps_topic = None
        self.last_lat = None
        self.last_lon = None
        self.max_points = 10000

        topics = self.get_topic_names_and_types()
        topic_name = [t[0] for t in topics]

        for t in gps_topics:
            if t in topic_name:
                self.gps_topic = t
                break

        #pub
        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/gps_sampled',
            10
        )

        self.vehicle_status_pub = self.create_publisher(
            HunterStatus,
            '/vehicle_status_sampled',
            10
        )

        self.camera_pub = self.create_publisher(
            CompressedImage,
            '/camera_sampled',
            10
        )

        self.pcd_pub = self.create_publisher(
            PointCloud2,
            '/pcd_sampled',
            10
        )

        #sub
        if self.gps_topic is not None:
            self.create_subscription(
                NavSatFix,
                self.gps_topic,
                lambda msg: self.sensor_callback(
                    name='gps',
                    msg=msg,
                    pub=self.gps_pub
                ),
                10
            )

        self.create_subscription(
            HunterStatus,
            '/hunter_status',
            lambda msg: self.sensor_callback(
                name='vehicle_status',
                msg=msg,
                pub=self.vehicle_status_pub
            ),
            10
        )

        self.create_subscription(
            CompressedImage,
            '/zed/zed_node/left/image_rect_color/compressed',
            lambda msg: self.sensor_callback(
                name='camera',
                msg=msg,
                pub=self.camera_pub
            ),
            10
        )

        self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            lambda msg: self.sensor_callback(
                name='pcd',
                msg=msg,
                pub=self.pcd_pub
            ),
            10
        )

        self.get_logger().info('sampling node started')


    def sensor_callback(self, name, msg, pub):
        # now = time.time()
        now = self.get_clock().now().nanoseconds * 1e-9

        period = self.periods[name]

        if name != 'gps' and now - self.last_sent_time[name] < period:
            return

        if name == 'gps':
            lat = round(msg.latitude, 5)
            lon = round(msg.longitude, 5)

            changed = (self.last_lat != lat or self.last_lon != lon)

            if not changed and now - self.last_sent_time[name] < period:
                return
            
            self.last_lat = lat
            self.last_lon = lon

        if name == 'pcd':
            points = np.array([
                [p[0], p[1], p[2]]
                for p in point_cloud2.read_points(
                    msg,
                    field_names=('x', 'y', 'z'),
                    skip_nans=True
                )
            ])

            if points.shape[0] == 0:
                return

            if points.shape[0] > self.max_points:
                step = max(1, points.shape[0] // self.max_points)
                points = points[::step][:self.max_points]

            out_msg = point_cloud2.create_cloud_xyz32(
                header=msg.header,
                points=points
            )

            pub.publish(out_msg)
            self.last_sent_time[name] = now
            self.get_logger().info(f"[SEND] {name}")
            return

        pub.publish(msg)
        self.last_sent_time[name] = now
        self.get_logger().info(f"[SEND] {name}")
    

def main():
    rclpy.init()
    node = SamplingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()