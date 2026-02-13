import rclpy
import datetime
import numpy as np

from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, CompressedImage, PointCloud2
from hunter_msgs.msg import HunterStatus
from sensor_msgs_py import point_cloud2
from data_sampling_msgs.msg import SamplingStatus


class SamplingStatusManager:
    def __init__(self, node):
        self.node = node

        self.status_pub = self.node.create_publisher(
            SamplingStatus,
            '/sampling_status',
            10
        )

        self.sampling_status_timer = self.node.create_timer(
            1.0,
            self.publish_sampling_status
        )
    
    def publish_sampling_status(self):
        msg = SamplingStatus()
        now = self.node.get_clock().now()
        msg.stamp = now.to_msg()

        mapping = {
            'gps': 'gps',
            'pcd': 'pcd',
            'camera': 'camera',
            'vehicle_status': 'vehicle_status'
        }

        for value, field in mapping.items():
            sensor_msg = getattr(msg, field)
            sensor_status = self.node.status[value]

            sensor_msg.topic = sensor_status.get('topic', "")

            last_time = sensor_status['last_publish']

            if last_time:
                age = (now - last_time).nanoseconds / 1e9
                sensor_msg.last_publish = last_time.to_msg()
                sensor_msg.age = age

                timeout = self.node.periods[value] * 3

                if age <= timeout:
                    sensor_msg.state = "connected"
                elif age <= timeout * 2:
                    sensor_msg.state = "slow"
                else:
                    sensor_msg.state = "lost"

            else:
                sensor_msg.state = "no_data"
                sensor_msg.age = -1.0
            
        self.status_pub.publish(msg)


class SamplingNode(Node):
    def __init__(self):
        super().__init__('data_sampling')
        self.status = {
            'gps': {
                'topic' : '',
                'state': 'Not connected',
                'last_publish': None,
            },
            'pcd': {
                'state': 'No data',
                'last_publish': None,
            },
            'camera': {
                'state': 'No data',
                'last_publish': None,
            },
            'vehicle_status' : {
                'state': 'No data',
                'last_publish': None,
            }
        }

        self.sampling_status = SamplingStatusManager(self)

        self.periods = {
            'gps': 1.0,
            'vehicle_status': 1.0,
            'camera' : 1.0,
            'pcd' : 1.0,
        }

        self.last_sent_time = {
            'gps': None,
            'vehicle_status': None,
            'camera' : None,
            'pcd' : None,
        }

        self.gps_topic = None
        self.gps_subscription = None
        self.last_lat = None
        self.last_lon = None
        self.max_points = 10000

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.gps_check_timer = self.create_timer(
            2.0,
            self.check_gps_topic
        )

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
            sensor_qos
        )

        self.pcd_pub = self.create_publisher(
            PointCloud2,
            '/pcd_sampled',
            sensor_qos
        )

        #sub
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
            sensor_qos
        )

        self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            lambda msg: self.sensor_callback(
                name='pcd',
                msg=msg,
                pub=self.pcd_pub
            ),
            sensor_qos
        )

        self.get_logger().info('sampling node started')

    def check_gps_topic(self):
        if self.gps_subscription is not None:
            return
        
        gps_topics = [
            '/ublox_gps_node/fix',
            '/ublox_gps/fix',
            '/ublox/fix',
        ]

        topics = self.get_topic_names_and_types()
        topic_names = [t[0] for t in topics]

        for t in gps_topics:
            if t in topic_names:
                self.get_logger().info(f"GPS topic connected: {t}")
                self.status['gps']['topic'] = t
                self.gps_subscription = self.create_subscription(
                NavSatFix,
                t,
                lambda msg: self.sensor_callback(
                    name='gps',
                    msg=msg,
                    pub=self.gps_pub
                    ),
                10
                )

                self.gps_check_timer.cancel()
                break

    # sensor_data_send
    def sensor_callback(self, name, msg, pub):
        now = self.get_clock().now()
        self.status[name]['last_publish'] = now
        period = Duration(seconds=self.periods[name])

        if self.last_sent_time[name] is None:
            self.last_sent_time[name] = now
        else:
            if now - self.last_sent_time[name] < period:
                return
    
        if name == 'gps':
            distance_threshold = 0.00001

            if self.last_lat is None or self.last_lon is None:
                self.last_lat = msg.latitude
                self.last_lon = msg.longitude
                pub.publish(msg)
                self.last_sent_time[name] = now
                return

            changed = (
                abs(msg.latitude - self.last_lat) > distance_threshold or
                abs(msg.longitude - self.last_lon) > distance_threshold
            )

            if not changed:
                return
            
            self.last_lat = msg.latitude
            self.last_lon = msg.longitude

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

            # if points.shape[0] > self.max_points:
            #     step = max(1, points.shape[0] // self.max_points)
            #     points = points[::step][:self.max_points]

            # 위 코드에서 변경
            if points.shape[0] > self.max_points:
                indices = np.random.choice(
                    points.shape[0],
                    self.max_points,
                    replace=False
                )
                points = points[indices]

            msg = point_cloud2.create_cloud_xyz32(
                header=msg.header,
                points=points
            )

        pub.publish(msg)
        self.last_sent_time[name] = now
        self.get_logger().debug(f"[SEND] {name}")
    

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