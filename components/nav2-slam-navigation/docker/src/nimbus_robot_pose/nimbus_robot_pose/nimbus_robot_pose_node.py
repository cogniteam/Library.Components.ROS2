import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped


class NimbusRobotPoseNode(Node):
    def __init__(self):
        super().__init__('nimbus_robot_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nimbus_robot_pose_pub = self.create_publisher(PoseStamped, '/nimbus_robot_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)

    def publish_pose(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            nimbus_pose = PoseStamped()
            nimbus_pose.header.stamp = self.get_clock().now().to_msg()
            nimbus_pose.header.frame_id = 'map'
            nimbus_pose.pose.position.x = tf_msg.transform.translation.x
            nimbus_pose.pose.position.y = tf_msg.transform.translation.y
            nimbus_pose.pose.position.z = tf_msg.transform.translation.z
            nimbus_pose.pose.orientation = tf_msg.transform.rotation
            self.nimbus_robot_pose_pub.publish(nimbus_pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = NimbusRobotPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
