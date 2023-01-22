#!/usr/bin/env python3
#ros2 topic pub /command std_msgs/msg/String "data: cross" -1
#ros2 topic pub /command std_msgs/msg/String "data: circle" -1
# "src/kobuki_actions/resource/kobuki_actions.json"
import rclpy
import json
import math
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException

from tf_transformations import euler_from_quaternion



PI = math.pi

class KobukiActionsNode(Node):

    def __init__(self):
        super().__init__("kobuki_actions")
        self.declare_parameter("jsonPath", "src/kobuki_actions/resource/kobuki_actions.json")
        self.jsonPath = self.get_parameter("jsonPath").value
        self.twist_publisher = self.create_publisher(Twist, "/mobile_base/commands/velocity", 10)
        self.string_subscriber = self.create_subscription(String, "/command", self.command_callback, 10)
        self.jsonPath_subscriber = self.create_subscription(String, "/reload_json", self.loadJson, 10)
        
        # open the gestures file and loads it as json
        self.loadJson({})


        # map the basic func
        self.func = {"move":self.move,"rotate":self.rotate,"moveAndRotate":self.moveAndRotate}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self,spin_thread=True)
        self.transform = None
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.01, self.on_timer,callback_group=self.timer_cb_group)

    def command_callback(self, msg):
        if msg.data in self.gestures:  # Check if the gestures is in the gestures file
            commands = self.gestures[msg.data] # Extract the commands
            for command in commands:
                for i in range(command["repetitions"]):
                    self.func[command["name"]](command["args"])
        else:
            self.get_logger().info(msg.data+" gesture is not in json file")

    def move(self,args):
        self.get_logger().info("In move function")
        vel_msg = Twist()
        speed, distance, isForward = args["speed"],args["distance"],args["isForward"]
        rate = self.create_rate(25)

        distance *= (1) if isForward else 1
        #Since we are moving just in x-axis
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        #Checking if the movement is forward or backwards
        vel_msg.linear.x = abs(speed) if isForward else -abs(speed)

        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
    

        t0 = self.get_clock().now().nanoseconds/1000000000
        current_distance = 0.0

        #Loop to move the robot in an specified distance
        while(current_distance < distance):
            self.twist_publisher.publish(vel_msg)
            rate.sleep()
            t1 = self.get_clock().now().nanoseconds/1000000000
            current_distance= speed*(t1-t0)


        #After the loop, stops the robot
        vel_msg.linear.x = 0.0
        self.twist_publisher.publish(vel_msg)
        self.get_logger().info("end move function")

    def rotate(self, args):
        self.get_logger().info("In rotate function")
        vel_msg = Twist()
        speed, angle, clockwise = args["speed"],args["angle"],args["clockwise"]
        deg_deviation = math.radians(1) 
        rate = self.create_rate(45)

        #Converting from angles to radians
        angular_speed = math.radians(speed)
        relative_angle = angle if clockwise else (-1)*angle

        #We wont use linear components
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = - abs(angular_speed) if clockwise else abs(angular_speed)
        # calc the relative angle
        self.on_timer()
        current_angle = self.get_transform()[2]     
        relative_angle = (math.degrees(current_angle) + 360 + relative_angle) % 360
        relative_angle = relative_angle - 360 if relative_angle > 180 else relative_angle
        relative_angle = math.radians(relative_angle) #back to radian between [-PI,PI]

        while(abs(relative_angle-current_angle) > deg_deviation):
            self.twist_publisher.publish(vel_msg)
            rate.sleep()
            current_angle = self.get_transform()[2] #gettint the current angle in radian
            
        rate.destroy()
        #Forcing our robot to stop
        vel_msg.angular.z = 0.0
        self.twist_publisher.publish(vel_msg)
        self.get_logger().info("end rotate function")
    
    def moveAndRotate(self,args):
        self.get_logger().info("In moveAndRotate function")
        vel_msg = Twist()
        speed, distance, isForward, angle, clockwise = args["speed"],args["distance"],args["isForward"],args["angle"],args["clockwise"]
        angle = math.radians(angle)
        rate = self.create_rate(25)

        distance *= (1) if isForward else 1
        #Since we are moving just in x-axis
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        #Checking if the movement is forward or backwards
        vel_msg.linear.x = abs(speed) if isForward else -abs(speed)

        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = - abs(angle) if clockwise else abs(angle)
    

        t0 = self.get_clock().now().nanoseconds/1000000000
        current_distance = 0.0

        #Loop to move the robot in an specified distance
        while(current_distance < distance):
            self.twist_publisher.publish(vel_msg)
            rate.sleep()
            t1 = self.get_clock().now().nanoseconds/1000000000
            current_distance= speed*(t1-t0)


        #After the loop, stops the robot
        vel_msg.linear.x = 0.0
        self.twist_publisher.publish(vel_msg)
        self.get_logger().info("end moveAndRotate function")
    def get_transform(self):
        return euler_from_quaternion([
                    self.transform.transform.rotation.x,
                    self.transform.transform.rotation.y,
                    self.transform.transform.rotation.z,
                    self.transform.transform.rotation.w])

    def on_timer(self):
        flag =True
        while flag:
            try:
                self.transform = self.tf_buffer.lookup_transform(
                "base_link",
                "odom",
                rclpy.time.Time())
                flag = False
            except LookupException as ex:
                self.get_logger().info("LookupException")
    
    def loadJson(self,data):
        try:
            f = open(self.jsonPath)
            self.gestures = json.loads(f.read())
            f.close()
            self.get_logger().info("loaded Json")
        except OSError as ex:
            self.get_logger().info("cant open file")
            return

def main(args=None):
    rclpy.init(args=args)
    print("In the node")
    node = KobukiActionsNode()
    executor = MultiThreadedExecutor(3)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()