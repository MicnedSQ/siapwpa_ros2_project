import rclpy
from rclpy.node import Node
from centroids_msg.msg import Centroids
from geometry_msgs.msg import Twist

class ControllerNode(Node):
  def __init__(self):
    super().__init__("controller_node")
    
    self.centroids_subscription = self.create_subscription(Centroids, "/centroids", self.steer_callback, 10)

    self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    self.kp = 0.0045
    self.ki = 0.0005
    self.kd = 0.0035

    self.previous_error = 0.0
    self.integral = 0.0

    self.start_time = self.get_clock().now()

    self.previous_angular_z = 0.0


  def steer_callback(self, msg):
    output_cmd_vel = Twist()
    length = msg.length
    centroids = msg.centroids

    centroid_left_x = centroids[0]
    centroid_left_y = centroids[1]
    centroid_right_x = centroids[2]
    centroid_right_y = centroids[3]

    center = 0
    offset = 500

    if centroid_left_x == 0 and centroid_left_y == 0:
      center = centroid_right_x - offset
      # self.get_logger().info('Only right line visible')
    elif centroid_right_x == 1920 and centroid_right_y == 1080:
      center = centroid_left_x + offset
      # self.get_logger().info('Only left line visible')
    else:
      center = int((centroid_left_x + centroid_right_x) / 2)    

    screen_center = 1920 / 2

    error = center - screen_center

    time_delta = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    self.integral += error
    max_integral = 20000.0
    self.integral = max(min(self.integral, max_integral), -max_integral)
    derivative = error - self.previous_error
    self.previous_error = error
    
    angular_z = -(self.kp * error + self.ki * self.integral * time_delta + self.kd * derivative / time_delta)

    output_cmd_vel.angular.z = angular_z
    self.get_logger().info(f'{self.integral}')
    output_cmd_vel.linear.x = 15.0
    self.cmd_vel_publisher.publish(output_cmd_vel)

    self.start_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
