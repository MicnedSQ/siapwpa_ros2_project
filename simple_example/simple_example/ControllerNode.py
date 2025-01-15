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
    self.ki = 0.001
    self.kd = 0.003

    self.previous_error = 0.0
    self.integral = 0.0

    self.start_time = self.get_clock().now()


  def steer_callback(self, msg):
    output_cmd_vel = Twist()
    length = msg.length
    centroids = msg.centroids

    centroids_pairs = [(centroids[i], centroids[i+1]) for i in range(0, len(centroids), 2)]
    
    # centroid_left = [0, 0]
    # centroid_right = [1920, 1080]
    # left = False
    # right = False

    # for centroid in centroids_pairs:
    #     if centroid[0] < (1920 / 2):
    #        if centroid[0] > centroid_left[0]:
    #           if centroid[1] > (1080 / 2) - 100:
    #             centroid_left = centroid
    #             left = True
    #     else:
    #        if centroid[0] < centroid_right[0]:
    #           if centroid[1] > (1080 / 2) - 100:
    #             centroid_right = centroid
    #             right = True

    # center = (centroid_left[0] + centroid_right[0]) / 2

    
    # if left and right:
    #     center = (centroid_left[0] + centroid_right[0]) / 2
    # elif left:
    #     center = (centroid_left[0] + screen_center) / 2
    # elif right:
    #     center = (centroid_right[0] + screen_center) / 2
    # else:
    #     center = screen_center


    center = centroids_pairs[0][0]

    screen_center = 1920 / 2

    error = center - screen_center

    time_delta = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    self.integral += error
    max_integral = 1000.0
    self.integral = max(min(self.integral, max_integral), -max_integral)
    derivative = error - self.previous_error
    self.previous_error = error

    angular_z = -(self.kp * error + self.ki * self.integral * time_delta + self.kd * derivative / time_delta)

    output_cmd_vel.angular.z = angular_z
    output_cmd_vel.linear.x = 15.0
    self.cmd_vel_publisher.publish(output_cmd_vel)

    # self.get_logger().info('Time now: %s' % str(time_delta))
    self.start_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
