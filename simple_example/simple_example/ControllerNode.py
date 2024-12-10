import rclpy
from rclpy.node import Node
from centroids_msg.msg import Centroids
from geometry_msgs.msg import Twist

class ControllerNode(Node):
  def __init__(self):
    super().__init__("controller_node")
    
    self.centroids_subscription = self.create_subscription(Centroids, "/centroids", self.steer_callback, 10)

    self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    

  def steer_callback(self, msg):
    output_cmd_vel = Twist()
    length = msg.length
    centroids = msg.centroids

    centroids_pairs = [(centroids[i], centroids[i+1]) for i in range(0, len(centroids), 2)]
    
    centroid_left = [0, 0]
    centroid_right = [1920, 1080]
    left = False
    right = False

    for centroid in centroids_pairs:
        if centroid[0] < (1920 / 2):
           if centroid[0] > centroid_left[0]:
              if centroid[1] > (1080 / 2) - 100:
                centroid_left = centroid
                left = True
        else:
           if centroid[0] < centroid_right[0]:
              if centroid[1] > (1080 / 2) - 100:
                centroid_right = centroid
                right = True

    if left and right:
      center = (centroid_left[0] + centroid_right[0]) / 2
    elif left:
       center = centroid_left[0] + 100
    else:
       center = centroid_right[0] - 100

    screen_center = 1920 / 2

    offset = center - screen_center

    steering_factor = 0.01
    output_cmd_vel.angular.z = -steering_factor * offset

    output_cmd_vel.linear.x = 8.0

    self.cmd_vel_publisher.publish(output_cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
