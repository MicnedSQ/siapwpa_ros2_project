import rclpy
from rclpy.node import Node

class ControllerNode(Node):
  def __init__(self):
    super().__init__("controller_node"):
    
    self.image_subscription = self.create_subscription(Image, "/front_camera", self.image_callback,
                                                       qos_profile=1, callback_group=self._mutex_cb_group)
