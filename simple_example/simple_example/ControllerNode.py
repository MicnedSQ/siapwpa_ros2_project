import rclpy
from rclpy.node import Node
from centroids_msg.msg import Centroids
from geometry_msgs.msg import Twist

class ControllerNode(Node):
  def __init__(self):
    super().__init__("controller_node")
    
    # Utworzenie subskrybcji do customowego topica /centroids
    self.centroids_subscription = self.create_subscription(Centroids, "/centroids", self.steer_callback, 10)

    # Utworzenie publishera na topic /cmd_vel, który steruje pojazdem w Gazebo
    self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    # Inicjalizacja parametrów regulatora PID
    self.kp = 0.0045
    self.ki = 0.0005
    self.kd = 0.0035

    # Inicjalizacja zmiennych potrzebnych do poprawnego działania regulatora PID
    self.previous_error = 0.0
    self.integral = 0.0
    self.start_time = self.get_clock().now()
    self.previous_angular_z = 0.0

    # Obliczenie środka obrazu
    self.screen_center = 1920 / 2

    # Inicjalizacja zmiennej przechowującej prędkość pojazdu
    self.speed = 15.0

  # Callback dla subscribera topicu /centroids
  def steer_callback(self, msg):
    # Utworzenie obiektu sterującego pojazdem
    output_cmd_vel = Twist()

    # Zaczytanie centroidów do zmiennych w callbacku
    centroids = msg.centroids
    centroid_left_x = centroids[0]
    centroid_left_y = centroids[1]
    centroid_right_x = centroids[2]
    centroid_right_y = centroids[3]

    # Odległość od lini w pixelach w przypadku wykrycia tylko jednej lini
    offset = 500
    center = 0

    # Sprawdzenie czy wykryte zostały dwie linie czy jedna. W przypadku wykrycia jednej linii,
    # środek, którego pojazd ma się trzymać jest zdefiniowany jako odległość o "offset" od
    # jednej z lini. Jeśli są wykryte oba centroidy, to środkiem jest średnia arytmetyczna pomiędzy
    # dwoma centroidami
    if centroid_left_x == 0 and centroid_left_y == 0:
      center = centroid_right_x - offset
    elif centroid_right_x == 1920 and centroid_right_y == 1080:
      center = centroid_left_x + offset
    else:
      center = int((centroid_left_x + centroid_right_x) / 2)    

    # Obliczenie błędu do regulatora PID
    error = center - self.screen_center

    # Regulator PID
    time_delta = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
    self.integral += error
    max_integral = 20000.0
    self.integral = max(min(self.integral, max_integral), -max_integral)
    derivative = error - self.previous_error
    self.previous_error = error
    
    # Obliczenie prędkości kątowej do sterowania skrętem kół
    angular_z = -(self.kp * error + self.ki * self.integral * time_delta + self.kd * derivative / time_delta)

    # Zapisanie obliczonych wartości do message'u w topicu sterującego pojazdem - /cmd_vel
    output_cmd_vel.angular.z = angular_z
    output_cmd_vel.linear.x = self.speed
    self.cmd_vel_publisher.publish(output_cmd_vel)

    # Reset czasu
    self.start_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
