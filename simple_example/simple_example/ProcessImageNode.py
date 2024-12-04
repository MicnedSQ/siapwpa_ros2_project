import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
from centroids_msg.msg import Centroids

class ProcessImageNode(Node):
  def __init__(self):
    super().__init__("process_image_node")

    self.image_subscription = self.create_subscription(Image, "/front_camera", self.image_callback, 10)

    self.publisher = self.create_publisher(Centroids, "centroids", 10)

    self.bridge = CvBridge()

    self.frame = None

  def image_callback(self, msg):
    try:
      self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

      birdseye = self.perspectiveWarp()

      self.processImage(birdseye)

      self.display_image()
    except Exception as e:
      self.get_logger().info('Error processing image: %s' % str(e))

  def send_msg(self, centroids_list):
    msg = Centroids()
    msg.length = len(centroids_list)

    flattened_centroids = [coord for centroid in centroids_list for coord in centroid]

    msg.centroids = flattened_centroids
    self.publisher.publish(msg)

  def display_image(self):
    cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
    cv2.imshow("Frame", self.frame)
    # cv2.imwrite("src/test1.png", self.frame)
    cv2.waitKey(1)

  def perspectiveWarp(self):
    src = np.float32([
        [520, 560], 
        [1300, 560],
        [0, 840], 
        [1920, 840]
        ])

    dst = np.float32([
        [0, 0],
        [1920, 0],
        [0, 1080],
        [1920, 1080]
        ])

    img_size = (self.frame.shape[1], self.frame.shape[0])

    matrix = cv2.getPerspectiveTransform(src, dst)
    birdseye = cv2.warpPerspective(self.frame, matrix, img_size)

    # cv2.namedWindow("Birdseye", cv2.WINDOW_NORMAL)
    # cv2.imshow("Birdseye", birdseye)

    return birdseye

  def processImage(self, birdseye):
    hsv = cv2.cvtColor(birdseye, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([27, 120, 41])
    upper_yellow = np.array([30, 255, 55])
    binary_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    

    kernel = np.ones((11, 11), np.uint8)
    binary_mask = cv2.dilate(binary_mask, kernel, iterations=2)
    contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    out_img = np.dstack((binary_mask, binary_mask, binary_mask)) * 255

    centroids_list = []

    counter = 0 
    for i, contour in enumerate(contours):
      area = cv2.contourArea(contour)
      if area > 10000:
        counter += 1
        M = cv2.moments(contour)
        if M['m00'] != 0:
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])
          centroids_list.append([cx, cy])
        else:
          cx, cy = 0, 0
        cv2.circle(out_img, (cx, cy), 5, (255, 0, 0), -1)
        cv2.drawContours(out_img, [contour], -1, (0, 255, 0), 2)

    self.send_msg(centroids_list)

    self.get_logger().info('NO lines: %s' % str(counter))

    cv2.namedWindow("Birdseye out_img", cv2.WINDOW_NORMAL)
    cv2.imshow("Birdseye out_img", out_img)
    cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ProcessImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
