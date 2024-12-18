import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
from centroids_msg.msg import Centroids
from skimage.morphology import skeletonize
from fil_finder import FilFinder2D
import astropy.units as u

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
      resized_frame = cv2.resize(birdseye, (960, 540))
      self.processImage(resized_frame)

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
    bounding_boxes = []

    counter = 0 
    for i, contour in enumerate(contours):
      area = cv2.contourArea(contour)
      if area > 10000:
        counter += 1
        # M = cv2.moments(contour)

        x, y, w, h = cv2.boundingRect(contour)

        bounding_boxes.append([x, y, w, h])

        # if M['m00'] != 0:
        #   cx = int(M['m10'] / M['m00'])
        #   cy = int(M['m01'] / M['m00'])
        #   centroids_list.append([cx, cy])
        # else:
        #   cx, cy = 0, 0
        # cv2.circle(out_img, (cx, cy), 5, (255, 0, 0), -1)
        cv2.drawContours(out_img, [contour], -1, (0, 255, 0), cv2.FILLED)

    gray = cv2.cvtColor(out_img, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)

    binary_bool = binary > 0
    skeleton = skeletonize(binary_bool).astype(np.uint8) * 255
    fil = FilFinder2D(skeleton, distance=250 * u.pc, mask=skeleton)
    fil.preprocess_image(flatten_percent=85)
    fil.create_mask(border_masking=True, verbose=False, use_existing_mask=True)
    fil.medskel(verbose=False)
    fil.analyze_skeletons(branch_thresh=40* u.pix, skel_thresh=10 * u.pix, prune_criteria='length')

    cv2.imshow("Test", fil.skeleton, cmap='gray')
    # cv2.contour(fil.skeleton_longpath, colors='r')
    cv2.waitKey(1)


    branch_points = []

    for box in bounding_boxes:
      x_start, y_start, width, height = box
      x_end = x_start + width
      y_end = y_start + height

      roi = skeleton[y_start:y_end, x_start:x_end]

      padded_roi = np.pad(roi, pad_width=1, mode='constant', constant_values=0)

      kernel = np.ones((3, 3), dtype=np.uint16)
      neighborhood_sum = cv2.filter2D(padded_roi.astype(np.uint16), -1, kernel)[1:-1, 1:-1]

      branch_point_mask = (roi == 255) & (neighborhood_sum > 3 * 255)

      # cv2.imshow("Neighborhood Sum", (branch_point_mask * 255).astype(np.uint8))

      branch_point_coords = np.argwhere(branch_point_mask)

      branch_point_coords += [y_start, x_start]

      branch_points.extend(branch_point_coords.tolist())

    for y, x in branch_points:
        if x > 1920 / 2:
          cv2.circle(out_img, (x, y), 5, (0, 0, 255), -1)

          if x + 1 < skeleton.shape[1] and skeleton[y, x + 1] == 255:
              for i in range(x + 1, skeleton.shape[1]):
                  if skeleton[y, i] == 255:
                      if i == x + 1:
                        continue
                      skeleton[y, i] = 0
                  else:
                      break
          if x + 1 < skeleton.shape[1] and y - 1 >= 0 and skeleton[y - 1, x + 1] == 255:
            i = 1
            while x + i < skeleton.shape[1] and y - i >= 0:
                if skeleton[y - i, x + i] == 255:
                    skeleton[y - i, x + i] = 0
                else:
                    break
                i += 1
        else:
          cv2.circle(out_img, (x, y), 5, (0, 0, 255), -1)

          if x - 1 >= 0 and skeleton[y, x - 1] == 255:
              for i in range(x - 1, -1, -1):
                  if skeleton[y, i] == 255:
                      if i == x - 1:
                          continue
                      skeleton[y, i] = 0
                  else:
                      break

          if x - 1 >= 0 and y - 1 >= 0 and skeleton[y - 1, x - 1] == 255:
              i = 1
              while x - i >= 0 and y - i >= 0:
                  if skeleton[y - i, x - i] == 255:
                      skeleton[y - i, x - i] = 0
                  else:
                      break
                  i += 1
           
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(skeleton)

    areas = stats[:, cv2.CC_STAT_AREA]
    sorted_indices = np.argsort(areas)[::-1]

    largest_components_mask = np.zeros_like(skeleton)

    # centroid_lst = []
    
    for i in sorted_indices[1:3]:
        component_label = i
        largest_components_mask = np.zeros_like(skeleton)
        largest_components_mask[labels == component_label] = 255

        M = cv2.moments(largest_components_mask)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            centroids_list.append((cx * 2, cy * 2))
            
            cv2.circle(out_img, (cx, cy), 10, (0, 0, 255), -1)
        else:
            centroids_list.append((0, 0))

    skeleton_with_largest_components = cv2.bitwise_and(skeleton, largest_components_mask)

    labels_colored = cv2.applyColorMap(skeleton_with_largest_components.astype(np.uint8), cv2.COLORMAP_JET)

    cv2.namedWindow("Largest Connected Components", cv2.WINDOW_NORMAL)
    cv2.imshow("Largest Connected Components", labels_colored)
    cv2.waitKey(1)

    cv2.namedWindow("Skeletonized Image", cv2.WINDOW_NORMAL)
    cv2.imshow("Skeletonized Image", skeleton)
    cv2.waitKey(1)
    self.send_msg(centroids_list)

    cv2.namedWindow("Birdseye out_img", cv2.WINDOW_NORMAL)
    cv2.imshow("Birdseye out_img", out_img)
    cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ProcessImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
