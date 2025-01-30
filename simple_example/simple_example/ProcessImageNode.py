import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from centroids_msg.msg import Centroids
from skimage.morphology import skeletonize


def prepareYellowMask(img):
  hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  lower_yellow = np.array([27, 120, 41])
  upper_yellow = np.array([30, 255, 55])
  binary_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
  kernel = np.ones((11, 11), np.uint8)
  binary_mask = cv2.dilate(binary_mask, kernel, iterations=2)
  return binary_mask


def extractLargeAreas(contours, img):
  bounding_boxes = []
  counter = 0 
  for i, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    if area > 10000:
      counter += 1
      x, y, w, h = cv2.boundingRect(contour)
      bounding_boxes.append([x, y, w, h])
      cv2.drawContours(img, [contour], -1, (0, 255, 0), cv2.FILLED)
  return bounding_boxes


def skeletonizeImage(img):
  gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  _, binary = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
  binary_bool = binary > 0
  skeleton = skeletonize(binary_bool).astype(np.uint8) * 255
  return skeleton


def findBranchPoints(bounding_boxes, skeleton):
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
    branch_point_coords = np.argwhere(branch_point_mask)

    branch_point_coords += [y_start, x_start]

    branch_points.extend(branch_point_coords.tolist())
  return skeleton, branch_points


def deleteBranchPoints(branch_points, skeleton, img):
  for y, x in branch_points:
    if x > 1920 / 2:
      cv2.circle(img, (x, y), 5, (0, 0, 255), -1)

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
      cv2.circle(img, (x, y), 5, (0, 0, 255), -1)

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
  return skeleton


def calculateCentroids(skeleton):
  centroids_list = []
  num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(skeleton)
  areas = stats[:, cv2.CC_STAT_AREA]
  sorted_indices = np.argsort(areas)[::-1]

  largest_components_mask = np.zeros_like(skeleton)

  for i in sorted_indices[1:num_labels]:
      component_label = i
      largest_components_mask = np.zeros_like(skeleton)
      largest_components_mask[labels == component_label] = 255

      M = cv2.moments(largest_components_mask)
      if M['m00'] != 0:
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])
          centroids_list.append((cx * 2, cy * 2))
  return centroids_list


def selectCentroids(centroids_list):
  centroid_left_x = 0
  centroid_left_y = 0
  centroid_right_x = 1920
  centroid_right_y = 1080

  for cx, cy in centroids_list:
    if (cx < 1920 / 2 and cx > centroid_left_x):
      centroid_left_x = cx
      centroid_left_y = cy
    if (cx > 1920 / 2 and cx < centroid_right_x):
      centroid_right_x = cx
      centroid_right_y = cy
  return centroid_left_x, centroid_left_y, centroid_right_x, centroid_right_y


class ProcessImageNode(Node):
  def __init__(self):
    super().__init__("process_image_node")

    # Utworzenie subskrybcji do topica przechowującego feed z kamery z Gazebo - /front_camera
    self.image_subscription = self.create_subscription(Image, "/front_camera", self.image_callback, 10)

    # Utworzenie publishera na customowy topic /centroids, wysyłający informacje o środkach linii
    # do Node'a sterującego pojazdem
    self.publisher = self.create_publisher(Centroids, "centroids", 10)

    # Konwerter z ROS2 do OpenCV
    self.bridge = CvBridge()

    # Bufor na ramkę z kamery
    self.frame = None

  # Główna funkcja wykonująca operacje na obrazie. Callback topicu /front_camera pobierający 
  # obraz z feedu z kamery. Obraz jest następnie przekształcany w taki sposób, aby był to widok
  # "z lotu ptaka" (self.perspectiveWarp()). Następnie na obrazie wykrywane są linie oraz 
  # obliczane są środki linii (self.processImage(resized_frame)).
  def image_callback(self, msg):
    try:
      # Pobranie pojedynczej ramki z message'u
      self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

      # Przekształcenie obrazu na obraz "z lotu ptaka"
      birdseye = self.perspectiveWarp()

      # Zmniejszenie dwukrotne rozdzielczości obrazu w celu optymalizacji
      resized_frame = cv2.resize(birdseye, (960, 540))

      # Wykrywanie linii, obliczanie centroidów linii oraz wysyłanie ich na topic /centroids
      self.processImage(resized_frame)

    except Exception as e:
      self.get_logger().info('Error processing image: %s' % str(e))

  # Callback wysyłający message na topic /centroids
  def send_msg(self, centroids_list):
    msg = Centroids()
    msg.length = len(centroids_list)

    flattened_centroids = [coord for centroid in centroids_list for coord in centroid]

    msg.centroids = flattened_centroids
    self.publisher.publish(msg)

  def perspectiveWarp(self):
    # Obszar z obrazu wejściowego, który ma być rzutowany
    src = np.float32([
        [520, 600], 
        [1300, 600],
        [0, 840], 
        [1920, 840]
        ])
    
    # Obszar, na który src ma być rzutowany
    dst = np.float32([
        [0, 0],
        [1920, 0],
        [0, 1080],
        [1920, 1080]
        ])

    # Wyciągnięcie rozmiaru obrazu
    img_size = (self.frame.shape[1], self.frame.shape[0])

    # Rzutowanie obszaru z src na obszar dst
    matrix = cv2.getPerspectiveTransform(src, dst)
    birdseye = cv2.warpPerspective(self.frame, matrix, img_size)

    return birdseye

  def processImage(self, birdseye):
    # Przygotowanie maski w kolorze lini wyznaczającej tor jazdy
    binary_mask = prepareYellowMask(birdseye)

    # Znalezienie konturów wszystkich linii
    contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Przekształcenie z obrazu binarnego na obraz o formacie RGB (ale dalej binarny)
    out_img = np.dstack((binary_mask, binary_mask, binary_mask)) * 255

    # Wyodrębnienie z obrazu tylko dużych obszarów w celu usunięcia zakłóceń
    bounding_boxes = extractLargeAreas(contours, out_img)
    
    # Szkieletyzacja obrazu
    skeleton = skeletonizeImage(out_img)

    # Znajdowanie branch pointów - punktów, w których pojawia się rozwidlenie linii
    skeleton, branch_points = findBranchPoints(bounding_boxes, skeleton)

    # Usunięcie punktów łączących rozgałęzienia
    skeleton = deleteBranchPoints(branch_points, skeleton, out_img)

    # Obliczenie centroidów dla każdego z obszarów
    centroids_list = calculateCentroids(skeleton)

    # Wyszukanie dwóch centroidów na lewo i na prawo od środka obrazu, które są najbliżej środka
    (centroid_left_x, centroid_left_y,
      centroid_right_x, centroid_right_y) = selectCentroids(centroids_list)

    # Dodanie do obrazu wynikowego lini będącej środkiem obrazu
    cv2.line(out_img, (int(1920 / 4), 0), (int(1920 / 4), 1080), (0, 255, 0), 10)

    # Obliczenie centroidu środkowego
    middle_centroid = int((centroid_left_x + centroid_right_x) / 2)

    # Dodanie do obrazu wynikowego centroidu środkowego
    cv2.circle(out_img, (int(middle_centroid / 2), int(1080 / 4)), 10, (255, 0, 0), -1)

    # Dodanie do obrazu wynikowego centroidu lewego
    cv2.circle(out_img, (int(centroid_left_x / 2),int(centroid_left_y / 2)), 10, (255, 0, 0), -1)
    # Dodanie do obrazu wynikowego centroidu prawego
    cv2.circle(out_img, (int(centroid_right_x / 2), int(centroid_right_y / 2)), 10, (0, 0, 255), -1)

    # Zapisanie do tablicy centroidów, do wysłania na customowym topicu /centroids
    centroids_to_send = [(centroid_left_x, centroid_left_y), (centroid_right_x, centroid_right_y)]

    # Wysłanie customowego topica /centroids
    self.send_msg(centroids_to_send)

    # Wyświeltenie obrazu wynikowego
    cv2.namedWindow("Birdseye out_img", cv2.WINDOW_NORMAL)
    cv2.imshow("Birdseye out_img", out_img)
    cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ProcessImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
