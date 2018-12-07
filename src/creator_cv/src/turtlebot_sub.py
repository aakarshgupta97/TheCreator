import time
import rospy
import ros_numpy
import numpy as np
import scipy.misc
# ROS Image message
import imutils
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt

from sklearn.neighbors.kde import KernelDensity

import PIL
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()
depth_name = 0
img_name = 0
called_pc = False

depth_images = []

def image_callback(msg):
    global img_name
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        print(cv2_img.shape, type(cv2_img))
        print(np.amax(cv2_img))
        img_name += 1
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 

        # succ = cv2.imwrite('./image_dump/image.jpg', cv2_img)

        cv2_img[:, :, :] = cv2_img[:, :, ::-1]

        im = PIL.Image.fromarray(cv2_img)
        succ = im.save("./image_dump/" + str(img_name) + ".png")
        if succ:
            print("saved somewhere?")

def depth_callback(depth_data):
    try:
        global depth_name
        depth_name += 1
        print("Reached the depth callback!")
        depth_image = bridge.imgmsg_to_cv2(depth_data, "passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        depth_array[np.isnan(depth_array)] = 0
        np.save("model3/depth_arrs/model3_depth" + str(depth_name), depth_array)
        print(depth_array.shape)
        
    except CvBridgeError as e:
        print(e)

def pointcloud_callback(pointcloud):
    global called_pc
    if called_pc is False:
        try:
            print("Reached the pointcloud callback!")
            pc_arr = ros_numpy.point_cloud2.pointcloud2_to_array(pointcloud)
            xyz_array = ros_numpy.point_cloud2.get_xyz_points(pc_arr) 
            np.save("model3/pc_arrs/model_3pc", xyz_array)
            called_pc = True
        except CvBridgeError as e:
            print(e)
    else:
      pass


def main():
    t0 = time.time()
    while not rospy.core.is_shutdown() and not called_pc:
      rospy.init_node('image_listener')
      # Define your image topic
      pointcloud_topic = "/camera/depth_registered/points"
      depth_topic = "/camera/depth_registered/image_raw"
      image_topic = "/camera/rgb/image_raw"

      # Set up your subscriber and define its callback
      try:
          rospy.Subscriber(pointcloud_topic, PointCloud2, pointcloud_callback)
          # rospy.Subscriber(depth_topic, Image, depth_callback)
          # rospy.Subscriber(image_topic, Image, image_callback)
      except ZeroDivisionError:
          break
      
      rospy.rostime.wallsleep(0.1)
    t1 = time.time()
    print "Time Taken:", t1 - t0, "seconds."
    print("Exited The Thing!!!!!!!!!!!!!!!")

if __name__ == '__main__':
    main()
