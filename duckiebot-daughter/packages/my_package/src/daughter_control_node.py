#!/usr/bin/env python3

import os
import random
import numpy as np
import cv2 as cv
import time
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import LEDPattern, Twist2DStamped, Pixel
from duckietown_msgs.srv import SetCustomLEDPattern
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image
from duckietown_utils.jpg import bgr_from_jpg
from cv_bridge import CvBridge

from duckietown_utils import get_duckiefleet_root
from duckietown_utils.yaml_wrap import yaml_load_file

# homography and computer vision code based off:
# https://github.com/charan223/charan_ros_core/blob/v1/packages/purepursuit/src/purepursuit_controller_node.py
# https://github.com/duckietown-ethz/proj-lfvop/blob/master/packages/dynamic_obstacle_avoidance/src/duckie_detection_node.py


RANDOM_COLORS = ["green", "red", "blue", "white", "yellow", "purple", "cyan", "pink"]

class DaughterControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DaughterControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        rospy.loginfo(os.environ['VEHICLE_NAME'])
        self.veh_name = os.environ['VEHICLE_NAME']

        self.bridge = CvBridge()
        self.resolution=[120,160]
        self.crop_factor = 0.3 #percentage of image cropped from the top
        self.H = self.load_homography()

        # Setup the wheel publisher
        car_topic=f"/{self.veh_name}/joy_mapper_node/car_cmd"
        self.car = rospy.Publisher(car_topic, Twist2DStamped, queue_size=1)

        # Setup the image subscriber
        rospy.Subscriber(f"/{self.veh_name}/camera_node/image/compressed", CompressedImage, self.processImage, queue_size=1)

        # kinematics of car
        self.v = 0
        self.omega = 0
        self.spin_count = 0
        self.drive_count = 0
        self.yellow_low = np.array([25,100,100])
        self.yellow_high = np.array([35,255,255])

    def run(self):
        # change colors randomly every second
        leds_on = ["white", "white", "white", "white", "white"]
        self.set_LEDs(leds_on)
        rate = rospy.Rate(2) # run twice every second
        while not rospy.is_shutdown():
            self.car.publish(self.createCarCmd(self.v, self.omega))
            rate.sleep()

    def load_homography(self):
        '''Load homography (extrinsic parameters)'''
        filename = (f"{get_duckiefleet_root()}/calibrations/camera_extrinsic/{self.veh_name}.yaml")
        if not os.path.isfile(filename):
            rospy.logwarn(f"no extrinsic calibration parameters for {self.veh_name}, trying default")
            filename = (f"{get_duckiefleet_root()}/calibrations/camera_extrinsic/default.yaml")
            if not os.path.isfile(filename):
                rospy.logerr("can't find default either, something's wrong")
            else:
                data = yaml_load_file(filename)
        else:
            rospy.loginfo(f"Using extrinsic calibration of {self.veh_name}")
            data = yaml_load_file(filename)
        return np.array(data[b'homography']).reshape((3,3))

    def point2ground(self, x_arr, y_arr, norm_x, norm_y):
        new_x_arr, new_y_arr = [], []
        for i in range(len(x_arr)):
            u = x_arr[i] * 480/norm_x
            v = y_arr[i] * 640/norm_y
            uv_raw = np.array([u, v])
            uv_raw = np.append(uv_raw, np.array([1]))
            ground_point = np.dot(self.H, uv_raw)
            point = Point()
            x = ground_point[0]
            y = ground_point[1]
            z = ground_point[2]
            point.x = x/z
            point.y = y/z
            point.z = 0.0
            new_x_arr.append(point.x)
            new_y_arr.append(point.y)
        return new_x_arr, new_y_arr

    def pixel2ground(self,pixel):
        uv_raw = np.array([pixel.u, pixel.v])
        uv_raw = np.append(uv_raw, np.array([1]))
        ground_point = np.dot(self.H, uv_raw)
        point = Point()
        x = ground_point[0]
        y = ground_point[1]
        z = ground_point[2]
        point.x = x/z
        point.y = y/z
        point.z = 0.0
        return point

    def processImage(self, image_msg):
        #image_size = [120,160]
        # top_cutoff = 40

        rospy.loginfo("Start processing image")

        start_time = time.time()
        try:
            image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        except ValueError as e:
            rospy.loginfo("image decode error", e)
            return
        
        # Resize and crop image
        hei_original, wid_original = image_cv.shape[0:2]

        if self.resolution[0] != hei_original or self.resolution[1] != wid_original:
            image_cv = cv.resize(image_cv, (self.resolution[1], self.resolution[0]), interpolation=cv.INTER_NEAREST)
        hsv = cv.cvtColor(image_cv, cv.COLOR_BGR2HSV)

        # uses HSV not RGB
        # hsv = cv.cvtColor(image_cv, cv.COLOR_BGR2HSV)
        # Red test
        #hsv_obs_red1 = np.array([0, 50, 20])
        #hsv_obs_red2 = np.array([15, 255, 255])
        #hsv_obs_red3 = np.array([165, 50, 20])
        #hsv_obs_red4 = np.array([180, 255, 255])

        # Yellow test
        """ hsv_obs_red2 = np.array([255,255,204]) # Light yellow1
        hsv_obs_red1 = np.array([51,51,0]) # Dark yellow4
        hsv_obs_red4 = np.array([255,255,224]) # lightyellow
        hsv_obs_red3 = np.array([154,205,50]) # yellowgreen """

        """ hsv_obs_red1 = np.array([0, 140, 100]) # Green
        hsv_obs_red2 = np.array([15, 255, 255]) # Blue
        hsv_obs_red3 = np.array([165, 140, 100]) # Brown/tan
        hsv_obs_red4 = np.array([180, 255, 255]) # Brighter blue """

        #bw1 = cv.inRange(hsv, hsv_obs_red1, hsv_obs_red2)
        #bw2 = cv.inRange(hsv, hsv_obs_red3, hsv_obs_red4)
        #bw = cv.bitwise_or(bw1, bw2)
        mask = cv.inRange(hsv, self.yellow_low, self.yellow_high)
        #cnts = cv.findContours(bw.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
        #rospy.loginfo(len(cnts))
        #rospy.loginfo(cnts)
        #cv.imshow(bw) # maybe will display something??????

        #initalize parameters for blob detector
        #minInertiaRatio is especially important, it filters out the elongated lane segments
        params = cv.SimpleBlobDetector_Params()
        params.filterByColor = True
        params.blobColor = 255
        params.filterByArea = True
        params.minArea = 40
        params.filterByInertia = True
        params.minInertiaRatio = 0.5 #if high ratio: only compact blobs are detected, elongated blobs are filtered out
        params.filterByConvexity = False
        params.maxConvexity = 0.99
        params.filterByCircularity = False
        params.minCircularity = 0.5
        detector = cv.SimpleBlobDetector_create(params)

        # Detect blobs and draw them in image and in mask
        keypoints = detector.detect(mask)
        #t = cv.drawKeypoints(cv_image_crop, keypoints, cv_image_crop, color=(0, 0, 255), flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #t = cv.drawKeypoints(mask, keypoints, mask, color=(0, 0, 255), flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        locs = []
        if keypoints:
            for key in keypoints:
                duckie_loc_pix = Pixel()
                duckie_loc_pix.u=key.pt[0]
                duckie_loc_pix.v=key.pt[1]+key.size/2+float(self.resolution[1])*self.crop_factor #to compensate for crop

                duckie_loc_world = self.pixel2ground(duckie_loc_pix)
                locs.append(duckie_loc_world)
        rospy.loginfo(locs)

        #    rospy.loginfo('object detected')
        #    red_area = max(cnts, key=cv.contourArea)
        #    (xg,yg,wg,hg) = cv.boundingRect(red_area)
        #    #rospy.loginfo(f"BEFORE X: {[xg, xg+wg]}, BEFORE Y: {[yg, yg+hg]}")
        #    x_arr, y_arr = self.point2ground([xg, xg+wg], [yg, yg+hg], image_size[0], image_size[1])
        #    #rospy.loginfo(f"BOTTOM OF ROBOT X: {x_arr}, Y : {y_arr}")
        #    self.omega = max(min((y_arr[0] + y_arr[1]) / 2, 2), -2)
        #    if x_arr[0] < 0.35:
        #        # too close to object - stop!
        #        self.v = 0.1
        #        rospy.loginfo(f"STOPPING: {self.omega}")
        #    else:
        #        # object detected - head for object!
        #        self.v = 0.5
        #        rospy.loginfo(f"AIMING TOWARDS RED: {self.omega}")
        #    self.spin_count = 0
        #    self.drive_count = 0
        #else:
        #    if self.spin_count > 50 or (self.drive_count > 0 and self.drive_count < 10):
        #        self.omega = 0
        #        self.v = 1
        #        self.spin_count = 0
        #        self.drive_count += 1
        #        rospy.loginfo(f"RUNNING FORWARD: {self.drive_count}")
        #    else:
        #        # no objects detected - spin and look for objects?
        #        self.omega = 2
        #        self.v = 0.3
        #        self.spin_count += 1
        #        self.drive_count = 0
        #        rospy.loginfo(f"SPINNING: {self.spin_count}")
        ##rospy.loginfo(f"Time to process: {time.time() - start_time}")

    
    def set_LEDs(self, color_list=None):
        led_service = f"/{self.veh_name}/led_emitter_node/set_custom_pattern"
        # rospy.wait_for_service(led_service)

        if color_list == None:
            color_list = random.choices(RANDOM_COLORS, k = 5)
        try:
            service = rospy.ServiceProxy(led_service, SetCustomLEDPattern)
            msg = LEDPattern()
            msg.color_list = color_list
            msg.color_mask = [1, 1, 1, 1, 1]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
            service(msg)
        except rospy.ServiceException as e:
            print (f"LED Service call failed: {e}")
    

    def createCarCmd(self, v, omega):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = v
        msg.omega = omega
        return msg

    
    def onShutdown(self):
        """Shutdown procedure.
        Publishes a zero velocity command at shutdown."""

        # MAKE SURE THAT THE LAST WHEEL COMMAND YOU PUBLISH IS ZERO,
        # OTHERWISE YOUR DUCKIEBOT WILL CONTINUE MOVING AFTER
        # THE NODE IS STOPPED

        self.car.publish(self.createCarCmd(0, 0))

        leds_off = ["switchedoff", "switchedoff", "switchedoff", "switchedoff", "switchedoff"]
        self.set_LEDs(leds_off)

        #super(DaughterControlNode, self).onShutdown()

if __name__ == '__main__':
    # create the node
    node = DaughterControlNode(node_name='daughter_control_node')
    # run node
    node.run()
    node.onShutdown()
    # keep spinning
    rospy.spin()
