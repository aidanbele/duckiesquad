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
from sensor_msgs.msg import CompressedImage
from duckietown_utils.jpg import bgr_from_jpg
from cv_bridge import CvBridge

from duckietown_utils import get_duckiefleet_root
from duckietown_utils.yaml_wrap import yaml_load_file

# homography and computer vision code based off:
# https://github.com/charan223/charan_ros_core/blob/v1/packages/purepursuit/src/purepursuit_controller_node.py
# https://github.com/duckietown-ethz/proj-lfvop/blob/master/packages/dynamic_obstacle_avoidance/src/duckie_detection_node.py


RANDOM_COLORS = ["green", "red", "blue", "white", "yellow", "purple", "cyan", "pink"]
bridge = CvBridge()

class MotherControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MotherControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        rospy.loginfo(os.environ['VEHICLE_NAME'])
        self.veh_name = os.environ['VEHICLE_NAME']
        self.H = self.load_homography()

        # Setup the wheel publisher
        car_topic=f"/{self.veh_name}/joy_mapper_node/car_cmd"
        self.car = rospy.Publisher(car_topic, Twist2DStamped, queue_size=1)

        # Setup the image subscriber
        ing_topic = f"/{self.veh_name}/camera_node/image/compressed"
        rospy.Subscriber(ing_topic, CompressedImage, callback=self.processImage, queue_size=1)

        # kinematics of car
        self.v = 0
        self.omega = 0

    def run(self):
        # change colors randomly every second
        leds_on = ["green", "green", "green", "green", "green"]
        self.set_LEDs(leds_on)
        rate = rospy.Rate(2) # run twice every second
        while not rospy.is_shutdown():
            self.car.publish(self.createCarCmd(self.v, self.omega))
            self.set_LEDs(leds_on)
            rate.sleep()

    # homography and computer vision code based off:
    # https://github.com/charan223/charan_ros_core/blob/v1/packages/purepursuit/src/purepursuit_controller_node.py
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
        image_size = [120,160]
        # top_cutoff = 40

        #rospy.loginfo("Start processing image")

        start_time = time.time()
        try:
            image_cv = bgr_from_jpg(image_msg.data)
        except ValueError as e:
            rospy.loginfo("image decode error", e)
            return
        
        # Resize and crop image
        hei_original, wid_original = image_cv.shape[0:2]

        if image_size[0] != hei_original or image_size[1] != wid_original:
            image_cv = cv.resize(image_cv, (image_size[1], image_size[0]), interpolation=cv.INTER_NEAREST)

        # uses HSV not RGB - we use a bitwise or here because the red color specturm wraps around
        hsv = cv.cvtColor(image_cv, cv.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([350,55,100])

        mask = cv.inRange(hsv, lower_black, upper_black)

        params = cv.SimpleBlobDetector_Params()
        params.filterByColor = True
        params.blobColor = 255
        params.filterByArea = True
        params.minArea = 40
        params.filterByInertia = True
        params.minInertiaRatio = 0.35 #if high ratio: only compact blobs are detected, elongated blobs are filtered out
        params.filterByConvexity = False
        params.maxConvexity = 0.99
        params.filterByCircularity = False
        params.minCircularity = 0.5
        detector = cv.SimpleBlobDetector_create(params)

        # Detect blobs and draw them in image and in mask
        keypoints = detector.detect(mask)

        locs = []
        if keypoints:
            for key in keypoints:
                locs.append({"x": key.pt[0], "y": key.pt[0], "size": key.size})
        if len(locs) > 0:
            largest_loc = locs[0]
            for loc in locs:
                if loc["size"] > largest_loc["size"]:
                    largest_loc = loc
            rospy.loginfo(f"Found: {len(locs)} locs")
            rospy.loginfo(largest_loc)
            self.omega = -max(min((90 - largest_loc["x"]) / 45, 2), -2) # negated so we go away from zones
            if largest_loc["size"] > 70:
                self.v = -1
                self.omega = 0
            elif largest_loc["size"] > 15:
                self.v = 0.1
            elif largest_loc["size"] > 7:
                self.v = 0.15
            else:
                self.v = 0.2
            
        else:
            rospy.loginfo("Nothing found")
            self.v = 0.25


        '''
        cnts = cv.findContours(bw.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]

        if len(cnts)>1:
            rospy.loginfo('object detected')
            red_area = max(cnts, key=cv.contourArea)
            (xg,yg,wg,hg) = cv.boundingRect(red_area)
            #rospy.loginfo(f"BEFORE X: {[xg, xg+wg]}, BEFORE Y: {[yg+hg, yg+hg]}")
            x_arr, y_arr = self.point2ground([xg, xg+wg], [yg, yg+hg], image_size[0], image_size[1])
            rospy.loginfo(f"BOTTOM OF ROBOT X: {x_arr}, Y : {y_arr}")
            self.omega = -max(min(y_arr[0], 2), -2)
            if x_arr[0] < 0.15:
                # object detected close to front of car
                rospy.loginfo("REVERSE")
                self.v = -0.1
            elif x_arr[0] < 0.35:
                # object detected close to front of car
                rospy.loginfo("SLOWER")
                self.v = 0.15
            elif x_arr[0] < 0.9:
                # object detected close to front of car
                rospy.loginfo("SLOW")
                self.v = 0.2
            else:
                # object detected, but its far away
                self.omega = 0
                self.v = 0.25
        else:
            # no objects detected
            self.omega = 0
            self.v = 0.25
        rospy.loginfo(f"Time to process: {time.time() - start_time}")
        '''

    
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

        #super(MotherControlNode, self).onShutdown()

if __name__ == '__main__':
    # create the node
    node = MotherControlNode(node_name='daughter_control_node')
    # run node
    node.run()
    node.onShutdown()
    # keep spinning
    rospy.spin()
