#!/usr/bin/python

import rospy
import rospkg
import cv2

from keras.models import load_model
from geometry_msgs.msg import Vector3

"""
sending necessary data via vector3 message type 

Vector3.x --> speed

Vector3.y --> steer angle

Vector3.z --> throttle

"/current_CAN_data"(Vector3) topic will be used for getting current CAN data

"/commands"(Vector3) will be used for sending necessary data to drive car

"/camera/center" will be used for getting images

"""

from sensor_msgs.msg import Image as Img
from cv_bridge import CvBridge

MAX_SPEED = 25
MIN_SPEED = 10

IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS = 160, 320, 3


class SteerNode():
    def __init__(self):
        path = rospkg.RosPack().get_path("ros_ai")
        self.bridge = CvBridge()
        self.pub= rospy.Publisher("/commands", Vector3)
        self.sub_img = rospy.Subscriber("/camera/center", Img, self.callback_steer)
        self.sub_can_data = rospy.Subscriber("/current_CAN_data", Vector3, self.callback_curr_data)
        self.model = load_model("{}/scripts/steer.h5".format(path))#nvidia end-to-end self driving car paper
        self.command = Vector3()
        
        self.command.x = 0
        self.command.y = 0
        self.command.z = 0
        self.speed_limit = MAX_SPEED
    
    def callback_steer(self, data):
        img = self.bridge.imgmsg_to_cv2(data)
        img = self.preprocess(img)
        self.command.y = self.model.predict(img)
        self.pub.publish(self.command)
    
    
    def callback_curr_data(self, data):
        if(data.speed > self.speed_limit):
            self.speed_limit = MIN_SPEED
        else:
            self.speed_limit = MAX_SPEED
        self.command.z = 1.0 - self.command.y**2 - (data.speed/self.speed_limit)**2
        self.command.x = self.speed_limit
        
        
    def preprocess(self, image):
        image = image[60:-25, :, :] # change depending on real cam
        image = cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT), cv2.INTER_AREA)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
        return image
    
    
def main():

    rospy.init_node("steer_node",  anonymous = True)
    SteerNode()
    try:
        rospy.spin()
    except Exception as e:
        print("Exception:\n", e,"\n")
