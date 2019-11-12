import rospy
import cv2

from keras.models import load_model
from custom_msgs.msg import custom, CAN_data
from sensor_msgs.msg import Image as Img
from cv_bridge import CvBridge

MAX_SPEED = 25
MIN_SPEED = 10

IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS = 160, 320, 3


class SteerNode():
    def __init__(self):
        self.bridge = CvBridge()
        self.pub= rospy.Publisher("dest?", custom)
        self.sub_img = rospy.Subscriber("source?", Img, self.callback_steer)
        self.sub_can_data = rospy.Subscriber("source?", CAN_data, self.callback_curr_data)
        self.model = load_model("steer.h5")#nvidia end-to-end self driving car paper
        self.custom = custom()
        
        self.custom.speed = 0
        self.custom.steer_angle = 0
        self.custom. throttle = 0
        self.speed_limit = MAX_SPEED
    
    def callback_steer(self, data):
        img = self.bridge.imgmsg_to_cv2(data)
        img = self.preprocess(img)
        self.custom.steer_angle = self.model.predict(img)
        self.pub_steer.publish(self.custom)
    
    
    def callback_curr_data(self, data):
        if(data.speed > self.speed_limit):
            self.speed_limit = MIN_SPEED
        else:
            self.speed_limit = MAX_SPEED
        self.custom.throttle = 1.0 - self.custom.steer_angle**2 - (data.speed/self.speed_limit)**2
        self.custom.speed = self.speed_limit
        
        
    def preprocess(self, image):
        image = image[60:-25, :, :] # change depending on real cam
        image = cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT), cv2.INTER_AREA)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
        return image
    
    
def main():
    rospy.node_init("steer_node",  anonymous = True)
    node =  SteerNode()
    try:
        rospy.spin()
    except Exception as e:
        print("Exception:\n", e)
