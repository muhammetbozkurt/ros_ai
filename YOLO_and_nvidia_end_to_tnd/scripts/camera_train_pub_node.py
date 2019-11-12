#!/usr/bin/env python
import cv2 
import rospy
import pandas as pd

from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime

"""
topic name protocol:
	camera/center/image/raw
"""


class Camera(object):
	"""docstring for Camera"""
	def __init__(self):
		super(Camera, self).__init__()
		self.bridge = CvBridge()
		self.pub = rospy.Publisher("camera/center/image/raw",Image,queue_size=10)
		self.cap = cv2.VideoCapture(0)#assume first camera is center camera 
		#self.rate = rospy.Rate() cv2 ile yavaşlat

	def publish(self):
		if(self.cap.isOpened()):
			ret, img = self.cap.read()
			if(ret):
				try:
					msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
					self.pub.publish(msg)
				except CvBridgeError as e:
					print("---ERROR---\n",e,"\n---ERROR---")
					


class TrainCollector(object):
	"""docstring for TrainCollector"""
	def __init__(self, camera_name, counter = 0):
		super(TrainCollector, self).__init__()
		self.sub_camera = rospy.Subscriber("camera/center/image/raw", Image, self.callback_cam)
		self.bridge = CvBridge()
		self.date = datetime.now()
		self.counter = counter
		self.camera_name = camera_name

		self.sub_can = rospy.Subscriber("can/train", can, self.callback_can)

	def callback_cam(self, img):
		try:
			cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
			cv2.imwrite("~/images/{1}/{0}_{2}"format(self.date,self.camera_name,self.counter),cv_img)

		except CvBridgeError as e:
			print("---ERROR---\n",e,"\n---ERROR---")

	def callback_can(self, can):
		#i will add after decoding CAN bus message
		pass



def main():
	rospy.init_node("collector")
	cam = Camera()
	cam.publish()


if __name__ == '__main__':
	main()		