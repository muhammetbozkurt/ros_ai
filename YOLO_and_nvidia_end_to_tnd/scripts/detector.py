#usr/bin/env python

import os

import cv2
import rospy

import numpy as np
from keras import backend as K
from keras.models import load_model
from keras.layers import Input

from yolo3.model import yolo_eval, yolo_body, tiny_yolo_body
from yolo3.utils import letterbox_image
from keras.utils import multi_gpu_model

from custom_msgs.msg import custom
from sensor_msgs.msg import Image as Img
from cv_bridge import CvBridge

class YOLO(object):
    
    def __init__(self):
        self.model_path = "model.h5"
        self.classes_path = "classes.txt"
        self.score = 0.3
        self.iou = 0.45
        self._image_size = (416, 416)
        self.gpu_num = 0
        self.class_names = self._get_class()
        self.anchors = np.array([10.,14., 23.,27., 37.,58., 81.,82., 135.,169., 344.,319.]).reshape(-1, 2)
        self.sess = K.get_session()
        self.boxes, self.scores, self.classes = self.generate()

    def _get_class(self):
        classes_path = os.path.expanduser(self.classes_path)
        with open(classes_path) as f:
            class_names = f.readlines()
        class_names = [c.strip() for c in class_names]
        return class_names


    def generate(self):
        model_path = os.path.expanduser(self.model_path)
        assert model_path.endswith('.h5'), 'Keras model or weights must be a .h5 file.'

        # Load model, or construct model and load weights.
        num_anchors = len(self.anchors)
        num_classes = len(self.class_names)
        is_tiny_version = num_anchors==6 # default setting
        try:
            self.yolo_model = load_model(model_path, compile=False)
        except:
            self.yolo_model = tiny_yolo_body(Input(shape=(None,None,3)), num_anchors//2, num_classes) \
                if is_tiny_version else yolo_body(Input(shape=(None,None,3)), num_anchors//3, num_classes)
            self.yolo_model.load_weights(self.model_path) # make sure model, anchors and classes match
        else:
            print('output_shape = %d' %(self.yolo_model.layers[-1].output_shape[-1]))
            print('num_anchors = %d' % num_anchors)
            print('len = %d' %(len(self.yolo_model.output) * (num_classes + 5)))
            print('len_output = %d' %(len(self.yolo_model.output)))
            assert self.yolo_model.layers[-1].output_shape[-1] == num_anchors/len(self.yolo_model.output) * (num_classes + 5), 'Mismatch between model and given anchor and class sizes'

        print('{} model, anchors, and classes loaded.'.format(model_path))

        # Generate output tensor targets for filtered bounding boxes.
        self.input_image_shape = K.placeholder(shape=(2, ))
        if self.gpu_num>=2:
            self.yolo_model = multi_gpu_model(self.yolo_model, gpus=self.gpu_num)
        boxes, scores, classes = yolo_eval(self.yolo_model.output, self.anchors,
                len(self.class_names), self.input_image_shape,
                score_threshold=self.score, iou_threshold=self.iou)
        return boxes, scores, classes

    def detect_image(self, image):

        if self.model_image_size != (None, None):
            assert self.model_image_size[0]%32 == 0, 'Multiples of 32 required'
            assert self.model_image_size[1]%32 == 0, 'Multiples of 32 required'
            boxed_image = letterbox_image(image, tuple(reversed(self.model_image_size)))
        else:
            new_image_size = (image.width - (image.width % 32),
                              image.height - (image.height % 32))
            boxed_image = letterbox_image(image, new_image_size)
        image_data = np.array(boxed_image, dtype='float32')

        print(image_data.shape)
        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

        out_boxes, out_scores, out_classes = self.sess.run(
            [self.boxes, self.scores, self.classes],
            feed_dict={
                self.yolo_model.input: image_data,
                self.input_image_shape: [image.size[1], image.size[0]],
                K.learning_phase(): 0
            })
        #print('Found {} boxes for {}'.format(len(out_boxes), 'img'))
        return out_boxes

    def close_session(self):
        self.sess.close()





class PedesterianDetector():
    def __init(self):
        self.cv_bridge = CvBridge()
        self.subscriber = rospy.Subscriber("source?", Img, self.callback)
        self.publisher = rospy.Publisher("dest?", custom)
        self.yolo = YOLO()
        self.msg = custom
        
    def callback(self, data):
        image = self.cv_bridge.imgmsg_to_cv2(data)
        if(len(self.yolo.detect_image(image))>0):
            self.msg.int = 1
            self.publisher.publish(self.msg)
        
        else:
            self.msg.int = 0
            self.publisher.publish(self.msg)
    
    def nodender(self):
        self.yolo.close_session()

def main():
    pd = PedesterianDetector()
    rospy.init_node("PedestrianDetector", anonymous=True)
    try:
        rospy.spin()
    except Exception as e:
        print("Exception:\n", e)
    pd.yolo.close_session()
