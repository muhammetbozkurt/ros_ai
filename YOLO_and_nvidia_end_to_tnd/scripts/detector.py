#!/usr/bin/python

import os

import cv2
import rospy
import rospkg
import Image as Img
import colorsys

import numpy as np
from keras import backend as K
from keras.models import load_model
from keras.layers import Input

from yolo3.model import yolo_eval, yolo_body, tiny_yolo_body
from yolo3.utils import letterbox_image
from keras.utils import multi_gpu_model

from PIL import Image, ImageFont, ImageDraw
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge


class YOLO(object):
    
    def __init__(self):
        path = rospkg.RosPack().get_path("ros_ai")
        print("path ",path)
        self.model_path = "{}/scripts/model.h5".format(path)
        self.classes_path = "{}/scripts/classes.txt".format(path)
        self.score = 0.3
        self.iou = 0.45
        self.model_image_size = (416, 416)
        #self._image_size = (416, 416)
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
            print("anchors: ",  self.anchors )
            print("class num: ",  num_classes)
            print('output_shape = %d' %(self.yolo_model.layers[-1].output_shape[-1]))
            print('num_anchors = %d' % num_anchors)
            print('len = %d' %(len(self.yolo_model.output) * (num_classes + 5)))
            print('len_output = %d' %(len(self.yolo_model.output)))
            assert self.yolo_model.layers[-1].output_shape[-1] == num_anchors/len(self.yolo_model.output) * (num_classes + 5), 'Mismatch between model and given anchor and class sizes'

        print('{} model, anchors, and classes loaded.'.format(model_path))

        # Generate colors for drawing bounding boxes.
        hsv_tuples = [(x / len(self.class_names), 1., 1.)
                      for x in range(len(self.class_names))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
                self.colors))
        np.random.seed(10101)  # Fixed seed for consistent colors across runs.
        np.random.shuffle(self.colors)  # Shuffle colors to decorrelate adjacent classes.
        np.random.seed(None)  # Reset seed to default.

        # Generate output tensor targets for filtered bounding boxes.
        self.input_image_shape = K.placeholder(shape=(2, ))
        if self.gpu_num>=2:
            self.yolo_model = multi_gpu_model(self.yolo_model, gpus=self.gpu_num)
        boxes, scores, classes = yolo_eval(self.yolo_model.output, self.anchors,
                len(self.class_names), self.input_image_shape,
                score_threshold=self.score, iou_threshold=self.iou)
        return boxes, scores, classes

    def detect_image(self, image):

        """if self.model_image_size != (None, None):
            assert self.model_image_size[0]%32 == 0, 'Multiples of 32 required'
            assert self.model_image_size[1]%32 == 0, 'Multiples of 32 required'
            boxed_image = letterbox_image(image, tuple(reversed(self.model_image_size)))"""

        if self.model_image_size != (None, None):
            assert self.model_image_size[0]%32 == 0, 'Multiples of 32 required'
            assert self.model_image_size[1]%32 == 0, 'Multiples of 32 required'
            boxed_image = letterbox_image(image, tuple(reversed(self.model_image_size)))
        else:
            new_image_size = (image.width - (image.width % 32),
                              image.height - (image.height % 32))
            boxed_image = letterbox_image(image, new_image_size)
        image_data = np.array(boxed_image, dtype='float32')

        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

        out_boxes, out_scores, out_classes = self.sess.run(
            [self.boxes, self.scores, self.classes],
            feed_dict={
                self.yolo_model.input: image_data,
                self.input_image_shape: [image.size[1], image.size[0]],
                #K.learning_phase(): 0
            })

        print('Found {} boxes for {}'.format(len(out_boxes), 'img'))

        font = ImageFont.truetype(font='/usr/share/fonts/truetype/freefont/FreeMono.ttf',
                    size=np.floor(8e-2 * image.size[1] + 0.5).astype('int32'))
        thickness = (image.size[0] + image.size[1]) // 300

        for i, c in reversed(list(enumerate(out_classes))):
            predicted_class = self.class_names[c]
            box = out_boxes[i]
            score = out_scores[i]

            label = '{} {:.2f}'.format(predicted_class, score)
            draw = ImageDraw.Draw(image)
            label_size = draw.textsize(label, font)

            top, left, bottom, right = box
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(image.size[1], np.floor(bottom + 0.5).astype('int32'))
            right = min(image.size[0], np.floor(right + 0.5).astype('int32'))
            print(label, (left, top), (right, bottom))

            if top - label_size[1] >= 0:
                text_origin = np.array([left, top - label_size[1]])
            else:
                text_origin = np.array([left, top + 1])

            for i in range(thickness):
                draw.rectangle(
                    [left + i, top + i, right - i, bottom - i],
                    outline=self.colors[c])
            draw.rectangle(
                [tuple(text_origin), tuple(text_origin + label_size)],
                fill=self.colors[c])
            draw.text(text_origin, label, fill=(0, 0, 0), font=font)
            del draw
            
        return out_boxes, image

    def close_session(self):
        self.sess.close()



class PedesterianDetector():
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.publisher = rospy.Publisher("/yolo/point", Point,queue_size=1)
        self.publisher_img = rospy.Publisher("/yolo/image", Image,queue_size=1)
        self.yolo = YOLO()
        self.msg = Point()
        
    def callback(self, data):
        image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
        image = cv2.resize(image,self.yolo.model_image_size)
        image = image[:,:,::-1].copy()
        image = Img.fromarray(image)
        boxes, image= self.yolo.detect_image(image)
        self.publisher.publish(self.msg)
        image = np.array(image)
        image = image[:,:,::-1].copy()
        image = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
        self.publisher_img.publish(image)
    
    def nodender(self):
        self.yolo.close_session()

def main():
    pd = PedesterianDetector()
    rospy.init_node("PedestrianDetector", anonymous=True)
    try:
        print("---------------------------")
        rospy.spin()
    except Exception as e:
        print("Exception:\n", e)
    pd.yolo.close_session()


if __name__ == "__main__":
    main()