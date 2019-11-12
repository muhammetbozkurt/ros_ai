import  cv2
import argparser
import numpy as np

from random import randint
from os import listdir

class Label():
    def __init__(self, cls, x, y, width, height):
        self.cls = cls
        self.x = x
        self.y = y
        self.width = width
        self.height = height
    
    def __str__(self):
        return "{} {} {} {} {}\n".format(self.cls, self.x,  self.y, self.width, self.height)


class Image():
    def __init__(self, img_name, dir):
        self.img = cv2.imread("{}/{}".format(dir, img_name))
        self.name = img_name
        self.labels = []
    
    def read_label(self, label_dir):
        with open("{}//{}.txt".format(label_dir, self.name[:-4]), "r") as file:
            lines = file.readlines()
            for line in lines:
                cls,  x,  y,  w, h = line.split(",") #sıra hatalı olabilir
                self.labels.append(Label(float(cls), float(x), float(y), float(w), float(h) ))
    
    def brightness(self):
        hsvImg = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        value = randint(-50, 50)
        vValue = hsvImg[...,2]
        if(value>=0):
            hsvImg[...,2]=np.where((255-vValue)<value,255,vValue+value)
        else:
            hsvImg[...,2]=np.where(vValue>abs(value),255,vValue+value)
        self.img = cv2.cvtColor(hsvImg,  cv2.COLOR_HSV2BGR)


class Augmentor():
    def __init__(self, source_img, source_label, dest_dir = "./output"):
        self.source_img = source_img
        self.source_label = source_label
        self.dest_dir = dest_dir
        self.image_width = None
        self.image_height = None
    
    def reflect(self, img): #img is a Image object
        if(randint(0, 2)<1):
            for label in img.labels:
                label.x = 1 - label.x # check it
        
    def move(self, img):#kontol mekanizmasını ekle daha sonra 
        if(randint(0, 5)<3):
            step_x = randint(-20, 20) / self.image_width #check
            step_y = randint(-20, 20) / self.image_height #check 
            for label in img.labels:
                label.x += step_x
                label.y += step_y
    
    def save(self, img, id):
        cv2.imwrite("{}/{}_{}.txt".format(self.dest_dir, img.name, id), img.img)
        out_label = open("{}/{}_{}.txt".format(self.dest_dir, img.name[:-4], id),"w")
        for label in img.labels:
            out_label.write("{}\n".format(label))

    def handle(self):
        for img_name in listdir(self.source_img):
            temp = Image(img_name, self.source_img)
            temp.read_label(self.source_label)
            temp.brightness()
            self.reflect(temp)
            self.move(temp)
            self.save(temp)

def main():
    parser = argparser.ArgumentParser()
    parser.add_argument("--src_img",  type=str,
        help = "source directory of original images", 
        default = "./images")
    parser.add_argument("--src_label",  type=str, 
        help = "source directory of orginal labels", 
        default = "./labels" )
    parser.add_argument("--dest",  type=str, 
        help = "output directory", 
        default = "./results")
    args = parser.parse_args()
    Aug = Augmentor(args.src_img, args.src_label,  args.dest)
    Aug.handle()

if(__name__ == "__main__"):
    main()
