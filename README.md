# ros_ai

This tos package contains two main node:

-steer

   This node applies deep learning model that nvidia end-to-end self driving car paper offers
  
-detector
   
   This node tries to detect pedestrians and other object from image using a keras based tiny yolo model
 
To run:

 
      roslaunch ros_ai test_yolo.launch


preview of results:

https://www.youtube.com/watch?v=fOJYagaZdyY
  
Beside these there is a augmentation.py file which is not a ros node for augment images to retrain model.

## installation

catkin_ws is assumed as your workspace

    cd ~/cakin_ws/src
  
    git clone https://github.com/muhammetbozkurt/ros_ai.git
  
    cd ..
  
    catkin_make
 
 
Dependancies:

   - Opencv
   - Keras 2.2.0   
   - tensorflow 1.6.0
   - Turtlebot
