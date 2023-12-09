# Computer Vision tasks using ROS1-noetic
This branch of the repository deals with computer vision tasks in autonomous vehicles and implementing them using ROS noetic.

# Setup the enviroment for ROS
You must first clone the repository, then change your terminal directory to its path. Proceed to enter the following commands into the terminal:


```bash
catkin_make # note that catkin_make can only be done in the workspace directory
source devel/setup.bash
```
Now you are ready to run our work!

# Object Detection
We're using OAK-D camera and YOLO as our object detection method. You can run the code with the following command:

```bash
rosrun oakd spatial.py
```

This will result in two messages being sent to ROS, an array of detected objects, one with classes, the other with coordiantes.
You can view the output from terminal or by running rviz to see realtime object detection.

```bash
rviz
```

After launching rviz, select the add button to add an image visualization, then change the image topic to /oakd/camera/image.