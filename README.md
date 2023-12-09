# YOLO Custom Training for OAK-D Camera
This branch of the repository deals with training custom dataset for our graduation project.

The weighted result from training will be uploaded on [Luxonis Tools](https://tools.luxonis.com/) to compile a
blob and JSON configuration for use with our OAK-D camera.

Our notebooks
- [YOLOv6n-udacity](https://www.kaggle.com/code/bumbleboss/yolov6-training)
- [YOLOv6n-udacity+pedestrians](https://www.kaggle.com/code/bumbleboss/yolov6-training-pedestrians)

Outsource model
- [YOLOv7](https://github.com/WongKinYiu/yolov7/releases)

# Custom Trained Models
### YOLOv6n-udacity
This model works well for all classes except for pedestrians, for which there is very poor recognition. The reason for this is that pedestrian labels are underdefined and car labels are overdefined. We attempted to solve this issue in the following section by adding more pedestrian images and labels to our udacity dataset.

### YOLOv6n-udacity+pedestrians
The recognition of pedestrians improved as more images were added to the udacity dataset, but problems with other classes also surfaced. These objects are not recognized when they are close to the camera, but their recognition improves when increasing the distance.

# Pretrained Models
### YOLOv7-tiny
We utilized the Yolo v7-tiny that is trained on COCO dataset whilst removing the classes that we won't need for our autonomous driving journey.

# Feedback
You are welcome to provide us with feedback and insight on improving our current models by submitting a pull request.