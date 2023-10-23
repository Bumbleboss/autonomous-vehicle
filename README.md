# YOLOv5 Custom Training for OAK-D Camera
This branch of the repository deals with training custom dataset for our graduation project.

The trained model will be uploaded on [Luxonis Tools](https://tools.luxonis.com/) to compile a
blob and JSON configuration for use with our OAK-D camera. Please refer to our [notebook](train.ipynb) if you wish
to train the same dataset we are using.
---
### Notes

- The notebook is set up to work on Google Colab, other platforms may face paths complications
  - If you wish to use it locally or on other platforms, make sure to modify `data.yaml` inside `datasets/*/` folder
- We use [YOLOv5 PyTorch TXT](https://roboflow.com/formats/yolov5-pytorch-txt) annotation format
- `autosplit.py` is used to automatically split a dataset into train/val/test splits and save the resulting splits into `autosplit_*.txt` files.