### Usage Instructions

#### 1. Publish Images Only via ROS Topic

In this mode, the `image_publisher` publishes live images to other ROS2 nodes through a ROS topic. The images are shared solely as a ROS topic and are not saved anywhere.
```bash
ros2 run opencv_tools image_publisher
```

#### 2. Publish Images via ROS Topic and Save Images to a Folder

This mode publishes images via a ROS topic while also saving each image along with a **timestamp** to a specified folder. The path for saving the images should be indicated by the `save_folder` parameter.
```bash
ros2 run opencv_tools image_publisher --ros-args -p save_images:=true -p save_folder:="your_path"
```

> **Note**: Replace `your_path` with the full path of the directory where you want the images to be saved.

