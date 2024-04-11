# ROS2 YOLOv4 with USB Camera Integration

This README provides a comprehensive guide on how to integrate YOLOv4 for object detection in ROS2 using a USB camera. It covers the setup of a new ROS2 workspace, building the YOLOv4 package, and launching nodes for capturing video feed and performing object detection.

## Prerequisites

Ensure you have the following installed on your system before proceeding:

- ROS2
- A USB camera compatible with ROS2

## Setup Instructions

### Creating the ROS2 Workspace

1. **Create a New Workspace**:
   
   Navigate to your desired workspace directory. If you are using Docker, this should be at the same level as your Docker container. Create a new workspace named `yolov4_ws`.

    ```bash
    mkdir -p ~/yolov4_ws
    cd ~/yolov4_ws
    ```

2. **Clone the YOLOv4 ROS2 Package**:

   Clone the YOLOv4 package from the provided GitHub repository into the `src` directory of your workspace.

    ```bash
    git clone https://github.com/TKUwengkunduo/ROS2_YOLOv4_usbCamera.git src
    ```

3. **Build the Workspace**:

   Use `colcon` to build the workspace. The `--symlink-install` flag creates symbolic links instead of copying files, saving disk space.

    ```bash
    colcon build --symlink-install
    ```

    
### Launching the Nodes

To launch the necessary nodes for capturing video and performing YOLOv4 object detection, follow these steps:



#### Open the Camera Node

To capture video from the USB camera, you'll need to launch the camera node.

```bash
source install/setup.bash
ros2 run camera publish_usb_cam
```


#### Start the YOLOv4 Detection Node

   In a new terminal window, source the ROS2 environment again and then start the YOLOv4 detection node. This node subscribes to the video feed, performs object detection, and publishes the results.

    ```bash
    source install/setup.bash
    ros2 run yolov4 yolo_detection
    ```

#### View Detection Results

   To see the detection results published by the YOLOv4 node, open another terminal window, source the ROS2 environment, and use the `echo` command to listen to the detection results topic.

    ```bash
    source install/setup.bash
    ros2 topic echo /yolo_detection/detections
    ```
