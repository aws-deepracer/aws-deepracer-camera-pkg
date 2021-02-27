# AWS DeepRacer Camera Package 

## Overview

The DeepRacer Camera ROS package creates the *camera_node* which is part of the core AWS DeepRacer application and will be launched from the deepracer_launcher. More details about the application and the components can be found [here](https://github.com/aws-racer/aws-deepracer-launcher).

This node is responsible for reading the raw data from the one/two cameras connected to the USB slots at the front of the device and publishing them as CameraMsg at the rate at which they are read.

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The DeepRacer device comes with all the pre-requisite packages and libraries installed to build and run the camera_pkg. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-racer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The camera_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

1. *deepracer_interfaces_pkg* - This packages contains the custom message and service type definitions used across the AWS DeepRacer core application.
2. *cv_bridge* - This contains CvBridge, which converts between ROS Image messages and OpenCV images.
3. *image_transport* - It provides transparent support for transporting images in low-bandwidth compressed formats.
4. *sensor_msgs* - This package defines messages for commonly used sensors, including cameras and scanning laser rangefinders.


### Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the camera_pkg on the DeepRacer device:

        git clone https://github.com/aws-racer/aws-deepracer-camera-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-camera-pkg
        rosws update

1. Resolve the camera_pkg dependencies:

        cd ~/deepracer_ws && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the camera_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws && colcon build --packages-select camera_pkg deepracer_interfaces_pkg

## Usage

The camera_node provides the core functionality to combine the camera data from the cameras connected to the USB slots at the front of the DeepRacer vehicle. Although the nodes is built to work with the AWS DeepRacer application, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built camera_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Navigate to the deepracer workspace:

        cd ~/deepracer_ws

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/install/setup.bash 

1. Launch the camera_node using the launch script:

        ros2 launch camera_pkg camera_pkg_launch.py

### Activating the camera image publisher using CLI:

Once the camera_pkg_launch.py has been kicked-off, open up a adjacent new terminal as root user:

1. Navigate to the deepracer workspace:
        
        cd ~/deepracer_ws

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/install/setup.bash 

1. Activate the camera publisher to start publishing images using the below ros2 service call:

        ros2 service call /camera_pkg/media_state deepracer_interfaces_pkg/srv/VideoStateSrv "{activate_video: 1}"


## Launch Files

The  camera_pkg_launch.py is also included in this package that gives an example of how to launch the node independently from the core application.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='camera_pkg',
                    namespace='camera_pkg',
                    executable='camera_node',
                    name='camera_node'
                )
            ])


## Node Details

### camera_node

#### Published Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
| /camera_pkg/video_mjpeg | CameraMsg | Publisher to publish the single camera or two camera images read from the cameras connected to the USB slots at the front of the device. |
| /camera_pkg/display_mjpeg | Image | Publisher to publish one camera image for display purposes in the device console UI.|

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
| media_state | VideoStateSrv | Service that is called to activate the camera node and start publishing the images to the video_mjpeg and display_mjpeg topics. |
