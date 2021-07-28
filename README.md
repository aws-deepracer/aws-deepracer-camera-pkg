# AWS DeepRacer camera package 

## Overview

The AWS DeepRacer camera ROS package creates the `camera_node`, which is part of the core AWS DeepRacer application and launches from the `deepracer_launcher`. For more information about the application and the components, see the [`aws-deepracer-launcher` repository](https://github.com/aws-deepracer/aws-deepracer-launcher).

This node is responsible for reading the raw data from the one or two cameras connected to the USB slots at the front of the device and publishing them as `CameraMsg` at the rate at which they are read.

## License

The source code is released under Apache 2.0 (https://aws.amazon.com/apache-2-0/).

## Installation

Follow these steps to install the AWS DeepRacer camera package.

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed to run the `camera_pkg`. For more information about the preinstalled set of packages and libraries on the AWS DeepRacer, and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md).

The `camera_pkg` specifically depends on the following ROS 2 packages as build and run dependencies:

1. `deepracer_interfaces_pkg`: This package contains the custom message and service-type definitions used across the AWS DeepRacer core application.
2. `cv_bridge`: This package contains CvBridge, which converts between ROS Image messages and OpenCV images.
3. `image_transport`: This package provides transparent support for transporting images in low-bandwidth compressed formats.
4. `sensor_msgs`: This package defines messages for commonly used sensors, including cameras and scanning laser rangefinders.


## Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the `camera_pkg` on the AWS DeepRacer device:

        git clone https://github.com/aws-deepracer/aws-deepracer-camera-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-camera-pkg
        rosws update

1. Resolve the `camera_pkg` dependencies:

        cd ~/deepracer_ws/aws-deepracer-camera-pkg && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `camera_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_ws/aws-deepracer-camera-pkg && colcon build --packages-select camera_pkg deepracer_interfaces_pkg

## Usage

The `camera_node` provides the core functionality to combine the camera data from the cameras connected to the USB slots at the front of the AWS DeepRacer vehicle. Although the node is built to work with the AWS DeepRacer application, you can run it independently for development, testing, and debugging purposes.

### Run the node

To launch the built `camera_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user:

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-camera-pkg/install/setup.bash 

1. Launch the `camera_node` using the launch script:

        ros2 launch camera_pkg camera_pkg_launch.py

### Activating the camera image publisher using the CLI

Once the `camera_pkg_launch.py` has been kicked off, open an adjacent new terminal as the root user:

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-camera-pkg/install/setup.bash 

1. Activate the camera publisher to start publishing images using the following ROS 2 service call:

        ros2 service call /camera_pkg/media_state deepracer_interfaces_pkg/srv/VideoStateSrv "{activate_video: 1}"


## Launch files

The `camera_pkg_launch.py` included in this package provides an example demonstrating how to launch the node independently from the core application.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='camera_pkg',
                    namespace='camera_pkg',
                    executable='camera_node',
                    name='camera_node'
                    parameters=[
                        {'resize_images': False}
                ]
                )
            ])


### Configuration file and parameters

| Parameter name   | Description  |
| ---------------- |  ----------- |
| `resize_images` | Set to `True` or `False` depending on if you want to resize the images in camera_pkg |


## Node details

### `camera_node`

#### Published topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
| /`camera_pkg`/`video_mjpeg` | `CameraMsg` | Publisher that publishes the single-camera or two-camera images read from the cameras connected to the USB slots at the front of the device. |
| /`camera_pkg`/`display_mjpeg` | Image | Publisher that publishes one camera image for display purposes in the device console UI.|

#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
| `media_state` | `VideoStateSrv` | Service that is called to activate the camera node and start publishing the images to the `video_mjpeg` and `display_mjpeg` topics. |

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
