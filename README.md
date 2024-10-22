# jetson_nano_cuda_csi_cam_ros

ROS driver for the Jetson Nano DevKit B01 + dual CSI camera.

![](https://rt-net.github.io/images/jetson-nano/jetson_nano_dual_csi.jpg)

This ROS package is designed to acquire images from the one or two CSI cameras attached to the Jetson Nano DevKit B01 and publish them as [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) in ROS.

## Requirements

* JetPack 4.5.1
    * CUDA 10.x
* [dusty-nv/jetson-utils](https://github.com/dusty-nv/jetson-utils)
* [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

## Installation

### 1. Download and install [dusty-nv/jetson-utils](https://github.com/dusty-nv/jetson-utils)

Check if it is already installed.

```
ldconfig -p | grep jetson && echo "installed" || echo "not installed yet"
```

If it is not installed, follow the steps below to install it.

```
sudo apt install libglew-dev
git clone https://github.com/dusty-nv/jetson-utils.git
mkdir jetson-utils/build
cd jetson-utils/build
cmake -DENABLE_NVMM=OFF ..
make
sudo make install
sudo ldconfig
```

### 2. Download and install this package

```
cd ~/catkin_ws/src
git clone https://github.com/rt-net/jetson_nano_cuda_csi_cam_ros.git
rosdep install -r -i -y --from-paths jetson_nano_cuda_csi_cam_ros
catkin build
catkin source
```

## Usage
### Quick Start

To publish the CAM0 camera stream data as `/csr_cam_0/image_raw` topic, run the following command.

```
roslaunch jetson_nano_cuda_csi_cam jetson_csi_cam.launch sensor_id:=0 width:=640 height:=480 fps:=60
```

To publish the CAM0 and CAM1 camera stream data as `/csr_cam_0/image_raw` topic and `/csr_cam_1/image_raw` topic, run the following command.

```
roslaunch jetson_nano_cuda_csi_cam jetson_dual_csi_cam.launch width:=640 height:=480 fps:=60
```

* __`width`__: image width
* __`height`__: image height
* __`fps`__: desired framerate

## LICENSE

(C) 2021 RT Corporation

This repository is licensed under the Apache License, Version 2.0, see [LICENSE](./LICENSE).  
Unless attributed otherwise, everything in this repository is under the Apache License, Version 2.0.

### Acknowledgements

This project is based on [dusty-nv/ros_deep_learning](https://github.com/dusty-nv/ros_deep_learning) released under the MIT License.

```txt
/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
```

This project is based on [peter-moran/jetson_csi_cam](https://github.com/peter-moran/jetson_csi_cam) released under the MIT License.

```
MIT License

Copyright (c) 2017 Peter Moran

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```