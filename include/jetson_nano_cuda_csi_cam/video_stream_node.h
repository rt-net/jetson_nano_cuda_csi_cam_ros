// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright 2021 RT Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * This program is based on https://github.com/dusty-nv/ros_deep_learning
 * which is released under the MIT License.
 *
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * https://github.com/dusty-nv/ros_deep_learning/blob/ac40e93413f4b7cb911a18c0e4d5daac479234d4/src/ros_compat.h
 */

#ifndef __JETSON_CAM_VIDEO_STREAM_NODE_H_
#define __JETSON_CAM_VIDEO_STREAM_NODE_H_

#include "jetson_nano_cuda_csi_cam/image_converter.h"
#include <jetson-utils/videoSource.h>

template <typename T>
static void __ros_declare_parameter(ros::NodeHandle& nh, const std::string& name, const T& default_value)
{
  if (!nh.hasParam(name))
    nh.setParam(name, default_value);
}
template <class MessageType>
using Publisher = ros::Publisher*;

#endif