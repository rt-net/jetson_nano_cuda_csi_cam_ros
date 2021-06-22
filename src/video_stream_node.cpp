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
 * This program is based on https://github.com/dusty-nv/ros_deep_learning.
 * which is released under the MIT License.
 *
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * https://github.com/dusty-nv/ros_deep_learning/blob/ac40e93413f4b7cb911a18c0e4d5daac479234d4/src/node_video_source.cpp
 */

#include "jetson_nano_cuda_csi_cam/video_stream_node.h"

#include <camera_info_manager/camera_info_manager.h>
#include <jetson-utils/videoSource.h>

// globals
videoSource* stream = NULL;
imageConverter* image_cvt = NULL;
Publisher<sensor_msgs::Image> image_pub = NULL;

// aquire and publish camera frame
bool aquireFrame()
{
  imageConverter::PixelType* nextFrame = NULL;

  // get the latest frame
  if (!stream->Capture(&nextFrame, 1000))
  {
    ROS_ERROR("failed to capture next frame");
    return false;
  }

  // assure correct image size
  if (!image_cvt->Resize(stream->GetWidth(), stream->GetHeight(), imageConverter::ROSOutputFormat))
  {
    ROS_ERROR("failed to resize camera image converter");
    return false;
  }

  // populate the message
  sensor_msgs::Image msg;

  if (!image_cvt->Convert(msg, imageConverter::ROSOutputFormat, nextFrame))
  {
    ROS_ERROR("failed to convert video stream frame to sensor_msgs::Image");
    return false;
  }

  // populate timestamp in header field
  msg.header.stamp = ros::Time::now();

  // publish the message
  image_pub->publish(msg);
  ROS_DEBUG("published %ux%u video frame", stream->GetWidth(), stream->GetHeight());

  return true;
}

// node main loop
int main(int argc, char** argv)
{
  /*
   * create node instance
   */
  ros::init(argc, argv, "video_source");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  /*
   * declare parameters
   */
  videoOptions video_options;

  std::string resource_str;
  std::string codec_str;
  std::string flipmethod_str;

  int video_width = video_options.width;
  int video_height = video_options.height;
  __ros_declare_parameter(private_nh, "resource", resource_str);
  __ros_declare_parameter(private_nh, "codec", codec_str);
  __ros_declare_parameter(private_nh, "width", video_width);
  __ros_declare_parameter(private_nh, "height", video_height);
  __ros_declare_parameter(private_nh, "framerate", video_options.frameRate);
  __ros_declare_parameter(private_nh, "loop", video_options.loop);
  __ros_declare_parameter(private_nh, "flip_method", flipmethod_str);

  /*
   * retrieve parameters
   */
  private_nh.getParam("resource", resource_str);
  private_nh.getParam("codec", codec_str);
  private_nh.getParam("width", video_width);
  private_nh.getParam("height", video_height);
  private_nh.getParam("framerate", video_options.frameRate);
  private_nh.getParam("loop", video_options.loop);
  private_nh.getParam("flip_method", flipmethod_str);

  if (resource_str.size() == 0)
  {
    ROS_ERROR("resource param wasn't set - please set the node's resource parameter to the input device/filename/URL");
    return 0;
  }

  if (codec_str.size() != 0)
    video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());

  if (flipmethod_str.size() != 0)
    video_options.flipMethod = videoOptions::FlipMethodFromStr(flipmethod_str.c_str());

  video_options.width = video_width;
  video_options.height = video_height;

  ROS_INFO("opening video source: %s", resource_str.c_str());

  /*
   * open video source
   */
  stream = videoSource::Create(resource_str.c_str(), video_options);

  if (!stream)
  {
    ROS_ERROR("failed to open video source");
    return 0;
  }

  /*
   * create image converter
   */
  image_cvt = new imageConverter();

  if (!image_cvt)
  {
    ROS_ERROR("failed to create imageConverter");
    return 0;
  }

  /*
   * advertise publisher topics
   */
  ros::Publisher __publisher_image_pub = private_nh.advertise<sensor_msgs::Image>("image_raw", 2);
  image_pub = &__publisher_image_pub;

  /*
   * start the camera streaming
   */
  if (!stream->Open())
  {
    ROS_ERROR("failed to start streaming video source");
    return 0;
  }

  /*
   * start publishing video frames
   */
  while (ros::ok())
  {
    if (!aquireFrame())
    {
      if (!stream->IsStreaming())
      {
        ROS_INFO("stream is closed or reached EOS, exiting node...");
        break;
      }
    }

    if (ros::ok())
      ros::spinOnce();
  }

  /*
   * free resources
   */
  delete stream;
  delete image_cvt;

  return 0;
}
