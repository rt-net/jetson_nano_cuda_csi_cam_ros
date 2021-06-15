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
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * https://github.com/dusty-nv/ros_deep_learning/blob/ac40e93413f4b7cb911a18c0e4d5daac479234d4/src/image_converter.h
 */

#ifndef __JETSON_CAM_IMAGE_CONVERTER_H_
#define __JETSON_CAM_IMAGE_CONVERTER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <jetson-utils/cudaUtility.h>
#include <jetson-utils/imageFormat.h>

/**
 * GPU image conversion
 */
class imageConverter
{
public:
  /**
   * Output image pixel type
   */
  typedef uchar3 PixelType;

  /**
   * Image format used for internal CUDA processing
   */
  static const imageFormat InternalFormat = IMAGE_RGB8;

  /**
   * Image format used for outputting ROS image messages
   */
  static const imageFormat ROSOutputFormat = IMAGE_BGR8;

  /**
   * Constructor
   */
  imageConverter();

  /**
   * Destructor
   */
  ~imageConverter();

  /**
   * Free the memory
   */
  void Free();

  /**
   * Convert to 32-bit RGBA floating point
   */
  bool Convert(const sensor_msgs::ImageConstPtr& input);

  /**
   * Convert to ROS sensor_msgs::Image message
   */
  bool Convert(sensor_msgs::Image& msg_out, imageFormat outputFormat);

  /**
   * Convert to ROS sensor_msgs::Image message
   */
  bool Convert(sensor_msgs::Image& msg_out, imageFormat outputFormat, PixelType* imageGPU);

  /**
   * Resize the memory (if necessary)
   */
  bool Resize(uint32_t width, uint32_t height, imageFormat inputFormat);

  /**
   * Retrieve the converted image width
   */
  inline uint32_t GetWidth() const
  {
    return mWidth;
  }

  /**
   * Retrieve the converted image height
   */
  inline uint32_t GetHeight() const
  {
    return mHeight;
  }

  /**
   * Retrieve the GPU pointer of the converted image
   */
  inline PixelType* ImageGPU() const
  {
    return mOutputGPU;
  }

private:
  uint32_t mWidth;
  uint32_t mHeight;
  size_t mSizeInput;
  size_t mSizeOutput;

  void* mInputCPU;
  void* mInputGPU;

  PixelType* mOutputCPU;
  PixelType* mOutputGPU;
};

#endif
