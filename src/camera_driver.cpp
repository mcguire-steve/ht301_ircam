/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2012 Ken Tossell
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include "ht301_ircam/camera_driver.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <image_transport/camera_publisher.h>
#include <ht301_msgs/Ht301MetaData.h>

#include <libuvc/libuvc.h>
#define libuvc_VERSION (libuvc_VERSION_MAJOR * 10000 \
                      + libuvc_VERSION_MINOR * 100 \
                      + libuvc_VERSION_PATCH)

namespace ht301_ircam {

HT301CameraDriver::HT301CameraDriver(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  : nh_(nh), priv_nh_(priv_nh),
    state_(kInitial),
    ctx_(NULL), dev_(NULL), devh_(NULL), rgb_frame_(NULL),
    it_(nh_),
    cinfo_manager_(nh) {
  raw_pub_ = it_.advertiseCamera("image_raw", 1, false);
  rgb_pub_ = it_.advertiseCamera("image_rgb", 1, false);
  meta_pub_ = nh_.advertise<ht301_msgs::Ht301MetaData>("metadata", 5, false);
  
  priv_nh_.param<std::string>("vendor", vendor, "0x0");
  priv_nh_.param<std::string>("product", product, "0x0");
  priv_nh_.param<std::string>("serial", serial, "");
  priv_nh_.param<std::string>("video_mode", video_mode, "YUYV");
  priv_nh_.param<std::string>("frame_id", frame_id, "ircam");
  priv_nh_.param<int32_t>("index", index, 0);
  priv_nh_.param<int32_t>("width", width, 640);
  priv_nh_.param<int32_t>("height", height, 480);
  priv_nh_.param<int32_t>("frame_rate", frame_rate, 30);

}

HT301CameraDriver::~HT301CameraDriver() {
  if (rgb_frame_)
    uvc_free_frame(rgb_frame_);

  if (ctx_)
    uvc_exit(ctx_);  // Destroys dev_, devh_, etc.
}

bool HT301CameraDriver::Start() {
  assert(state_ == kInitial);

  uvc_error_t err;

  err = uvc_init(&ctx_, NULL);

  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_init");
    return false;
  }

  state_ = kStopped;
  OpenCamera();
  return state_ == kRunning;
}

void HT301CameraDriver::Stop() {
  boost::recursive_mutex::scoped_lock lock(mutex_);

  assert(state_ != kInitial);

  if (state_ == kRunning)
    CloseCamera();

  assert(state_ == kStopped);

  uvc_exit(ctx_);
  ctx_ = NULL;

  state_ = kInitial;
}

void HT301CameraDriver::ImageCallback(uvc_frame_t *frame) {
  ros::Time timestamp = ros::Time(frame->capture_time.tv_sec, frame->capture_time.tv_usec * 1000);
  if ( timestamp == ros::Time(0) ) {
    timestamp = ros::Time::now();
  }

  if (frame->data_bytes < frame->width*frame->height*2)
    {
      ROS_INFO("Incomplete frame recv, skipping");
      return;
    }
  
  boost::recursive_mutex::scoped_lock lock(mutex_);

  assert(state_ == kRunning);
  assert(rgb_frame_);

  sensor_msgs::Image::Ptr rgb_image(new sensor_msgs::Image());
  sensor_msgs::Image::Ptr raw_image(new sensor_msgs::Image());
  ht301_msgs::Ht301MetaData::Ptr metadata(new ht301_msgs::Ht301MetaData());
  raw_image->width = width;
  raw_image->height = height;
  raw_image->encoding = "YUV422";
  raw_image->step = width * 2;
  raw_image->data.resize(raw_image->step * raw_image->height);
  memcpy(&(raw_image->data[0]), frame->data, frame->data_bytes);

  //Trim the bottom 4 rows

  uint32_t metaStart = raw_image->step * (raw_image->height - 4);
  metadata->header.stamp = timestamp;
  metadata->width = raw_image->step;
  metadata->data.resize(raw_image->step * 4);
  memcpy(&(metadata->data[0]), frame->data + metaStart, raw_image->step * 4);
  meta_pub_.publish(metadata);
  
  rgb_image->width = width;
  rgb_image->height = height - 4;
  rgb_image->step = rgb_image->width * 3;
  rgb_image->data.resize(rgb_image->step * rgb_image->height);

  if (frame->frame_format == UVC_FRAME_FORMAT_BGR){
    rgb_image->encoding = "bgr8";
    memcpy(&(rgb_image->data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_RGB){
    rgb_image->encoding = "rgb8";
    memcpy(&(rgb_image->data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_UYVY) {
    uvc_error_t conv_ret = uvc_uyvy2bgr(frame, rgb_frame_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    rgb_image->encoding = "bgr8";
    memcpy(&(rgb_image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_GRAY8) {
    rgb_image->encoding = "8UC1";
    rgb_image->step = rgb_image->width;
    rgb_image->data.resize(rgb_image->step * rgb_image->height);
    memcpy(&(rgb_image->data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_GRAY16) {
    rgb_image->encoding = "16UC1";
    rgb_image->step = rgb_image->width*2;
    rgb_image->data.resize(rgb_image->step * rgb_image->height);
    memcpy(&(rgb_image->data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_YUYV) {
    // FIXME: uvc_any2bgr does not work on "yuyv" format, so use uvc_yuyv2bgr directly.
    uvc_error_t conv_ret = uvc_yuyv2bgr(frame, rgb_frame_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    rgb_image->encoding = "bgr8";
    memcpy(&(rgb_image->data[0]), rgb_frame_->data, rgb_image->step * rgb_image->height);
#if libuvc_VERSION     > 00005 /* version > 0.0.5 */
  } else if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
    // Enable mjpeg support despite uvs_any2bgr shortcoming
    //  https://github.com/ros-drivers/libuvc_ros/commit/7508a09f
    uvc_error_t conv_ret = uvc_mjpeg2rgb(frame, rgb_frame_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    rgb_image->encoding = "rgb8";
    memcpy(&(rgb_image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
#endif
  } else {
    uvc_error_t conv_ret = uvc_any2bgr(frame, rgb_frame_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    rgb_image->encoding = "bgr8";
    memcpy(&(rgb_image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
  }


  sensor_msgs::CameraInfo::Ptr cinfo(
    new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

  rgb_image->header.frame_id = frame_id;
  rgb_image->header.stamp = timestamp;
  cinfo->header.frame_id = frame_id;
  cinfo->header.stamp = timestamp;
  cinfo->width = rgb_image->width;
  cinfo->height = rgb_image->height;
  rgb_pub_.publish(rgb_image, cinfo);

  raw_pub_.publish(raw_image, cinfo); //not quite right...

}

/* static */ void HT301CameraDriver::ImageCallbackAdapter(uvc_frame_t *frame, void *ptr) {
  HT301CameraDriver *driver = static_cast<HT301CameraDriver*>(ptr);

  driver->ImageCallback(frame);
}


enum uvc_frame_format HT301CameraDriver::GetVideoMode(std::string vmode){
  if(vmode == "uncompressed") {
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  } else if (vmode == "compressed") {
    return UVC_COLOR_FORMAT_COMPRESSED;
  } else if (vmode == "yuyv") {
    return UVC_COLOR_FORMAT_YUYV;
  } else if (vmode == "uyvy") {
    return UVC_COLOR_FORMAT_UYVY;
  } else if (vmode == "rgb") {
    return UVC_COLOR_FORMAT_RGB;
  } else if (vmode == "bgr") {
    return UVC_COLOR_FORMAT_BGR;
  } else if (vmode == "mjpeg") {
    return UVC_COLOR_FORMAT_MJPEG;
  } else if (vmode == "gray8") {
    return UVC_COLOR_FORMAT_GRAY8;
  } else if (vmode == "gray16") {
    return UVC_COLOR_FORMAT_GRAY16;
  } else {
    ROS_ERROR_STREAM("Invalid Video Mode: " << vmode);
    ROS_WARN_STREAM("Continue using video mode: uncompressed");
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  }
};

void HT301CameraDriver::OpenCamera() {
  assert(state_ == kStopped);

  int vendor_id = strtol(vendor.c_str(), NULL, 0);
  int product_id = strtol(product.c_str(), NULL, 0);

  ROS_INFO("Opening camera with vendor=0x%x, product=0x%x, serial=\"%s\", index=%d",
           vendor_id, product_id, serial.c_str(), index);

  uvc_device_t **devs;

  // Implement missing index select behavior
  // https://github.com/ros-drivers/libuvc_ros/commit/4f30e9a0
#if libuvc_VERSION     > 00005 /* version > 0.0.5 */
  uvc_error_t find_err = uvc_find_devices(
    ctx_, &devs,
    vendor_id,
    product_id,
    serial.empty() ? NULL : serial.c_str());

  if (find_err != UVC_SUCCESS) {
    uvc_perror(find_err, "uvc_find_device");
    return;
  }

  // select device by index
  dev_ = NULL;
  int dev_idx = 0;
  while (devs[dev_idx] != NULL) {
    if(dev_idx == index) {
      dev_ = devs[dev_idx];
    }
    else {
      uvc_unref_device(devs[dev_idx]);
    }

    dev_idx++;
  }

  if(dev_ == NULL) {
    ROS_ERROR("Unable to find device at index %d", index);
    return;
  }
#else
  uvc_error_t find_err = uvc_find_device(
    ctx_, &dev_,
    vendor_id,
    product_id,
    serial.empty() ? NULL : serial.c_str());

  if (find_err != UVC_SUCCESS) {
    uvc_perror(find_err, "uvc_find_device");
    return;
  }

#endif
  uvc_error_t open_err = uvc_open(dev_, &devh_);

  if (open_err != UVC_SUCCESS) {
    switch (open_err) {
    case UVC_ERROR_ACCESS:
#ifdef __linux__
      ROS_ERROR("Permission denied opening /dev/bus/usb/%03d/%03d",
                uvc_get_bus_number(dev_), uvc_get_device_address(dev_));
#else
      ROS_ERROR("Permission denied opening device %d on bus %d",
                uvc_get_device_address(dev_), uvc_get_bus_number(dev_));
#endif
      break;
    default:
#ifdef __linux__
      ROS_ERROR("Can't open /dev/bus/usb/%03d/%03d: %s (%d)",
                uvc_get_bus_number(dev_), uvc_get_device_address(dev_),
                uvc_strerror(open_err), open_err);
#else
      ROS_ERROR("Can't open device %d on bus %d: %s (%d)",
                uvc_get_device_address(dev_), uvc_get_bus_number(dev_),
                uvc_strerror(open_err), open_err);
#endif
      break;
    }

    uvc_unref_device(dev_);
    return;
  }
 uvc_print_diag(devh_, stderr);

  uvc_stream_ctrl_t ctrl;
  uvc_error_t mode_err = uvc_get_stream_ctrl_format_size(
    devh_, &ctrl,
    GetVideoMode(video_mode),
    width, height,
    frame_rate);

  if (mode_err != UVC_SUCCESS) {
    uvc_perror(mode_err, "uvc_get_stream_ctrl_format_size");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    ROS_ERROR("check video_mode/width/height/frame_rate are available");
    uvc_print_diag(devh_, NULL);
    return;
  }

  uvc_error_t stream_err = uvc_start_streaming(devh_, &ctrl, &HT301CameraDriver::ImageCallbackAdapter, this, 0);

  if (stream_err != UVC_SUCCESS) {
    uvc_perror(stream_err, "uvc_start_streaming");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    return;
  }

  if (rgb_frame_)
    uvc_free_frame(rgb_frame_);

  rgb_frame_ = uvc_allocate_frame(width * height * 3);
  assert(rgb_frame_);

  state_ = kRunning;
}

void HT301CameraDriver::CloseCamera() {
  assert(state_ == kRunning);

  uvc_close(devh_);
  devh_ = NULL;

  uvc_unref_device(dev_);
  dev_ = NULL;

  state_ = kStopped;
}

};
