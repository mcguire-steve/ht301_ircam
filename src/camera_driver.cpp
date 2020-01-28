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
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

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
  mono_pub_ = it_.advertiseCamera("image_mono", 1, false);
  therm_pub_ = it_.advertiseCamera("image_thermal", 1, false);
  
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

  if (frame->data_bytes != frame->width*frame->height*2)
    {
      ROS_INFO("Corrupt frame recv, only %d bytes, skipping", frame->data_bytes);
      return;
    }
  
  boost::recursive_mutex::scoped_lock lock(mutex_);

  assert(state_ == kRunning);
  assert(rgb_frame_);
  //Thermometry
  float maxtmp;
  int maxx;
  int maxy;
  float mintmp;
  int minx;
  int miny;
  float centertmp;
  float tmparr[3];

  
  sensor_msgs::Image::Ptr rgb_image(new sensor_msgs::Image());
  sensor_msgs::Image::Ptr raw_image(new sensor_msgs::Image());
  sensor_msgs::Image::Ptr therm_image(new sensor_msgs::Image());
  
  raw_image->width = width;
  raw_image->height = height;
  raw_image->encoding = "mono16";
  raw_image->step = width * 2;
  raw_image->data.resize(raw_image->step * raw_image->height);
  memcpy(&(raw_image->data[0]), frame->data, frame->data_bytes);

  therm_image->header.stamp = timestamp; 
  therm_image->width = width;
  therm_image->height = height - 4;
  therm_image->encoding = "32FC1";
  therm_image->step = width * sizeof(float);
  therm_image->data.resize(therm_image->step * therm_image->height); //float sizeof is in the step
  
  
  //Load the lookup table
  UpdateParam (0, (unsigned char*) frame->data); //for each frame
  //The first parameter might be inverted
  GetTmpData (0, (unsigned char*)frame->data, &maxtmp, &maxx, &maxy, &mintmp, &minx, &miny, &centertmp, tmparr, temperatureLUT);




 
  
	
  //use the LUT to figure out each pixel's float val
  for (uint32_t i=0; i<(raw_image->step*(raw_image->height-4)); i+=2)
    {
      uint16_t raw_val = *(uint16_t*)&raw_image->data[i];
      //ROS_INFO("Data val: %u", raw_val);
      float real_temp;
      if (raw_val > 16384)
	{
	  real_temp = 0.0f;
	}
      else
	{
	  real_temp = temperatureLUT[raw_val];
	}
      //ROS_INFO("Temp val: %1.2f", real_temp);
      memcpy(&therm_image->data[i*2], &real_temp, sizeof(float));
    }
  
  cv::Mat mono8_img = cv::Mat(height-4, width , CV_8UC1);
  cv_bridge::CvImageConstPtr therm_mat = cv_bridge::toCvShare(therm_image, "");

  /*
  for (int i = 0; i< therm_image->width*therm_image->height; i++)
    {
      ROS_INFO("Pixel: %d Temp: %1.2f", i, therm_mat->image.at<float>(i));
    }
  */
  
  cv::convertScaleAbs(cv_bridge::toCvShare(therm_image, "")->image, mono8_img, 2, 0.0);
  cv_bridge::CvImagePtr cvThermImage = boost::make_shared<cv_bridge::CvImage>(therm_image->header, "mono8", mono8_img);

  /*
  cv::namedWindow("therm_view", cv::WINDOW_NORMAL);
  cv::imshow("therm_view", cv_bridge::cvtColor(cvThermImage,"bgra8")->image);
  cv::waitKey(10);
  */
  
  ht301_msgs::Ht301MetaData::Ptr metadata(new ht301_msgs::Ht301MetaData());
  

  //Push as gray16 image: mono16 for raw values
  //Float data: TYPE_32FC1
  //Trim the bottom 4 rows
  


  metadata->header.stamp = timestamp;
  metadata->minTemp = mintmp;
  metadata->maxTemp = maxtmp;
  metadata->maxx = maxx;
  metadata->maxy = maxy;
  metadata->minx = minx;
  metadata->miny = miny;
  metadata->centerTemp = centertmp;
  meta_pub_.publish(metadata);

  /*
  mono_image->width = width;
  mono_image->height = height - 4;
  rgb_image->step = rgb_image->width * 3;
  //rgb_image->data.resize(rgb_image->step * rgb_image->height);
  */
  



  sensor_msgs::CameraInfo::Ptr cinfo(
    new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

  rgb_image->header.frame_id = frame_id;
  rgb_image->header.stamp = timestamp;

  cinfo->header.frame_id = frame_id;
  cinfo->header.stamp = timestamp;
  cinfo->width = raw_image->width;
  cinfo->height = raw_image->height;

  mono_pub_.publish(cvThermImage->toImageMsg(), cinfo);

  raw_pub_.publish(raw_image, cinfo); //not quite right...
  cinfo->height = raw_image->height-4;
  
  therm_pub_.publish(therm_image, cinfo);
  frameCount++;
  
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

  uvc_error_t find_err = uvc_find_device(
    ctx_, &dev_,
    vendor_id,
    product_id,
    serial.empty() ? NULL : serial.c_str());

  if (find_err != UVC_SUCCESS) {
    uvc_perror(find_err, "uvc_find_device");
    return;
  }

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

  uvc_print_stream_ctrl(&ctrl, stderr);

  //Set up the thermo library
  DataInit(width,height);
  frameCount = 0;

  //Send the magic make it work sequence
  uvc_set_zoom_abs(devh_,0x8004); //16-bit datamode
  uvc_set_zoom_abs(devh_,0x8000); //calibrate
  uvc_set_zoom_abs(devh_,0x8020); //enable thermal data
  
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


void HT301CameraDriver::spin() {
  ros::Rate r(10); // 10 hz
  while (ros::ok())
    {
      
      if (frameCount % 200 == 0)
	{
	   uvc_set_zoom_abs(devh_,0x8000); //calibrate
	}
      ros::spinOnce();
      r.sleep();
    }
}

};
