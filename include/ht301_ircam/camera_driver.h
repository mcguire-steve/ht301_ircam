#pragma once

#include <libuvc/libuvc.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/thread/mutex.hpp>
#include <ht301_ircam/XthermDll.h>

namespace ht301_ircam {

class HT301CameraDriver {
public:
  HT301CameraDriver(ros::NodeHandle nh, ros::NodeHandle priv_nh);
  ~HT301CameraDriver();

  bool Start();
  void Stop();
  void spin();
  
private:
  enum State {
    kInitial = 0,
    kStopped = 1,
    kRunning = 2,
  };

  // Flags controlling whether the sensor needs to be stopped (or reopened) when changing settings
  static const int kReconfigureClose = 3; // Need to close and reopen sensor to change this setting
  static const int kReconfigureStop = 1; // Need to stop the stream before changing this setting
  static const int kReconfigureRunning = 0; // We can change this setting without stopping the stream

  void OpenCamera();
  void CloseCamera();

  // Accept a reconfigure request from a client
  //void ReconfigureCallback(UVCCameraConfig &config, uint32_t level);
  enum uvc_frame_format GetVideoMode(std::string vmode);
  /*
  // Accept changes in values of automatically updated controls
  void AutoControlsCallback(enum uvc_status_class status_class,
                            int event,
                            int selector,
                            enum uvc_status_attribute status_attribute,
                            void *data, size_t data_len);
  static void AutoControlsCallbackAdapter(enum uvc_status_class status_class,
                                          int event,
                                          int selector,
                                          enum uvc_status_attribute status_attribute,
                                          void *data, size_t data_len,
                                          void *ptr);

  */
  // Accept a new image frame from the camera
  void ImageCallback(uvc_frame_t *frame);
  static void ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);

  ros::NodeHandle nh_, priv_nh_;

  State state_;
  boost::recursive_mutex mutex_;

  uvc_context_t *ctx_;
  uvc_device_t *dev_;
  uvc_device_handle_t *devh_;
  uvc_frame_t *rgb_frame_;

  float temperatureLUT[16384];
  
  std::string vendor;
  std::string product;
  std::string serial;
  std::string video_mode;
  std::string frame_id;
  int32_t index;
  int32_t width;
  int32_t height;
  int32_t frame_rate;
  uint8_t frameCount;
  
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher raw_pub_;
  image_transport::CameraPublisher rgb_pub_;
  image_transport::CameraPublisher therm_pub_;
  
  ros::Publisher meta_pub_;
  
  //dynamic_reconfigure::Server<UVCCameraConfig> config_server_;
  //UVCCameraConfig config_;
  //bool config_changed_;

  camera_info_manager::CameraInfoManager cinfo_manager_;
};

};
