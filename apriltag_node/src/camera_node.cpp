// A very simple node that publish grayscale images from a video cam through image
// transport
#include <string>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using camera_info_manager::CameraInfoManager;
using image_transport::CameraPublisher;
typedef boost::shared_ptr<CameraInfoManager> CameraInfoManagerPtr;

bool CheckCameraInfo(const CameraInfoManagerPtr &camera_info_manager,
                     const int height, const int width) {
  sensor_msgs::CameraInfoPtr cinfo = sensor_msgs::CameraInfoPtr(
      new sensor_msgs::CameraInfo(camera_info_manager->getCameraInfo()));
  if (cinfo->width != width || cinfo->height != height) {
    ROS_WARN("apriltag: Calibration dimension mismatch.");
    return false;
  }
  if (!camera_info_manager->isCalibrated()) {
    ROS_WARN("apriltag: Camera not calibrated.");
    return false;
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_node");
  ros::NodeHandle nh("~");

  // Read settings
  bool calibrated = false;
  double fps;
  int height, width;
  std::string calibration_url;
  nh.param("fps", fps, 20.0);
  nh.param("height", height, 240);
  nh.param("width", width, 320);
  if (!nh.getParam("calibration_url", calibration_url)) {
    calibration_url = "";
    ROS_WARN("No calibration file specified.");
  }
  ROS_INFO("Height: %d", height);
  ROS_INFO("Width:  %d", width);
  ROS_INFO("Fps:    %f", fps);

  // Initialize publisher
  image_transport::ImageTransport it(nh);
  CameraPublisher camera_pub = it.advertiseCamera("image_raw", 1);
  CameraInfoManagerPtr camera_info_manager = CameraInfoManagerPtr(
      new CameraInfoManager(nh, "webcam", calibration_url));
  calibrated = CheckCameraInfo(camera_info_manager, height, width);

  // Initialize opencv video capture
  cv::VideoCapture cap(0);
  cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
  usleep(1000);  // sleep for 1 second for video to respond

  ros::Rate loop_rate(fps);
  while (ros::ok()) {
    cv::Mat image_raw;
    cap.read(image_raw);
    if (image_raw.channels() > 1) {
      cv::cvtColor(image_raw, image_raw, CV_BGR2GRAY);
    }

    // Convert cv image to ros image message
    sensor_msgs::ImagePtr image = sensor_msgs::ImagePtr(new sensor_msgs::Image());
    image->height = height;
    image->width = width;
    image->step = width;
    image->encoding = sensor_msgs::image_encodings::MONO8;
    image->data.resize(image->step * image->height);
    memcpy(&image->data[0], image_raw.data, image->step * image->height);
    image->header.stamp = ros::Time::now();
    image->header.frame_id = std::string("camera");

    // Create camera info
    sensor_msgs::CameraInfoPtr camera_info(
        new sensor_msgs::CameraInfo(camera_info_manager->getCameraInfo()));
    if (!calibrated) {
      camera_info.reset(new sensor_msgs::CameraInfo());
      camera_info->width = image->width;
      camera_info->height = image->height;
    }
    camera_info->header.stamp = image->header.stamp;
    camera_info->header.frame_id = image->header.frame_id;

    // Publish image and camera info
    camera_pub.publish(image, camera_info);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
