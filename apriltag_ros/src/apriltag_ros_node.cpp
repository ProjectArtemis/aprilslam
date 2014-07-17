#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <map>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <cmath>

#define BUILD_MIT
#define USE_RVIZ

#if defined(BUILD_UMICH)
extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/image_u8.h"
#include "apriltag/tag36h11.h"
#include "apriltag/zarray.h"
}
#elif defined(BUILD_MIT)
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#endif

typedef cv::Point2d Point2;
typedef cv::Point3d Point3;

typedef struct tag {
  Point2 p[4];
} tag_t;

static std::map<int, tag> tag_w;

// Publisher
ros::Publisher pose_pub;
#if defined(USE_RVIZ)
ros::Publisher image_pub;
#endif

// blue, green, red and magenta
const cv::Scalar colors[] = { cv::Scalar(255, 0, 0, 0),
                              cv::Scalar(0, 255, 0, 0),
                              cv::Scalar(0, 0, 255, 0),
                              cv::Scalar(255, 0, 255, 0) };

cv::Mat rodriguesToQuat(const cv::Mat &r) {
  // theta = norm(r)
  // q = [cos(theta/2), sin(theta/2) * r / theta]
  cv::Mat q = cv::Mat::zeros(cv::Size(1, 4), CV_64F);
  double *pq = q.ptr<double>();
  const double *pr = r.ptr<double>();
  double x, y, z;
  x = pr[0], y = pr[1], z = pr[2];
  double theta = std::sqrt(x * x + y * y + z * z);
  if (theta < std::numeric_limits<double>::epsilon() * 10.0) {
    pq[0] = 1.0;
    return q;
  }

  double haver_sin = std::sin(0.5 * theta);
  double haver_cos = std::cos(0.5 * theta);
  pq[0] = haver_cos;
  pq[1] = haver_sin * x / theta;
  pq[2] = haver_sin * y / theta;
  pq[3] = haver_sin * z / theta;

  return q;
}

void cam_callback(const sensor_msgs::ImageConstPtr &image,
                  const sensor_msgs::CameraInfoConstPtr &cinfo) {
  // Get camera info
  static bool init_cam = false;
  static cv::Mat K = cv::Mat::zeros(cv::Size(3, 3), CV_64F);
  static cv::Mat D = cv::Mat::zeros(cv::Size(1, 5), CV_64F);

  // Stop if camera not calibrated
  if (cinfo->K[0] == 0.0)
    throw std::runtime_error("Camera not calibrated.");

  // TODO: convert to function later
  // Assign camera info only once
  if (!init_cam) {
    for (int i = 0; i < 3; ++i) {
      double *pk = K.ptr<double>(i);
      for (int j = 0; j < 3; ++j) {
        pk[j] = cinfo->K[3 * i + j];
      }
    }
    double *pd = D.ptr<double>();
    for (int k = 0; k < 5; k++) {
      pd[k] = cinfo->D[k];
    }
    init_cam = true;
  }

  // use cv_bridge and convert to grayscale image
  cv_bridge::CvImagePtr cv_ptr;
  // use toCvCopy because we will modify the image
  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);

  cv::Mat image_rgb;
  cv::cvtColor(cv_ptr->image, image_rgb, CV_GRAY2RGB);

#if defined(BUILD_UMICH)
  // Use apriltag_umich
  // Currently not using this version
  static april_tag_family_t *tf = tag36h11_create();
  static april_tag_detector_t *td = april_tag_detector_create(tf);

  image_u8_t *im = image_u8_create_from_gray(
      cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.data);
  zarray_t *detections = april_tag_detector_detect(td, im);

  ROS_INFO("Tags detected: %d", zarray_size(detections));

  for (size_t i = 0; i < zarray_size(detections); i++) {
    april_tag_detection_t *det;
    zarray_get(detections, i, &det);

    for (int j = 0; j < 4; j++) {
      const Point2 p = Point2(det->p[j][0], det->p[j][1]);
    }
    april_tag_detection_destroy(det);
  }

  zarray_destroy(detections);
  image_u8_destroy(im);

#elif defined(BUILD_MIT)
  // Use apriltag_mit
  static AprilTags::TagDetector tag_detector(AprilTags::tagCodes36h11);
  std::vector<AprilTags::TagDetection> detections =
      tag_detector.extractTags(cv_ptr->image);

  // Check detection size, only do work if there's tag detected
  if (detections.size()) {
    std::vector<Point2> pi; // Points in image
    std::vector<Point3> pw; // Points in world
    for (auto it = detections.begin(); it != detections.end(); it++) {
      const int id = it->id;
      const Point2 c2 = Point2(it->cxy.first, it->cxy.second);

      for (int j = 0; j < 4; j++) {
        const Point2 p2 = Point2(it->p[j].first, it->p[j].second);
        pi.push_back(p2);
        Point3 p3(tag_w[id].p[j].x, tag_w[id].p[j].y, 0.0);
        pw.push_back(p3);

        // Display tag corners
        cv::circle(image_rgb, p2, 6, colors[j], 2);
      }
      // Display tag id
      std::ostringstream ss;
      ss << id;
      cv::putText(image_rgb, ss.str(), Point2(c2.x - 5, c2.y + 5),
                  cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 255), 2);
    }

    // Get pose
    static cv::Mat r = cv::Mat::zeros(cv::Size(1, 3), CV_64F);
    static cv::Mat cTw = cv::Mat::zeros(cv::Size(1, 3), CV_64F);
    cv::Mat wTc(cv::Size(3, 3), CV_64F);
    cv::Mat cRw(cv::Size(3, 3), CV_64F), wRc(cv::Size(3, 3), CV_64F);
    cv::solvePnP(pw, pi, K, D, r, cTw, true);
    cv::Rodrigues(r, cRw);
    wRc = cRw.inv();
    wTc = -wRc * cTw;
    cv::Mat q = rodriguesToQuat(r);

    // Publish
    geometry_msgs::PoseStamped pose_cam;
    pose_cam.header = image->header;

    double *pt = wTc.ptr<double>();
    pose_cam.pose.position.x = pt[0];
    pose_cam.pose.position.y = pt[1];
    pose_cam.pose.position.z = pt[2];

    double *pq = q.ptr<double>();
    pose_cam.pose.orientation.w = pq[0];
    pose_cam.pose.orientation.x = pq[1];
    pose_cam.pose.orientation.y = pq[2];
    pose_cam.pose.orientation.z = pq[3];

    pose_pub.publish(pose_cam);

  }
#endif

  // Publish image
  cv_bridge::CvImage cv_image(image->header, sensor_msgs::image_encodings::BGR8,
                              image_rgb);
  image_pub.publish(cv_image.toImageMsg());
  // cv::imshow("image", image_rgb);
  // cv::waitKey(1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "apriltag_ros");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::CameraSubscriber camera_sub =
      it.subscribeCamera("image_raw", 1, cam_callback);
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_cam", 1);

  // Initialize simple test tag position
  // TODO: replace this part with yaml file
  double tag_size = 4.5 / 100;
  double tag_center[4][2] = { { tag_size, tag_size },
                              { tag_size, tag_size * 2 },
                              { tag_size * 2, tag_size },
                              { tag_size * 2, tag_size * 2 } };

  for (int i = 0; i < 4; ++i) {
    double x = tag_center[i][0];
    double y = tag_center[i][1];
    tag_w[i].p[0] = Point2(x - tag_size / 2, y - tag_size / 2);
    tag_w[i].p[1] = Point2(x + tag_size / 2, y - tag_size / 2);
    tag_w[i].p[2] = Point2(x + tag_size / 2, y + tag_size / 2);
    tag_w[i].p[3] = Point2(x - tag_size / 2, y + tag_size / 2);
  }

  image_pub = nh.advertise<sensor_msgs::Image>("image", 1);
  // cv::namedWindow("image", 1);
  ros::spin();

  return 0;
}
