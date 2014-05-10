/*
 *	main.cpp
 *	Demo code for detecting AprilTags in ROS
 *	Added by gareth on April 28th, 2014
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

#define BUILD_MIT

#if defined(BUILD_UMICH)
extern "C" {
#	include "apriltag/apriltag.h"
#	include "apriltag/image_u8.h"
#	include "apriltag/tag36h11.h"
#	include "apriltag/zarray.h"
}
#elif defined(BUILD_MIT)
#	include "AprilTags/TagDetector.h"
#	include "AprilTags/Tag36h11.h"
#endif

void cam_callback(const sensor_msgs::Image::ConstPtr& img)
{
	//	convert to greyscale
    cv::Mat image_gray;
    
    if (img->encoding == sensor_msgs::image_encodings::MONO8)
    {
        //  use existing data
        image_gray = cv::Mat(cv::Size(img->width, img->height), CV_8UC1, const_cast<uchar*>(&img->data[0]), img->step);
        
        //ROS_INFO("Made MONO8 image");
    } 
    else {
        ROS_WARN("Unsupported image encoding: %s", img->encoding.c_str());
        return;
    }
    
    std::vector <cv::Point2f> corners;
    
    cv::Mat display;
    cv::cvtColor(image_gray,display,CV_GRAY2RGB);
    
#if defined(BUILD_UMICH)

    //ROS_INFO("Looking for tags");
    
	static april_tag_family_t *tf = tag36h11_create();
	static april_tag_detector_t *td = april_tag_detector_create(tf);

    image_u8_t *im = image_u8_create(image_gray.cols, image_gray.rows);
    //  lazy, just copy for now
    memcpy(im->buf, image_gray.ptr(), image_gray.cols * image_gray.rows);

    //  detect tags
    zarray_t *detections = april_tag_detector_detect(td, im);
    
    for (int i = 0; i < zarray_size(detections); i++) 
    {
        april_tag_detection_t *det;
        zarray_get(detections, i, &det);
        
        for (int j=0; j < 4; j++) {
            corners.push_back(cv::Point2f(det->p[j][0], det->p[j][1]));
        }
        
        april_tag_detection_destroy(det);
    }
    
    zarray_destroy(detections);
    image_u8_destroy(im);
    
#elif defined(BUILD_MIT)

    static AprilTags::TagDetector tag_detector(AprilTags::tagCodes36h11);
    vector<AprilTags::TagDetection> detections = tag_detector.extractTags(image_gray);
    
    for (vector<AprilTags::TagDetection> :: iterator I = detections.begin(); I != detections.end(); I++)
    {
        for (int j=0; j < 4; j++)
        {
           const cv::Point2f p = cv::Point2f(I->p[j].first, I->p[j].second);
           corners.push_back(p);
        }
    }
    
#endif
    
    ROS_INFO("Found %lu tags\n", corners.size());
    
    //  draw and display
    for (size_t i=0; i < corners.size(); i++) {
        
       cv::circle(display, corners[i], 8, cv::Scalar(255,0,0,0), 2);
    }
    
    cv::imshow("camera", display);
    cv::waitKey(1);
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "test_apriltags");
	ros::NodeHandle n;
	image_transport::ImageTransport imgTransport(n);

	//	subscribe to camera
	image_transport::Subscriber camSub = imgTransport.subscribe("/camera/image_raw", 1, cam_callback);

	//	output for OpenCV
	cv::namedWindow("camera", 1);

	ros::spin();
	return 0;
}
