#ifndef LANE_DETECTION_NODE_H
#define LANE_DETECTION_NODE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/dnn.hpp>

class LaneDetectionNode
{
private:
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;
cv::dnn::Net net_;

// Lane detection parameters
float confidence_threshold_;
cv::Mat mean_;
std::vector<cv::Scalar> lane_colors_;

public:
LaneDetectionNode();

void imageCb(const sensor_msgs::ImageConstPtr &msg);
};

#endif // LANE_DETECTION_NODE_H