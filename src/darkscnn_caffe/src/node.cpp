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
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv::dnn::Net net_;

    // Lane detection parameters
    double confidence_threshold_;
    cv::Mat mean_;
    std::vector<cv::Scalar> lane_colors_;
    double alpha_;
    std::string lane_input_topic;
    std::string lane_visual_topic; // image topic for visualization




public:
    LaneDetectionNode() : nh_(), pnh_("~"), it_(nh_), confidence_threshold_(0.95) {

        std::string model_path, weight_path;
        pnh_.param<std::string>("model_path", model_path, "");
        pnh_.param<std::string>("weight_path", weight_path, "");

        // Load lane detection parameters
        pnh_.param<double>("confidence_threshold", confidence_threshold_, 0.95);
        pnh_.param<double>("alpha", alpha_, 0.75);

        pnh_.param<std::string>("lane_input_topic", lane_input_topic, "input/image");
        pnh_.param<std::string>("lane_visual_topic", lane_visual_topic, "output/image");
        

        // Load Caffe model
        net_ = cv::dnn::readNetFromCaffe(model_path, weight_path);
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);

        // Load lane detection parameters
        cv::Mat new_mean = cv::Mat(1, 1, CV_32FC3, cv::Scalar(95, 99, 96));
        mean_ = new_mean;


std::vector<cv::Scalar> new_lane_colors = {
            cv::Scalar(0, 0, 0),     // Background
            cv::Scalar(0, 0, 255),   // Red
            cv::Scalar(0, 255, 0),   // Green
            cv::Scalar(255, 0, 0),   // Blue
            cv::Scalar(0, 255, 255), // Yellow
            cv::Scalar(255, 255, 0), // Cyan
            cv::Scalar(255, 0, 255), // Magenta
            cv::Scalar(128, 0, 0),   // Dark Blue
            cv::Scalar(0, 128, 0),   // Dark Green
            cv::Scalar(0, 0, 128),   // Dark Red
            cv::Scalar(128, 128, 0), // Olive
            cv::Scalar(128, 0, 128), // Purple
            cv::Scalar(0, 128, 128), // Teal
};
lane_colors_ = new_lane_colors;


        // Subscribe to input image and advertise output image
        image_sub_ = it_.subscribe("input/image", 1, &LaneDetectionNode::imageCb, this);
        image_pub_ = it_.advertise("output/image", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Preprocess input image
        cv::Mat inputBlob = cv::dnn::blobFromImage(cv_ptr->image, 1.0 / 255.0, cv::Size(640, 480));
        net_.setInput(inputBlob);

        // Run neural network on input image
        cv::Mat outputBlob = net_.forward();

        // Post-process output blob
        cv::Mat mask_color(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
        for (int i = 0; i < outputBlob.size[1]; i++)
        {
            cv::Mat lane_mask(outputBlob.size[2], outputBlob.size[3], CV_32F, outputBlob.ptr<float>(0, i));
            cv::Mat lane_mask_binary = (lane_mask >= confidence_threshold_);

            cv::Scalar lane_color = lane_colors_[i];
            cv::Mat lane_mask_color(480, 640, CV_8UC3, lane_color);
            lane_mask_color.setTo(cv::Scalar(0, 0, 0), ~lane_mask_binary);

            mask_color.setTo(lane_color, lane_mask_binary);
            mask_color = mask_color | lane_mask_color;
        }

        // Overlay mask on input image
        double alpha = 0.75;
        cv::Mat outputImage;
        cv::addWeighted(cv_ptr->image, alpha, mask_color, 1.0 - alpha, 0.0, outputImage, CV_8UC3);

        // Publish output image
        sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg();
        image_pub_.publish(output_msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detection_node");
    LaneDetectionNode lane_detection_node;
    ros::spin();
    return 0;
}
