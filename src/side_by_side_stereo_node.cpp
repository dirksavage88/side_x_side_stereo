// (MIT License)
//
// Copyright 2019 David B. Curtis
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////
//
// Subscribes to an image stream of side-by-side stereo where each image
// message consists of a left and right image concatenated to form a single
// double-wide image.  This node splits the incoming image down the middle
// and republishes each half as stereo/left and stereo/right images.
//
// This is a modified version of public domain code posted by PeteBlackerThe3rd
// in response to my question on ROS Answers:
// https://answers.ros.org/question/315298/splitting-side-by-side-video-into-stereoleft-stereoright/
//
// -- dbc

#include <rclcpp/rclcpp.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <chrono>
#include <functional>
#include <string>

// If non-zero, outputWidth and outputHeight set the size of the output images.
// If zero, the outputWidth is set to 1/2 the width of the input image, and
// outputHeight is the same as the height of the input image.

using namespace std::chrono_literals;

class SplitImagePair : public rclcpp::Node
{
    public:
        SplitImagePair(): Node("split_image_pair")
        {
            // Declare params and type
            this->declare_parameter("left_output_image_topic", rclcpp::PARAMETER_STRING);
            this->declare_parameter("right_output_image_topic", rclcpp::PARAMETER_STRING);
            this->declare_parameter("left_camera_info_topic", rclcpp::PARAMETER_STRING);
            this->declare_parameter("right_camera_info_topic", rclcpp::PARAMETER_STRING);
            this->declare_parameter("output_width", rclcpp::PARAMETER_INTEGER);
            this->declare_parameter("output_height", rclcpp::PARAMETER_INTEGER);
            this->declare_parameter("input_image_topic", rclcpp::PARAMETER_STRING);
            

            // Bind timer cb to object
            // timer_ptr_ = this->create_wall_timer(1000ms, std::bind(&SplitImagePair::timer_callback, this));

            std::string inputImageTopic = this->get_parameter("input_image_topic").as_string();

            
            RCLCPP_INFO(this->get_logger(), "input topic to stereo splitter=%s\n", inputImageTopic.c_str());
            // Get params
            std::string left_output_image_topic = this->get_parameter("left_output_image_topic").as_string();
            std::string right_output_image_topic = this->get_parameter("right_output_image_topic").as_string();
            std::string left_camera_info_topic = this->get_parameter("left_camera_info_topic").as_string();
            std::string right_camera_info_topic = this->get_parameter("right_camera_info_topic").as_string();
            auto outputWidth = this->get_parameter("output_width").as_int();
            auto outputHeight = this->get_parameter("output_height").as_int();
            
            // Image publisher image transport instance TODO: move outside of function? 
            rclcpp::NodeOptions options;
            rclcpp::Node::SharedPtr imaget_ = rclcpp::Node::make_shared("image_publisher", options);
            image_transport::ImageTransport it(imaget_);
            pub_l_ = it.advertise("left/image_raw", 1);
            pub_r_ = it.advertise("right/image_raw", 1);

        }

       void timer_callback()
        {


            // std::string inputImageTopic = this->get_parameter("input_image_topic").as_string();
            // 
            //
            // RCLCPP_INFO(this->get_logger(), "input topic to stereo splitter=%s\n", inputImageTopic.c_str());
            // 
            // // Get params
            // std::string left_output_image_topic = this->get_parameter("left_output_image_topic").as_string();
            // std::string right_output_image_topic = this->get_parameter("right_output_image_topic").as_string();
            // std::string left_camera_info_topic = this->get_parameter("left_camera_info_topic").as_string();
            // std::string right_camera_info_topic = this->get_parameter("right_camera_info_topic").as_string();
            // auto outputWidth = this->get_parameter("output_width").as_int();
            // auto outputHeight = this->get_parameter("output_height").as_int();

        }

        // Image capture callback.
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
        {

            int outputWidth, outputHeight;
            sensor_msgs::msg::CameraInfo info_left, info_right;
            
            // RCLCPP_INFO(this->get_logger(), "Subscribing to input image topic");
            
            // Get double camera image.
            cv_bridge::CvImagePtr cvImg = cv_bridge::toCvCopy(msg, msg->encoding);
            cv::Mat image = cvImg->image;

            // Created shared ptr to image 
            auto img = std::make_shared<sensor_msgs::msg::Image>(); 
            
            // If there are any subscribers to either output topic then publish images
            // on them.
            if (pub_l_.getNumSubscribers() > 0u ||
                pub_r_.getNumSubscribers() > 0u)
            {
                // RCLCPP_INFO(this->get_logger(), "splitting images");
                // Define the relevant rectangles to crop.
                cv::Rect leftROI, rightROI;
                leftROI.y = rightROI.y = 0;
                leftROI.width = rightROI.width = image.cols / 2;
                leftROI.height = rightROI.height = image.rows;
                leftROI.x = 0;
                rightROI.x = image.cols / 2;

                // Crop images.
                cv::Mat leftImage = cv::Mat(image, leftROI);
                cv::Mat rightImage = cv::Mat(image, rightROI);

                // Apply scaling, if specified.
                bool use_scaled;
                cv::Mat leftScaled, rightScaled;
                if (use_scaled = (outputWidth > 0 && outputHeight > 0))
                {
                    cv::Size sz = cv::Size(outputWidth, outputHeight);
                    cv::resize(leftImage, leftScaled, sz);
                    cv::resize(rightImage, rightScaled, sz);
                }


                // Load camera info
                camera_info_manager::CameraInfoManager cimright(this);
                camera_info_manager::CameraInfoManager cimleft(this);

                // // Get camera info
                cimleft.getCameraInfo();
                cimright.getCameraInfo(); 

                cimleft.loadCameraInfo("");
                cimright.loadCameraInfo("");    

                // pub_img_l_ = create_publisher<sensor_msgs::msg::Image>("left/image_raw");
                // pub_img_l_ = create_publisher<sensor_msgs::msg::Image>("right/image_raw");
                // CV image bridge
                cv_bridge::CvImage cvImage;
                
                // RCLCPP_INFO(this->get_logger(), "Set up image pub topics");
                
                cvImage.encoding = msg->encoding;
                cvImage.header.frame_id = msg->header.frame_id;
                cvImage.header.stamp = msg->header.stamp;
                if (pub_img_l_->get_subscription_count() > 0u
                    || pub_cam_info_l_->get_subscription_count() > 0u)
                {

                    // RCLCPP_INFO(this->get_logger(), "fill in image pub message");
                    cvImage.image = use_scaled ? leftScaled : leftImage;
                    //TODO:fix
                    img = cvImage.toImageMsg();
                    pub_l_.publish(img);
                    info_left.header.stamp = img->header.stamp;
                    info_left.header.frame_id = img ->header.frame_id;
                    pub_cam_info_l_->publish(info_left);


                }
                if (pub_img_r_->get_subscription_count() > 0u
                    || pub_cam_info_r_->get_subscription_count() > 0u)
                {
                    cvImage.image = use_scaled ? rightScaled : rightImage;
                    img = cvImage.toImageMsg();
                    pub_r_.publish(img);
                    info_right.header.stamp = img->header.stamp;
                    info_right.header.frame_id = img->header.frame_id;
                    pub_cam_info_r_->publish(info_right);
                }
                // TODO: De-allocate CameraInfoManagers.
                // pub_cam_info_l.~CameraInfoManager();
                // pub_cam_info_r.~CameraInfoManager();

            }
        }


        ~SplitImagePair(){}
    private:
        //Publisher shared ptrs (formerly node handles)
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_l_, pub_img_r_;
        // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        // Set up image transports and camera info msg to fill with img data
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_cam_info_l_ = create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 1);
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_cam_info_r_ = create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 1);
        rclcpp::TimerBase::SharedPtr timer_ptr_;
        image_transport::Publisher pub_l_;
        image_transport::Publisher pub_r_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // auto node_ = std::make_shared<SplitImagePair>();
    SplitImagePair split_im;
    SplitImagePair *splitim_ptr;
    splitim_ptr = &split_im;
    auto rawimage = std::make_shared<rclcpp::Node>("image_sub_cb");
    image_transport::ImageTransport it(rawimage);
    // Create image pub advertise
    // rclcpp::spinscriber input image
    // Foxy QoS
    // rclcpp::QoS video_qos(10);
    // video_qos.keep_last(10);
    // video_qos.best_effort();
    // video_qos.durability_volatile();
    // auto subscription_ = rawimage->create_subscription<sensor_msgs::msg::Image>("image_raw", video_qos, std::bind(&SplitImagePair::imageCallback, splitim_ptr, std::placeholders::_1));
    image_transport::Subscriber sub = it.subscribe("image_raw", 1, std::bind(&SplitImagePair::imageCallback, splitim_ptr, std::placeholders::_1));
    rclcpp::spin(rawimage);

    rclcpp::shutdown();
}
