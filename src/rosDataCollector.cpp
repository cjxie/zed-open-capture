#include "videocapture.hpp"
#include "ocv_display.hpp"
#include "sensorcapture.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <thread>
#include <mutex>
#include <string>


const double DEG2RAD = 0.017453293;
std::mutex imuMutex;
bool sensThreadStop=false;
bool imageThreadStop=false;

bool show = false;


class imuImagePublisher : public rclcpp::Node
{
public:
    imuImagePublisher(sl_oc::video::VideoParams &params): Node("imu_image_publisher"), cap_(params), sens_(sl_oc::VERBOSITY::ERROR), last_ts_imu(0)
    {
        this->declare_parameter("fps", 30);
        int value = this->get_parameter("fps").as_int();

        if (value <=15)
        {
            cap_.setFps(15);
        }
        else if(value <= 30)
        {
            cap_.setFps(30);
        }
        else if(value <= 60)
        {
            cap_.setFps(60);
        }
        else
        {
            cap_.setFps(60);
        }

        if(!cap_.initializeVideo(-1))
        {
            RCLCPP_ERROR(this->get_logger(),  "Cannot open camera video capture");
        }
        
        int camSn= cap_.getSerialNumber();
        RCLCPP_INFO(this->get_logger(), "Video Capture connected to camera sn: %d", camSn);

        // sl_oc::sensors::SensorCapture sens;
        if(!sens_.initializeSensors(camSn))
        {
            RCLCPP_ERROR(this->get_logger(),  "Cannot open sensor capture");
        }
        RCLCPP_INFO(this->get_logger(), "Sensors Capture connected to camera sn: %d", sens_.getSerialNumber() );

        cap_.enableSensorSync(&sens_);
        int h, w;
        cap_.getFrameSize(w,h);
        RCLCPP_INFO(this->get_logger(), "Frame size: %d x %d ", w, h);

        // Get the intrinsic and extrinsic

        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/zed2/imu", 1000);
        imageL_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/zed2/left/compressed", 10);
        imageR_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/zed2/right/compressed", 10);
    }

    // ~imuImagePublisher()
    // {
    //     sens_.~SensorCapture();
        
    //     cap_.~VideoCapture();
    // }
  

    void publishImuData()
    {
        while(!sensThreadStop)
        {
            const sl_oc::sensors::data::Imu imu_data = sens_.getLastIMUData(100000);
            uint64_t ts =  imu_data.timestamp;
            rclcpp::Time ts_imu(ts);
            if(ts < last_ts_imu)
            {
                last_ts_imu = ts;
                RCLCPP_INFO(this->get_logger(), "Invalid imu timestamp");
                continue;
            }
        
            // RCLCPP_INFO(this->get_logger(), "imu ts: %f", ts / 1e9);
            double fps = 1e9 / static_cast<double>(ts - last_ts_imu);
            RCLCPP_INFO(this->get_logger(), "imu fps: %f", fps);


            last_ts_imu = ts;
            auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
            imu_msg->header.stamp = ts_imu;
            // imu_msg->orientation.x = imu_data.
            
            imu_msg->angular_velocity.x = imu_data.gX * DEG2RAD;
            imu_msg->angular_velocity.y = imu_data.gY * DEG2RAD;
            imu_msg->angular_velocity.z = imu_data.gZ * DEG2RAD;

            imu_msg->linear_acceleration.x = imu_data.aX;
            imu_msg->linear_acceleration.y = imu_data.aY;
            imu_msg->linear_acceleration.z = imu_data.aZ;
            
            imu_publisher_->publish(std::move(imu_msg));
        }
        
    }
    void publishImageData()
    {
        std::thread sensThread(std::bind(&imuImagePublisher::publishImuData, this));

        uint64_t last_ts = 0; 
        float frame_fps=0;
        sl_oc::video::Frame frame = cap_.getLastFrame(1);
        int h,w;
        cap_.getFrameSize(w, h);

        while(1 && frame.data!=nullptr)
        {
            frame = cap_.getLastFrame(1);
            // RCLCPP_INFO(this->get_logger(), "imageThread1");
            if(frame.data!=nullptr && frame.timestamp !=last_ts)
            {
                cv::Mat frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
                cv::Mat frameBGR;
                cv::cvtColor(frameYUV,frameBGR,cv::COLOR_YUV2BGR_YUYV);
                if (show)
                    sl_oc::tools::showImage( "Stream RGB", frameBGR, sl_oc::video::RESOLUTION::HD720);

                if (frame.timestamp !=last_ts)
                {
                    cv::Mat left = frameBGR(cv::Rect(0,0,int(w/2), h));
                    cv::Mat right = frameBGR(cv::Rect(int(w/2),0,int(w/2), h));
                    
                    frame_fps = 1e9/static_cast<double>(frame.timestamp - last_ts);
                    RCLCPP_INFO(this->get_logger(), "fps : %f", frame_fps);
                    // RCLCPP_INFO(this->get_logger(), "TimeStamp: %f", static_cast<double> (frame.timestamp) / 1e9);
                    
                    
                    rclcpp::Time ts(frame.timestamp);
                    auto imgL_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8", left);
                    imgL_msg.header.stamp = ts;
                    imgL_msg.header.frame_id = "camera_link";
                    imageL_publisher_->publish(std::move(*imgL_msg.toCompressedImageMsg()));

                    auto imgR_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8", right);
                    imgR_msg.header.stamp = ts;
                    imgR_msg.header.frame_id = "camera_link";
                    imageR_publisher_->publish(std::move(*imgR_msg.toCompressedImageMsg()));

                    int key = cv::waitKey( 1 );
                    if(key=='q' || key=='Q') // Quit
                    {
                        break;
                    }
                }
            }  
            // RCLCPP_INFO(this->get_logger(), "TimeStamp: %f", static_cast<double> (last_ts) / 1e9);
            last_ts = frame.timestamp;    
        }
        
        sensThreadStop = true;
        // imageThreadStop = true;
        if(sensThread.joinable())
        {
            RCLCPP_INFO(this->get_logger(), "Imu Thread closed!");
            sensThread.join();
        }

    }
   

private:
    sl_oc::video::VideoCapture cap_;
    sl_oc::sensors::SensorCapture sens_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr imageL_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr imageR_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    uint64_t last_ts_imu;
};



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD720;
    params.fps = sl_oc::video::FPS::FPS_30;
    params.verbose = sl_oc::VERBOSITY::INFO;

    show = true;

    // usea seperate thread for imu data
    auto node = std::make_shared<imuImagePublisher>(params);
    // sensThreadStop = false;
    
    // std::thread sensThread(std::bind(&imuImagePublisher::publishImuData, node));
    std::thread imageThread(std::bind(&imuImagePublisher::publishImageData, node));

    // while (rclcpp::ok())
    // {
    rclcpp::spin(node);
    // }
    rclcpp::shutdown();

    if(imageThread.joinable())
    {
        imageThread.join();
        RCLCPP_INFO(node->get_logger(), "Image Thread closed!");
    }
    return 0;
}

