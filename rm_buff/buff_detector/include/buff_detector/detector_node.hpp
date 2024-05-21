#ifndef BUFF_DETECTOR_NODE_HPP_
#define BUFF_DETECTOR_NODE_HPP_


#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>

#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include "buff_detector/detector.hpp"
#include "buff_detector/param_struct.hpp"
#include "buff_detector/window.hpp"
#include "buff_interfaces/msg/buff.hpp"
#include "buff_interfaces/msg/buffs.hpp"

//using namespace ament_index_cpp;
namespace rm_buff
{
    class BuffDetectorNode : public rclcpp::Node
    {
        public:
            BuffDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            ~BuffDetectorNode();
        
        private:
            void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
            void createDebugPublishers();
            void destroyDebugPublishers();

            std::unique_ptr<Detector> detector_;
            std::unique_ptr<Detector> initDetector();
            
            BuffParam buff_param_;
            PathParam path_param_;

            buff_interfaces::msg::Buffs buffs_msg_;
            rclcpp::Publisher<buff_interfaces::msg::Buffs>::SharedPtr buffs_pub_;


            visualization_msgs::msg::Marker buff_marker_;
            visualization_msgs::msg::Marker text_marker_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
            visualization_msgs::msg::MarkerArray marker_array_;

            bool debug_;
            std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
            std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
            image_transport::Publisher result_img_pub_;


            rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
            cv::Point2f cam_center_;
            std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
            std::unique_ptr<PnPSolver> pnp_solver_;

            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

            


    };

}

#endif