#include "buff_detector/detector_node.hpp"
#include "buff_detector/detector.hpp"
#include "buff_detector/param_struct.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <buff_interfaces/msg/detail/buff__struct.hpp>
#include <buff_interfaces/msg/detail/buffs__struct.hpp>
#include <iostream>
#include <iterator>
#include <memory>
#include <opencv2/videoio.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>



namespace rm_buff 
{
    BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions & options)
    : Node("buff_detector", options)
    {
        std::cout<<"start"<<std::endl;
        RCLCPP_INFO(this->get_logger(), "Starting BUFFDetectorNode!");
        detector_=initDetector();
        detector_->buff_detector_.initModel(path_param_.network_path);
        buffs_pub_=this->create_publisher<buff_interfaces::msg::Buffs>(
            "detector/buffs",  rclcpp::SensorDataQoS());
        buff_marker_.ns="buffs";
        buff_marker_.action=visualization_msgs::msg::Marker::ADD;
        buff_marker_.type=visualization_msgs::msg::Marker::CUBE;
        buff_marker_.scale.x=0.05;
        buff_marker_.scale.y=0.125;
        buff_marker_.color.a=1.0;
        buff_marker_.color.b=1.0;
        buff_marker_.color.g=0.5;
        buff_marker_.lifetime=rclcpp::Duration::from_seconds(0.1);

        text_marker_.ns = "classification";
        text_marker_.action = visualization_msgs::msg::Marker::ADD;
        text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker_.scale.z = 0.1;
        text_marker_.color.a = 1.0;
        text_marker_.color.r = 1.0;
        text_marker_.color.g = 1.0;
        text_marker_.color.b = 1.0;
        text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

        marker_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/buff_marker", 10);

        debug_ = this->declare_parameter("debug", false);
        if (debug_) {
        createDebugPublishers();
        }

        // Debug param change moniter
        debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        debug_cb_handle_ =
            debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
            debug_ = p.as_bool();
            debug_ ? createDebugPublishers() : destroyDebugPublishers();
            });

        cam_info_sub_ = 
            this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera_info", rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
                cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
                cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
                pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
                cam_info_sub_.reset();
                });

        img_sub_ = 
            this->create_subscription<sensor_msgs::msg::Image>("/image_raw", rclcpp::SensorDataQoS(),std::bind(&BuffDetectorNode::imageCallback, this, std::placeholders::_1));


    }

    BuffDetectorNode::~BuffDetectorNode(){}

    void BuffDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        std::stringstream latency_ss;
        auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
        if(img.empty())
        {
            std::cout<<"image empty"<<std::endl;
        }else{
            std::cout<<"have image"<<std::endl;
        }
        auto fans=detector_->run(img);
        if (pnp_solver_ != nullptr) 
        {
            buffs_msg_.header = buff_marker_.header = text_marker_.header = img_msg->header;
            buffs_msg_.buffs.clear();
            marker_array_.markers.clear();
            buff_marker_.id = 0;
            text_marker_.id = 0;
        }

        buff_interfaces::msg::Buff buff_msg;
        for(const auto &  fan:fans)
        {
            cv::Mat rvec, tvec;
            Eigen::Vector3d tvec_eigen;
            bool success = pnp_solver_->solvePnP(fan, rvec, tvec);
            if(success)
            {
                buff_msg.pose.position.x = tvec.at<double>(0);
                buff_msg.pose.position.y = tvec.at<double>(1);
                buff_msg.pose.position.z = tvec.at<double>(2);
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvec, rotation_matrix);
                // rotation matrix to quaternion
                tf2::Matrix3x3 tf2_rotation_matrix(
                rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
                rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
                rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
                rotation_matrix.at<double>(2, 2));
                tf2::Quaternion tf2_q;
                tf2_rotation_matrix.getRotation(tf2_q);
                buff_msg.pose.orientation = tf2::toMsg(tf2_q);
                buff_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(fan.center);
                buff_marker_.id++;
                buff_marker_.pose = buff_msg.pose;
                text_marker_.id++;
                text_marker_.pose.position = buff_msg.pose.position;
                text_marker_.pose.position.y -= 0.1;
                buffs_msg_.buffs.emplace_back(buff_msg);
                marker_array_.markers.emplace_back(buff_marker_);
                marker_array_.markers.emplace_back(text_marker_);
            }else {
                RCLCPP_WARN(this->get_logger(), "PnP failed!");
            }
            
        }
    }

    std::unique_ptr<Detector> BuffDetectorNode::initDetector()
    {
        this->declare_parameter<int>("color", 1);
        this->declare_parameter<std::string>("network_path", "/model/buff.xml");
        //std::string pkg_share_pth=ament_index_cpp::get_package_share_directory("buff_detector");
        this->path_param_.network_path = this->get_parameter("network_path").as_string();
        this->get_parameter("color", this->buff_param_.color);
        return std::make_unique<Detector>(buff_param_,path_param_);
    }

    void BuffDetectorNode::createDebugPublishers()
    {
        result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
    }

    void BuffDetectorNode::destroyDebugPublishers()
    {
        result_img_pub_.shutdown();
    }

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<rm_buff::BuffDetectorNode>());
    rclcpp::shutdown();

    return 0;
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff::BuffDetectorNode)