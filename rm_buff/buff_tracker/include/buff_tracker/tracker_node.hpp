#ifndef BUFF_TRACKER_NODE_HPP_
#define BUFF_TRACKER_NODE_HPP_
// ROS
#include <buff_interfaces/msg/detail/buffs__struct.hpp>
#include <buff_interfaces/msg/detail/target_info__struct.hpp>
#include <message_filters/subscriber.h>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// STD
#include <memory>
#include <string>
#include <vector>

#include "buff_tracker/tracker.hpp"
#include "buff_interfaces/msg/buffs.hpp"
#include "buff_interfaces/msg/target.hpp"
#include "buff_interfaces/msg/target_info.hpp"

namespace rm_buff
{
    using tf2_filter = tf2_ros::MessageFilter<buff_interfaces::msg::Buffs>;
    class BuffTrackerNode :public rclcpp::Node
    {
        public:
            explicit BuffTrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        private:
            void buffsCallback(const buff_interfaces::msg::Buffs::SharedPtr buff_ptr);
            void publishMarkers(const buff_interfaces::msg::Target & target_msg);

            // Maximum allowable armor distance in the XOY plane
            double max_fan_distance_;

            // The time when the last message was received
            rclcpp::Time last_time_;
            double dt_;

            // Armor tracker
            double s2qxyz_, s2qyaw_, s2qpitch_;
            double r_xyz_factor, r_yaw, r_pitch;
            double lost_time_thres_;
            std::unique_ptr<Tracker> tracker_;

            // Reset tracker service
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

            // Subscriber with tf2 message_filter
            std::string target_frame_;
            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
            message_filters::Subscriber<buff_interfaces::msg::Buffs> armors_sub_;
            std::shared_ptr<tf2_filter> tf2_filter_;

            // Tracker info publisher
            rclcpp::Publisher<buff_interfaces::msg::TargetInfo>::SharedPtr info_pub_;
            
            // Publisher
            rclcpp::Publisher<buff_interfaces::msg::Target>::SharedPtr target_pub_;

            // Visualization marker publisher
            visualization_msgs::msg::Marker position_marker_;
            visualization_msgs::msg::Marker linear_v_marker_;
            visualization_msgs::msg::Marker angular_v_marker_;
            visualization_msgs::msg::Marker armor_marker_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    };

}
#endif