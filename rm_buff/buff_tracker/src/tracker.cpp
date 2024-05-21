#include "buff_tracker/tracker.hpp"
#include <angles/angles.h>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <memory>
#include <string>

namespace rm_buff
{
    Tracker::Tracker(double max_match_distance, double max_match_yaw_diff,double max_match_pitch_diff)
    :tracker_state(LOST),
    measurement(Eigen::VectorXd::Zero(5)),
    target_state(Eigen::VectorXd::Zero(10)),
    max_match_distance_(max_match_distance),
    max_match_yaw_diff_(max_match_yaw_diff),
    max_match_pitch_diff_(max_match_pitch_diff)
    {}

    void Tracker::init(const Fans::SharedPtr & fans_msg)
    {
        if (fans_msg->buffs.empty()) 
        {
        return;
        }
         // Simply choose the armor that is closest to image center
        double min_distance = DBL_MAX;
        tracked_fan = fans_msg->buffs[0];
        for(const auto & fan:fans_msg->buffs){
            if(fan.distance_to_image_center<min_distance){
                min_distance=fan.distance_to_image_center;
                tracked_fan=fan;
            }
        }
        initEKF(tracked_fan);
        RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "Init EKF!");
        tracker_state = DETECTING;
    }

    void Tracker::update(const Fans::SharedPtr & fans_msg)
    {
        // KF predict
        Eigen::VectorXd ekf_prediction = ekf.predict();
        RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "EKF predict");
        // Use KF prediction as default target state if no matched fan is found
        target_state = ekf_prediction;
        bool matched=false;
        if (!fans_msg->buffs.empty())
        {
            auto predicted_position = getFanPositionFromState(ekf_prediction);
            double min_position_diff = DBL_MAX;
            double yaw_diff = DBL_MAX;
            double pitch_diff=DBL_MAX;
            
            for (auto fan : fans_msg->buffs) 
            {
                auto p = fan.pose.position;
                Eigen::Vector3d position_vec(p.x, p.y, p.z);
                double position_diff = (predicted_position - position_vec).norm();
                if (position_diff < min_position_diff) {
                // Find the closest armor
                min_position_diff = position_diff;
                yaw_diff = abs(orientationToYaw(fan.pose.orientation) - ekf_prediction(6));
                pitch_diff=abs(orientationToPitch(fan.pose.orientation)-ekf_prediction(8));
                tracked_fan = fan;
                }
            }
            // Store tracker info
        info_position_diff = min_position_diff;
        info_yaw_diff = yaw_diff;
        // Check if the distance and yaw difference of closest armor are within the threshold
    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_ && pitch_diff<max_match_pitch_diff_) {
      // Matched armor found
        matched = true;
        auto p = tracked_fan.pose.position;
        // Update EKF
        double measured_yaw = orientationToYaw(tracked_fan.pose.orientation);
        double measured_pitch=orientationToPitch(tracked_fan.pose.orientation);
        measurement << p.x,p.y,p.z,measured_yaw,measured_pitch;
        target_state = ekf.update(measurement);
        RCLCPP_DEBUG(rclcpp::get_logger("buff_tracker"), "EKF update");
    }else {
      // No matched armor found
        RCLCPP_WARN(rclcpp::get_logger("buff_tracker"), "No matched fan found!");
    }
        }
        if(tracker_state==DETECTING)
        {
            if(matched)
            {
                detect_count_++;
                if (detect_count_ > tracking_thres) 
                {
                detect_count_ = 0;
                tracker_state = TRACKING;
                }
            }else {
            detect_count_ = 0;
            tracker_state = LOST;
            }
        }else if(tracker_state == TRACKING)
        {
            if(!matched)
            {
                tracker_state = TEMP_LOST;
                lost_count_++;
            }
        }else if (tracker_state == TEMP_LOST) 
        {
            if (!matched) {
            lost_count_++;
            if (lost_count_ > lost_thres) {
                lost_count_ = 0;
                tracker_state = LOST;
            }
            } else {
            tracker_state = TRACKING;
            lost_count_ = 0;
            }
        }

    }

    void Tracker::initEKF(const Fan & f)
    {
         // 提取目标位置的 x, y, z 坐标
        double xa = f.pose.position.x;
        double ya = f.pose.position.y;
        double za = f.pose.position.z;
        last_yaw_ = 0;
        last_pitch_=0;
        double yaw = orientationToYaw(f.pose.orientation);
        double pitch =orientationToPitch(f.pose.orientation);

        //  初始化目标状态向量为零向量，维度为 9
        target_state = Eigen::VectorXd::Zero(10);
        double xc = xa;
        double yc = ya;
        target_state << xc, 0, yc, 0, za, 0, yaw, 0, pitch, 0;
        ekf.setState(target_state);
    }

    Eigen::Vector3d Tracker::getFanPositionFromState(const Eigen::VectorXd & x)
    {
    // Calculate predicted position of the current armor
    double xc = x(0), yc = x(2), za = x(4);
    double xa = xc ;
    double ya = yc ;
    return Eigen::Vector3d(xa, ya, za);
    }

    double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
    {
    // Get armor yaw
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    // Make yaw change continuous (-pi~pi to -inf~inf)
    yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
    last_yaw_ = yaw;
    return yaw;
    }

    double Tracker::orientationToPitch(const geometry_msgs::msg::Quaternion & q)
    {
        tf2::Quaternion tf_q;
        tf2::fromMsg(q,tf_q);
        double roll,pitch,yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        pitch=last_pitch_+angles::shortest_angular_distance(last_pitch_, pitch);
        last_pitch_=pitch;
        return pitch;
    }

}