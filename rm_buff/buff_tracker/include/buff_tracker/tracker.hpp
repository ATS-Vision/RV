#ifndef BUFF_TRACKER_HPP_
#define BUFF_TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <buff_interfaces/msg/detail/buff__struct.hpp>
#include <buff_interfaces/msg/detail/buffs__struct.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>
#include <string>

#include "buff_tracker/ekf.hpp"
#include "buff_interfaces/msg/buffs.hpp"
#include "buff_interfaces/msg/target.hpp"

namespace rm_buff
{
    enum BuffNumber{Normal=1};
    class Tracker
    {
    public:
        Tracker(double max_match_distance, double max_match_yaw_diff, double max_match_pitch_diff);
        using Fans=buff_interfaces::msg::Buffs;
        using Fan =buff_interfaces::msg::Buff;
        void init(const Fans::SharedPtr& fans_msg);
        void update(const Fans::SharedPtr& fans_msg);
        EKF ekf;
        int tracking_thres;
        int lost_thres;
        enum State {
                    LOST,
                    DETECTING,
                    TRACKING,
                    TEMP_LOST,
                } tracker_state;
        std::string tracked_id;
        Fan tracked_fan;
        BuffNumber tracked_fans_num;
        double info_position_diff;
        double info_yaw_diff;
        double info_pitch_diff;
        Eigen::VectorXd measurement;
        Eigen::VectorXd target_state;
        double v_pitch;

    private:
        void initEKF(const Fan & f);
        double orientationToYaw(const geometry_msgs::msg::Quaternion & q);
        double orientationToPitch(const geometry_msgs::msg::Quaternion & q);
        Eigen::Vector3d getFanPositionFromState(const Eigen::VectorXd & x);
        double max_match_distance_;
        double max_match_yaw_diff_;
        double max_match_pitch_diff_;

        int detect_count_;
        int lost_count_;

        double last_yaw_;
        double last_pitch_;
    };
}

#endif