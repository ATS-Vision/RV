#include "buff_tracker/tracker_node.hpp"
#include <buff_interfaces/msg/detail/buff__struct.hpp>
#include <buff_interfaces/msg/detail/buffs__struct.hpp>
#include <buff_interfaces/msg/detail/target_info__struct.hpp>
#include <cmath>
#include <rclcpp/node.hpp>

namespace rm_buff 
{
    BuffTrackerNode::BuffTrackerNode(const rclcpp::NodeOptions & options):rclcpp::Node("buff_tracker",options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting BuffTrackerNode!");
        max_fan_distance_ = this->declare_parameter("max_fan_distance", 10.0);
        // Tracker
        double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
        double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
        double max_match_pitch_diff = this->declare_parameter("tracker.max_match_pitch_diff",1.0);
        tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff,max_match_pitch_diff);
        tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);  //目标跟踪阈值

        lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);   ///目标丢失时间阈值


        // EKF
        // xa = x_fan  xc=x_fan
        // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, pitch v_pitch
        // measurement: xa, ya, za, yaw  pitch
        // f - Process function
        auto f = [this](const Eigen::VectorXd & x) {
        Eigen::VectorXd x_new = x;
        x_new(0) += x(1) * dt_;
        x_new(2) += x(3) * dt_;
        x_new(4) += x(5) * dt_;
        x_new(6) += x(7) * dt_;
        x_new(8) += x(9) * dt_;
        return x_new;
        };

        // J_f - Jacobian of process function
        auto j_f = [this](const Eigen::VectorXd &) {
          Eigen::MatrixXd f(10, 10);
          // clang-format off
          f <<    1,   dt_, 0,   0,   0,   0,   0,   0,   0,  0,
                  0,   1,   0,   0,   0,   0,   0,   0,   0,  0,
                  0,   0,   1,   dt_, 0,   0,   0,   0,   0,  0,
                  0,   0,   0,   1,   0,   0,   0,   0,   0,  0,
                  0,   0,   0,   0,   1,   dt_, 0,   0,   0,  0,
                  0,   0,   0,   0,   0,   1,   0,   0,   0,  0,
                  0,   0,   0,   0,   0,   0,   1,   dt_, 0,  0,
                  0,   0,   0,   0,   0,   0,   0,   1,   0,  0,
                  0,   0,   0,   0,   0,   0,   0,   0,   1,  dt_,
                  0,   0,   0,   0,   0,   0,   0,   0,   0,   1;
          // clang-format on
          return f;
    };

     // h - Observation function
    auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(5);
    double xc = x(0), yc = x(2);
    z(0) = xc;  // xa
    z(1) = yc;  // ya
    z(2) = x(4);            // za
    z(3) = x(6);            // yaw
    z(4) = x(8);           // pitch
    return z;
    };

    // J_h - Jacobian of observation function
    auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h(5, 10);
    // clang-format off
    //      xc   v_xc yc   v_yc za   v_za   yaw   v_yaw   pitch v_pitch
    h <<    1,   0,   0,   0,   0,   0,     0,      0,    0,     0,
            0,   0,   1,   0,   0,   0,     0,      0,    0,     0,
            0,   0,   0,   0,   1,   0,     0,      0,    0,     0,
            0,   0,   0,   0,   0,   0,     1,      0,    0,     0,
            0,   0,    0,   0,  0,    0,    0,      0,     1,    0;
    // clang-format on
    return h;
    };

     // update_Q - process noise covariance matrix 过程噪声协方差矩阵 
    s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 20.0);
    s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 100.0);
    s2qpitch_ = declare_parameter("ekf.sigma2_q_pitch", 800.0);
    auto u_q = [this]() {
    Eigen::MatrixXd q(10, 10);
    double t = dt_, x = s2qxyz_, y = s2qyaw_, p = s2qpitch_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
    double q_p_p = pow(t, 4) / 4 * p, q_p_vp= pow(t,3)/2 * x, q_vp_vp=pow(t,2)*p; 
    // clang-format off
    //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   pitch     v_pitch
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,            0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,            0,
          0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,            0,
          0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,            0,
          0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,            0,
          0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,            0,
          0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,            0,
          0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,            0,
          0,      0,      0,      0,      0,      0,      0,      0,      q_p_p,       q_p_vp,
          0,      0,      0,      0,      0,      0,      0,      0,      q_p_vp,      q_vp_vp;
    // clang-format on
    return q;
  };

    // update_R - measurement noise covariance matrix
  r_xyz_factor = declare_parameter("ekf.r_xyz_factor", 0.05);
  r_yaw = declare_parameter("ekf.r_yaw", 0.02);
  r_pitch=declare_parameter("ekf.r_pitch",0.02);
  auto u_r = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 5> r;
    double x = r_xyz_factor;
    r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw, r_pitch;
    return r;
  };

  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 10> p0;
  p0.setIdentity();
  tracker_->ekf = EKF{f, h, j_f, j_h, u_q, u_r, p0};

   // Reset tracker service
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/reset", [this](
                        const std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
    tracker_->tracker_state = Tracker::LOST;
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Tracker reset!");
    return;
    });

    // Subscriber with tf2 message_filter
    // tf2 relevant
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    // subscriber and filter
    armors_sub_.subscribe(this, "/detector/buffs", rmw_qos_profile_sensor_data);
    target_frame_ = this->declare_parameter("target_frame", "odom");
    tf2_filter_ = std::make_shared<tf2_filter>(
    armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&BuffTrackerNode::buffsCallback, this);

    // Measurement publisher (for debug usage)
    info_pub_ = this->create_publisher<buff_interfaces::msg::TargetInfo>("/tracker/info", 10);

    // Publisher
    target_pub_ = this->create_publisher<buff_interfaces::msg::Target>(
    "/tracker/bufftarget", rclcpp::SensorDataQoS());
    
     // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;
    linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    linear_v_marker_.ns = "linear_v";
    linear_v_marker_.scale.x = 0.03;
    linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.color.a = 1.0;
    linear_v_marker_.color.r = 1.0;
    linear_v_marker_.color.g = 1.0;
    angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    linear_v_marker_.ns = "linear_v";
    linear_v_marker_.scale.x = 0.03;
    linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.color.a = 1.0;
    linear_v_marker_.color.r = 1.0;
    linear_v_marker_.color.g = 1.0;
    angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    angular_v_marker_.ns = "angular_v";
    angular_v_marker_.scale.x = 0.03;
    angular_v_marker_.scale.y = 0.05;
    angular_v_marker_.color.a = 1.0;
    angular_v_marker_.color.b = 1.0;
    angular_v_marker_.color.g = 1.0;
    armor_marker_.ns = "buffs";
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.03;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.r = 1.0;
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);
    }

    void BuffTrackerNode::buffsCallback(const buff_interfaces::msg::Buffs::SharedPtr buffs_msg)
    {
          // Tranform armor position from image frame to world coordinate
        for (auto & buff : buffs_msg->buffs) 
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = buffs_msg->header;
            ps.pose = buff.pose;
            try {
            buff.pose = tf2_buffer_->transform(ps, target_frame_).pose;
            } catch (const tf2::ExtrapolationException & ex) {
            RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
            return;
            }
        }
          // Filter abnormal armors
        buffs_msg->buffs.erase(
        std::remove_if(
        buffs_msg->buffs.begin(), buffs_msg->buffs.end(),
        [this](const buff_interfaces::msg::Buff & buff) {
        return
            Eigen::Vector3d(buff.pose.position.x, buff.pose.position.y,buff.pose.position.z).norm() >
                max_fan_distance_;
        }),
    buffs_msg->buffs.end());

    // Init message
    buff_interfaces::msg::TargetInfo info_msg;
    buff_interfaces::msg::Target target_msg;
    rclcpp::Time time = buffs_msg->header.stamp;
    target_msg.header.stamp = time;
    target_msg.header.frame_id = target_frame_;

    // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(buffs_msg);
    target_msg.tracking = false;
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);
    tracker_->update(buffs_msg);
      // Publish Info
    info_msg.position_diff = tracker_->info_position_diff;
    info_msg.yaw_diff = tracker_->info_yaw_diff;
    info_msg.pitch_diff=tracker_->info_yaw_diff;
    info_msg.position.x = tracker_->measurement(0);
    info_msg.position.y = tracker_->measurement(1);
    info_msg.position.z = tracker_->measurement(2);
    info_msg.yaw = tracker_->measurement(3);
    info_msg.pitch=tracker_->measurement(4);
    info_pub_->publish(info_msg);

    if (tracker_->tracker_state == Tracker::DETECTING) {
    target_msg.tracking = false;
    }else if (tracker_->tracker_state == Tracker::TRACKING ||
        tracker_->tracker_state == Tracker::TEMP_LOST) {
    target_msg.tracking = true;
    const auto & state = tracker_->target_state;
    target_msg.position.x = state(0);   /// x
    target_msg.velocity.x = state(1);   /// vx
    target_msg.position.y = state(2);   /// y
    target_msg.velocity.y = state(3);   /// vy
    target_msg.position.z = state(4);   /// z
    target_msg.velocity.z = state(5);   ///vz
    target_msg.yaw = state(6);          ///yaw
    target_msg.v_yaw = state(7);        ///v_yaw
    target_msg.pitch = state(8);        ///pitch
    target_msg.v_pitch = state(9);   ////v_pitch
    }
    }
    last_time_ = time;
    target_pub_->publish(target_msg);
    publishMarkers(target_msg);
    }

    void BuffTrackerNode::publishMarkers(const buff_interfaces::msg::Target & target_msg)
    {
        position_marker_.header = target_msg.header;
        linear_v_marker_.header = target_msg.header;
        angular_v_marker_.header = target_msg.header;
        armor_marker_.header = target_msg.header;
        visualization_msgs::msg::MarkerArray marker_array;
        if(target_msg.tracking)
        {
            double yaw = target_msg.yaw, pitch = target_msg.pitch;
            double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
            double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
            
            position_marker_.action = visualization_msgs::msg::Marker::ADD;
            position_marker_.pose.position.x = xc;
            position_marker_.pose.position.y = yc;
            position_marker_.pose.position.z = za;

            linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
            linear_v_marker_.points.clear();
            linear_v_marker_.points.emplace_back(position_marker_.pose.position);
            geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
            arrow_end.x += vx;
            arrow_end.y += vy;
            arrow_end.z += vz;
            linear_v_marker_.points.emplace_back(arrow_end);

            angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
            angular_v_marker_.points.clear();
            angular_v_marker_.points.emplace_back(position_marker_.pose.position);
            arrow_end = position_marker_.pose.position;
            arrow_end.z += target_msg.v_yaw / M_PI;
            angular_v_marker_.points.emplace_back(arrow_end);

            armor_marker_.action = visualization_msgs::msg::Marker::ADD;
            geometry_msgs::msg::Point p_a;
            size_t a_n = 1;
            for (size_t i = 0; i < a_n; i++)
            {
              double tmp_yaw = yaw ;
              double tmp_pitch=pitch;
              p_a.z = za;
              p_a.x = xc;
              p_a.y = yc;
              armor_marker_.id = i;
              armor_marker_.pose.position = p_a;
              tf2::Quaternion q;
              q.setRPY(0, tmp_pitch, tmp_yaw);
              armor_marker_.pose.orientation = tf2::toMsg(q);
              marker_array.markers.emplace_back(armor_marker_);
            }
        }else {
          position_marker_.action = visualization_msgs::msg::Marker::DELETE;
          linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
          angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;

          armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
          marker_array.markers.emplace_back(armor_marker_);
          }

          marker_array.markers.emplace_back(position_marker_);
          marker_array.markers.emplace_back(linear_v_marker_);
          marker_array.markers.emplace_back(angular_v_marker_);
          marker_pub_->publish(marker_array);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<rm_buff::BuffTrackerNode>());
    rclcpp::shutdown();

    return 0;
}


#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff::BuffTrackerNode)
