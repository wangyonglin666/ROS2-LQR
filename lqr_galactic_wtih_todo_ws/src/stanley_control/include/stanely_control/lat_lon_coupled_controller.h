#ifndef LAT_LON_COUPLED_CONTROLLER_H_
#define LAT_LON_COUPLED_CONTROLLER_H_

#include "stanely_control/common.h"
#include "stanely_control/pid_controller.h"
#include "stanely_control/stanely_controller.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <lgsvl_msgs/msg/vehicle_control_data.hpp>
#include <fstream>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class StanelyAndPIDController : public rclcpp::Node
{
    public:
        StanelyAndPIDController();
        ~StanelyAndPIDController();
        double PointDistance(const TrajectoryPoint& point, const double x, const double y);
        // double PIDController(); // 定义纵向速度控制器
        void OdomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

    public:
        VehicleState vehicle_state_;

        TrajectoryPoint goal_point;

        bool first_record_;
        double V_set_; // m/s
        double wheelbase_;
        double car_length_;

    public:
        rclcpp::TimerBase::SharedPtr vehicle_control_iteration_timer;
        void VehicleControllerIterationCallback(); // 定时器定时回调函数

        rclcpp::TimerBase::SharedPtr global_path_publish_timer; 
        void GlobalPathPublushCallback();

        TrajectoryData planning_published_trajectory;

        ControlCmd cmd;

        std::unique_ptr<shenlan::control::PIDController> pid_controller_longitudinal;
        std::unique_ptr<shenlan::control::StanleyController> stanely_controller_lateral;

    public:
        rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr vehicle_control_publisher;
        lgsvl_msgs::msg::VehicleControlData control_cmd;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_data_subscriber;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr history_path_visualization_publisher;
        nav_msgs::msg::Path history_path;
        geometry_msgs::msg::PoseStamped history_path_points;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_publisher_;
        nav_msgs::msg::Path global_path;
        geometry_msgs::msg::PoseStamped this_pose_stamped;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_gps_vehicle;
};

#endif