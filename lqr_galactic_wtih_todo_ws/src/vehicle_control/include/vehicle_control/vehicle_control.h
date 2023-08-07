#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include "common.h"
#include "rclcpp/rclcpp.hpp"
#include "pid_controller.h"

using std::placeholders::_1;

class VehicleControlPublisher : public rclcpp::Node
{
public:
    VehicleControlPublisher();
    ~VehicleControlPublisher();

    double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y);
    
    TrajectoryPoint QueryNearestPointByPosition(const double x, const double y);

    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

    void VehicleControlIterationCallback(); // 定时器定时回调函数

public:
    double V_set_;
    double T_gap_;

    bool first_record_;
    bool cout_distance_;
    bool cout_speed_;

    int cnt;
    int qos;

    double controller_frequency;

    double acceleration_cmd;
    double yaw_cmd;

    std::vector<TrajectoryPoint> trajectory_points_;
    TrajectoryData planning_published_trajectory;

    // Input
    VehicleState vehicle_state_;

    tf2::Quaternion localization_quaternion_transform;

    rclcpp::TimerBase::SharedPtr vehicle_control_iteration_timer_;

    rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr vehicle_control_publisher;
    lgsvl_msgs::msg::VehicleControlData control_cmd;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_data_subscriber;

    
};

#endif