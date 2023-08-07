#include "lqr_controller/lqr_lateral_pid_longitudinal.h"

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// using namespace std;
using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("lqr_lateral_pid_longitudinal");

LQRControllerNode::LQRControllerNode() : Node("lqr_lateral_pid_longitudinal") {
    std::string vehicle_odom_topic;
    std::string vehicle_cmd_topic;
    std::string roadmap_path;
    std::string path_vis_topic;
    std::string frame_id;
    std::string vehicle_imu_topic;
    double speed_P, speed_I, speed_D, target_speed, vis_frequency;

    this->declare_parameter<std::string>("vehicle_odom_topic", vehicle_odom_topic);    //读取车辆定位的topic名
    this->declare_parameter<std::string>("vehicle_cmd_topic", vehicle_cmd_topic);      //读取车辆控制的topic名
    this->declare_parameter<std::string>("roadmap_path", roadmap_path);                //读取路网文件名
    this->declare_parameter<std::string>("path_vis_topic", path_vis_topic);            //读取可视化路网名
    this->declare_parameter<std::string>("vehicle_imu_topic", vehicle_imu_topic);      //读取可视化路网名
    this->declare_parameter<double>("target_speed", target_speed);                     //读取目标速度
    this->declare_parameter<double>("goal_tolerance", goalTolerance_);                 //读取目标速度
    this->declare_parameter<double>("speed_P", speed_P);                               //读取PID参数
    this->declare_parameter<double>("speed_I", speed_I);
    this->declare_parameter<double>("speed_D", speed_D);
    this->declare_parameter<double>("control_frequency", controlFrequency_);    //读取控制的频率
    this->declare_parameter<double>("vis_frequency", vis_frequency);            //读取路网显示的频率
    this->declare_parameter<std::string>("frame_id", frame_id);                 //读取全局坐标系名

    this->get_parameter<std::string>("vehicle_odom_topic", vehicle_odom_topic);    //读取车辆定位的topic名
    this->get_parameter<std::string>("vehicle_cmd_topic", vehicle_cmd_topic);      //读取车辆控制的topic名
    this->get_parameter<std::string>("roadmap_path", roadmap_path);                //读取路网文件名
    this->get_parameter<std::string>("path_vis_topic", path_vis_topic);            //读取可视化路网名
    this->get_parameter<std::string>("vehicle_imu_topic", vehicle_imu_topic);      //读取可视化路网名
    this->get_parameter<double>("target_speed", target_speed);                     //读取目标速度
    this->get_parameter<double>("goal_tolerance", goalTolerance_);                 //读取目标速度
    this->get_parameter<double>("speed_P", speed_P);                               //读取PID参数
    this->get_parameter<double>("speed_I", speed_I);
    this->get_parameter<double>("speed_D", speed_D);
    this->get_parameter<double>("control_frequency", controlFrequency_);    //读取控制的频率
    this->get_parameter<double>("vis_frequency", vis_frequency);            //读取路网显示的频率
    this->get_parameter<std::string>("frame_id", frame_id);                 //读取全局坐标系名

    //加载路网文件
    std::cout << "roadmap_path: " << roadmap_path << "  " << target_speed << std::endl;
    loadRoadmap(roadmap_path, target_speed);

    pid_controller_longitudinal = std::make_unique<shenlan::control::PIDController>(speed_P, speed_I, speed_D);
    lqr_controller_lateral = std::make_unique<shenlan::control::LqrController>();

    lqr_controller_lateral->LoadControlConf();
    lqr_controller_lateral->Init();

    localization_data_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(vehicle_odom_topic, 10, std::bind(&LQRControllerNode::OdomCallback, this, _1));
    lacalization_data_imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(vehicle_imu_topic, 10, std::bind(&LQRControllerNode::IMUCallback, this, _1));

    vehicle_control_publisher = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>(vehicle_cmd_topic, 1000);
    global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/global_reference_path", 2);
    history_path_visualization_publisher = this->create_publisher<nav_msgs::msg::Path>("/history_path", 2);

    // Initialize the transform broadcaster
    tf_broadcaster_gps_vehicle = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    vehicle_control_iteration_timer = this->create_wall_timer(10ms, std::bind(&LQRControllerNode::VehicleControllerIterationCallback, this));
    global_path_publish_timer = this->create_wall_timer(500ms, std::bind(&LQRControllerNode::GlobalPathPublushCallback, this));

    goalPoint_ = planningPublishedTrajectory_.trajectory_points.back();    //确定目标点

    RCLCPP_INFO(LOGGER, "lqr_control_node init finish!");
}

LQRControllerNode::~LQRControllerNode() {}

void LQRControllerNode::OdomCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
    // cout << "position.x: " << msg->pose.pose.position.x << " " << "position.y: " << msg->pose.pose.position.y << endl;
    if (firstRecord_) {
        vehicleState_.planning_init_x = msg->pose.pose.position.x;
        vehicleState_.planning_init_y = msg->pose.pose.position.y;
        firstRecord_ = false;
    }
    vehicleState_.x = msg->pose.pose.position.x;    //
    vehicleState_.y = msg->pose.pose.position.y;

    // 将orientation(四元数)转换为欧拉角(roll, pitch, yaw)
    tf2::Quaternion quat_tf;
    tf2::convert(msg->pose.pose.orientation, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(vehicleState_.roll, vehicleState_.pitch, vehicleState_.yaw);

    vehicleState_.heading = vehicleState_.yaw;    // pose.orientation是四元数
    // cout << "vehicleState_.heading: " << vehicleState_.heading << endl;

    vehicleState_.velocity = std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x + msg->twist.twist.linear.y * msg->twist.twist.linear.y);                // 速度
    // vehicleState_.angular_velocity = std::sqrt(msg->twist.twist.angular.x * msg->twist.twist.angular.x + msg->twist.twist.angular.y * msg->twist.twist.angular.y);    // 转角速度
    // vehicleState_.acceleration = 0.0;   
    /* 将收到的定位信息发布出来,在rviz里显示历史轨迹 */
    history_path.header.stamp = this->get_clock()->now();
    history_path.header.frame_id = "gps";

    history_path_points.header.stamp = this->get_clock()->now();
    history_path_points.header.frame_id = "gps";
    history_path_points.pose.position.x = vehicleState_.x;
    history_path_points.pose.position.y = vehicleState_.y;
    history_path_points.pose.position.z = 0;
    history_path_points.pose.orientation = msg->pose.pose.orientation;
    history_path.poses.push_back(history_path_points);

    if (history_path.poses.size() > 2000)
    {
        vector<geometry_msgs::msg::PoseStamped>::iterator k = history_path.poses.begin();
        history_path.poses.erase(k);
    }

    history_path_visualization_publisher->publish(history_path);

    // 将世界坐标系和车辆坐标系的位置关系广播出来
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "gps";
    transformStamped.child_frame_id = "vehicle_odometry";
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;

    transformStamped.transform.rotation.x = quat_tf.x();
    transformStamped.transform.rotation.y = quat_tf.y();
    transformStamped.transform.rotation.z = quat_tf.z();
    transformStamped.transform.rotation.w = quat_tf.w();

    tf_broadcaster_gps_vehicle->sendTransform(transformStamped);                                                                                                                              // 加速度
}
void LQRControllerNode::IMUCallback(sensor_msgs::msg::Imu::SharedPtr msg) {
    RCLCPP_INFO(LOGGER, "Got IMU data!!!");
    vehicleState_.angular_velocity = msg->angular_velocity.z;                                                                                                // 平面角速度(绕z轴转动的角速度)
    vehicleState_.acceleration = sqrt(msg->linear_acceleration.x * msg->linear_acceleration.x + msg->linear_acceleration.y * msg->linear_acceleration.y);    // 加速度
}
void LQRControllerNode::loadRoadmap(const std::string& roadmap_path, const double target_speed) {
    // 读取参考线路径
    std::ifstream infile;
    infile.open(roadmap_path);    //将文件流对象与文件连接起来
    assert(infile.is_open());
    std::vector<std::pair<double, double>> xy_points;
    std::string s, x, y;
    while (getline(infile, s)) {
        std::stringstream word(s);
        word >> x;
        word >> y;
        double pt_x = std::atof(x.c_str());
        double pt_y = std::atof(y.c_str());
        xy_points.push_back(std::make_pair(pt_x, pt_y));
    }
    infile.close();
    // Construct the reference_line path profile
    using namespace shenlan::control;
    std::vector<double> headings, accumulated_s, kappas, dkappas;
    //根据离散的点组成的路径，生成路网航向角,累计距离，曲率，曲率的导数
    std::unique_ptr<ReferenceLine> reference_line = std::make_unique<ReferenceLine>(xy_points);
    reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas, &dkappas);

    for (size_t i = 0; i < headings.size(); i++) {
        TrajectoryPoint trajectory_pt;
        trajectory_pt.x = xy_points[i].first;
        trajectory_pt.y = xy_points[i].second;
        trajectory_pt.v = target_speed;
        trajectory_pt.a = 0.0;
        trajectory_pt.heading = headings[i];
        trajectory_pt.kappa = kappas[i];
        planningPublishedTrajectory_.trajectory_points.push_back(trajectory_pt);

        this_pose_stamped.header.frame_id = "gps";
        this_pose_stamped.header.stamp = this->get_clock()->now();
        this_pose_stamped.pose.position.x = xy_points[i].first;
        this_pose_stamped.pose.position.y = xy_points[i].second;
        this_pose_stamped.pose.position.z = 0;
        this_pose_stamped.pose.orientation.x = 0;
        this_pose_stamped.pose.orientation.y = 0;
        this_pose_stamped.pose.orientation.z = 0;
        this_pose_stamped.pose.orientation.w = 0;    // 这里实际上是放的frenet坐标系的S

        global_path.poses.push_back(this_pose_stamped);
        global_path.header.frame_id = "gps";
    }
}

void LQRControllerNode::GlobalPathPublushCallback() {
    global_path.header.stamp = this->get_clock()->now();
    global_path_publisher_->publish(global_path);
}

void LQRControllerNode::VehicleControllerIterationCallback() {
    ControlCmd cmd;
    if (!firstRecord_) {    //有定位数据开始控制
        //小于容忍距离，车辆速度设置为0
        if (pointDistance(goalPoint_, vehicleState_.x, vehicleState_.y) < goalTolerance_) {
            targetSpeed_ = 0;
            isReachGoal_ = true;
        }
        if (!isReachGoal_) {
            lqr_controller_lateral->ComputeControlCommand(this->vehicleState_, this->planningPublishedTrajectory_, cmd);
        }

        lgsvl_msgs::msg::VehicleControlData control_cmd;

        double ego_speed = vehicleState_.velocity;
        double v_err = targetSpeed_ - ego_speed;    // 速度误差
        cout << "v_err: " << v_err << endl;
        double acceleration_cmd = pid_controller_longitudinal->Control(v_err, 0.01);

        control_cmd.header.stamp = this->get_clock()->now();
        control_cmd.acceleration_pct = acceleration_cmd;
        control_cmd.target_gear = lgsvl_msgs::msg::VehicleControlData::GEAR_DRIVE;
        control_cmd.target_wheel_angle = cmd.steer_target;

        vehicle_control_publisher->publish(control_cmd);
    }
}
int main(int argc, char** argv) {
    RCLCPP_INFO(LOGGER, "Initializa Node~");
    std::cout << argv[0] << std::endl;
    rclcpp::init(argc, argv);
    auto n = std::make_shared<LQRControllerNode>();
    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}