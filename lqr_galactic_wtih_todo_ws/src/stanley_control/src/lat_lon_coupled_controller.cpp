#include "stanely_control/lat_lon_coupled_controller.h"

using namespace std;
using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("lat_lon_coupled_controller");

StanelyAndPIDController::StanelyAndPIDController() : Node("vehicle_control")
{
    first_record_ = false;
    V_set_ = 5.0; // m/s
    wheelbase_ = 2.852;
    car_length_ = 2.852;

    pid_controller_longitudinal = std::make_unique<shenlan::control::PIDController>(0.5, 0.0, 0.0);
    stanely_controller_lateral = std::make_unique<shenlan::control::StanleyController>();
    stanely_controller_lateral->LoadControlConf();

    vehicle_control_publisher = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("/vehicle_cmd", 1000);

    global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/global_reference_path", 2);

    history_path_visualization_publisher = this->create_publisher<nav_msgs::msg::Path>("/history_path", 2);

    localization_data_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&StanelyAndPIDController::OdomCallback, this, _1));

    // Initialize the transform broadcaster
    tf_broadcaster_gps_vehicle = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    vehicle_control_iteration_timer = this->create_wall_timer(10ms, std::bind(&StanelyAndPIDController::VehicleControllerIterationCallback, this));
    global_path_publish_timer = this->create_wall_timer(500ms, std::bind(&StanelyAndPIDController::GlobalPathPublushCallback, this)); 

    // Read the reference_line txt
    std::ifstream infile;
    infile.open("src/stanley_control/data/cube_town_reference_line.txt");  //将文件流对象与文件连接起来
    assert(infile.is_open());  //若失败,则输出错误消息,并终止程序运行

    std::vector<std::pair<double, double>> xy_points;
    std::string s;
    std::string x;
    std::string y;
    while (getline(infile, s)) 
    {
        std::stringstream word(s);
        word >> x;
        word >> y;
        double pt_x = std::atof(x.c_str());
        double pt_y = std::atof(y.c_str());
        xy_points.push_back(std::make_pair(pt_x, pt_y));
    }
    infile.close();

    // Construct the reference_line path profile
    std::vector<double> headings;
    std::vector<double> accumulated_s;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::unique_ptr<shenlan::control::ReferenceLine> reference_line = std::make_unique<shenlan::control::ReferenceLine>(xy_points);
    reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas, &dkappas);

    for (size_t i = 0; i < headings.size(); i++) 
    {
        std::cout << "pt " << i << " heading: " << headings[i] << " acc_s: " << accumulated_s[i] << " kappa: " << kappas[i] << " dkappas: " << dkappas[i] << std::endl; // heading 是弧度单位

    }

    for (size_t i = 0; i < headings.size(); i++) 
    {
        TrajectoryPoint trajectory_pt;
        trajectory_pt.x = xy_points[i].first;
        trajectory_pt.y = xy_points[i].second;
        trajectory_pt.v = 2.0;
        trajectory_pt.a = 0.0;
        trajectory_pt.heading = headings[i];
        trajectory_pt.kappa = kappas[i];

        planning_published_trajectory.trajectory_points.push_back(trajectory_pt);

        this_pose_stamped.header.frame_id = "gps";             
        this_pose_stamped.header.stamp = this->get_clock()->now();
        this_pose_stamped.pose.position.x = xy_points[i].first;
        this_pose_stamped.pose.position.y = xy_points[i].second;
        this_pose_stamped.pose.position.z = 0;
        this_pose_stamped.pose.orientation.x = 0;
        this_pose_stamped.pose.orientation.y = 0;
        this_pose_stamped.pose.orientation.z = 0;
        this_pose_stamped.pose.orientation.w = 0; // 这里实际上是放的frenet坐标系的S
        
        global_path.poses.push_back(this_pose_stamped);
        global_path.header.frame_id = "gps";

    }

    goal_point = planning_published_trajectory.trajectory_points.back();

}
StanelyAndPIDController::~StanelyAndPIDController(){}

double StanelyAndPIDController::PointDistance(const TrajectoryPoint& point, const double x, const double y)
{
    const double dx = point.x - x;
    const double dy = point.y - y;
    return sqrt(dx * dx + dy * dy);
}

void StanelyAndPIDController::OdomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!first_record_)
    {
        first_record_ = true;
    }

    RCLCPP_INFO(LOGGER, "I hear localization");

    vehicle_state_.vx = msg->twist.twist.linear.x;
    vehicle_state_.vy = msg->twist.twist.linear.y; 
    
    tf2::Quaternion quat_tf;
    tf2::convert(msg->pose.pose.orientation, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(vehicle_state_.roll, vehicle_state_.pitch, vehicle_state_.yaw);

    vehicle_state_.heading = vehicle_state_.yaw;

    // 将位置转移到前车轮的中心点
    vehicle_state_.x = msg->pose.pose.position.x + std::cos(vehicle_state_.heading) * 0.5 * car_length_;
    vehicle_state_.y = msg->pose.pose.position.y + std::sin(vehicle_state_.heading) * 0.5 * wheelbase_;

    vehicle_state_.velocity = std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x + msg->twist.twist.linear.y * msg->twist.twist.linear.y);
    vehicle_state_.angular_velocity = std::sqrt(msg->twist.twist.angular.x * msg->twist.twist.angular.x + msg->twist.twist.angular.y * msg->twist.twist.angular.y);
    vehicle_state_.acceleration = 0.0;

    /* 将收到的定位信息发布出来,在rviz里显示历史轨迹 */
    history_path.header.stamp = this->get_clock()->now();
    history_path.header.frame_id = "gps";

    history_path_points.header.stamp = this->get_clock()->now();
    history_path_points.header.frame_id = "gps";
    history_path_points.pose.position.x = vehicle_state_.x;
    history_path_points.pose.position.y = vehicle_state_.y;
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

    tf_broadcaster_gps_vehicle->sendTransform(transformStamped);
}

void StanelyAndPIDController::VehicleControllerIterationCallback()
{
    if(this->first_record_)
    {
        if (this->PointDistance(this->goal_point, this->vehicle_state_.x, this->vehicle_state_.y) < 0.5)
        {
            V_set_ = 0;
        }
        stanely_controller_lateral->ComputeControlCmd(this->vehicle_state_, this->planning_published_trajectory, cmd);

        double ego_speed = std::sqrt(this->vehicle_state_.vx * this->vehicle_state_.vx + this->vehicle_state_.vy * this->vehicle_state_.vy);
        double v_err = V_set_ - ego_speed;  // 速度误差
        cout << "v_err: " << v_err << endl;
        double acceleration_cmd = pid_controller_longitudinal->Control(v_err, 0.01);
        control_cmd.header.stamp = this->get_clock()->now();

        control_cmd.acceleration_pct = acceleration_cmd;
        control_cmd.target_gear = lgsvl_msgs::msg::VehicleControlData::GEAR_DRIVE;
        control_cmd.target_wheel_angle = cmd.steer_target;

        vehicle_control_publisher->publish(control_cmd);

    }
    
}
void StanelyAndPIDController::GlobalPathPublushCallback()
{   
    global_path.header.stamp = this->get_clock()->now();
    global_path_publisher_->publish(global_path);
} 

int main(int argc, char **argv)
{
    RCLCPP_INFO(LOGGER, "Initializa Node~");
    rclcpp::init(argc, argv);

    auto n = std::make_shared<StanelyAndPIDController>();

    rclcpp::spin(n);

    rclcpp::shutdown();

    return 0;

}