#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <random>
#include <algorithm>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

struct Particle {
    Eigen::Vector3d state;
    double weight;
};

class WaypointNavNode : public rclcpp::Node {
public:
    WaypointNavNode();

private:
    void sendNextGoal();

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    bool loadWaypointsFromYAML(const std::string& filepath);
    void publishPose(const Eigen::Vector3d& state, nav_msgs::msg::Path& path,
                     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
                     std::ofstream& file, int color);
    void saveMapImage();

    void predictKF(double v, double dt);
    void updateKF(double z);
    void predictEKF(double v, double dt);
    void updateEKF(double z);
    void predictPF(double v, double dt);
    void updatePF(double z);

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_goal_idx_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_pf_, path_pub_kf_, path_pub_ekf_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    nav_msgs::msg::Path path_pf_, path_kf_, path_ekf_;
    nav_msgs::msg::OccupancyGrid map_;
    std::ofstream file_pf_, file_kf_, file_ekf_;

    std::vector<Particle> particles_;
    const int N_ = 100;

    Eigen::Vector3d kf_state_, ekf_state_, last_odom_pose_, prev_odom_pose_;
    Eigen::Matrix3d kf_P_, ekf_P_;
    bool got_odom_ = false;

    std::default_random_engine gen_;
    std::normal_distribution<double> motion_noise_{0.0, 0.02};
    std::normal_distribution<double> sensor_noise_{0.0, 0.1};
};

// IMPLEMENTATION

WaypointNavNode::WaypointNavNode() : Node("waypoint_nav_node"), current_goal_idx_(0) {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    std::string pkg_path = ament_index_cpp::get_package_share_directory("multi_filter_pkg");
    std::string yaml_path = pkg_path + "/config/waypoints.yaml";
    if (!loadWaypointsFromYAML(yaml_path)) {
        RCLCPP_ERROR(this->get_logger(), "Fehler beim Laden der Waypoints.");
        rclcpp::shutdown();
    }
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&WaypointNavNode::scanCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&WaypointNavNode::odomCallback, this, std::placeholders::_1));
    path_pub_pf_ = this->create_publisher<nav_msgs::msg::Path>("pf_path", 10);
    path_pub_kf_ = this->create_publisher<nav_msgs::msg::Path>("kf_path", 10);
    path_pub_ekf_ = this->create_publisher<nav_msgs::msg::Path>("ekf_path", 10);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("filter_path_map", 10);
    path_pf_.header.frame_id = path_kf_.header.frame_id = path_ekf_.header.frame_id = "map";
    file_pf_.open("pf_path.csv");
    file_kf_.open("kf_path.csv");
    file_ekf_.open("ekf_path.csv");
    map_.header.frame_id = "map";
    map_.info.resolution = 0.05;
    map_.info.width = 500;
    map_.info.height = 500;
    map_.info.origin.position.x = -12.5;
    map_.info.origin.position.y = -12.5;
    map_.data.resize(map_.info.width * map_.info.height, 0);
    particles_.resize(N_);
    for (auto& p : particles_) {
        p.state = Eigen::Vector3d::Zero();
        p.weight = 1.0 / N_;
    }
    kf_state_ = ekf_state_ = last_odom_pose_ = prev_odom_pose_ = Eigen::Vector3d::Zero();
    kf_P_ = ekf_P_ = Eigen::Matrix3d::Identity();
    timer_ = this->create_wall_timer(1s, std::bind(&WaypointNavNode::sendNextGoal, this));
}

void WaypointNavNode::sendNextGoal() {
    if (!client_->wait_for_action_server(5s)) {
        RCLCPP_WARN(this->get_logger(), "Warte auf Nav2 Action-Server...");
        return;
    }
    if (current_goal_idx_ >= waypoints_.size()) {
        RCLCPP_INFO(this->get_logger(), "Alle Ziele erreicht.");
        saveMapImage();
        rclcpp::shutdown();
        return;
    }
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = waypoints_[current_goal_idx_];
    goal_msg.behavior_tree = "";
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [](auto) {};
    options.feedback_callback = [](auto, auto) {};
    options.result_callback = [this](auto result) {
        RCLCPP_INFO(this->get_logger(), "Ziel %ld erreicht.", current_goal_idx_);
        current_goal_idx_++;
    };
    client_->async_send_goal(goal_msg, options);
}

void WaypointNavNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!got_odom_) return;
    double dt = 0.1;
    double dx = last_odom_pose_(0) - prev_odom_pose_(0);
    double dy = last_odom_pose_(1) - prev_odom_pose_(1);
    double dtheta = last_odom_pose_(2) - prev_odom_pose_(2);
    double v = std::sqrt(dx*dx + dy*dy) / dt;
    double omega = dtheta / dt;

    prev_odom_pose_ = last_odom_pose_;
    double z = scan->ranges[scan->ranges.size() / 2];

    predictPF(v, omega, dt);
    updatePF(z);
    Eigen::Vector3d pf_est = Eigen::Vector3d::Zero();
    for (const auto& p : particles_) pf_est += p.state;
    pf_est /= N_;
    publishPose(pf_est, path_pf_, path_pub_pf_, file_pf_, 200);

    predictKF(v, omega, dt);
    updateKF(z);
    publishPose(kf_state_, path_kf_, path_pub_kf_, file_kf_, 100);

    predictEKF(v, omega, dt);
    updateEKF(z);
    publishPose(ekf_state_, path_ekf_, path_pub_ekf_, file_ekf_, 150);

    map_.header.stamp = this->now();
    map_pub_->publish(map_);
}


void WaypointNavNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_odom_pose_(0) = msg->pose.pose.position.x;
    last_odom_pose_(1) = msg->pose.pose.position.y;
    last_odom_pose_(2) = tf2::getYaw(msg->pose.pose.orientation);
    if (!got_odom_) {
        prev_odom_pose_ = last_odom_pose_;
        got_odom_ = true;
    }
}

bool WaypointNavNode::loadWaypointsFromYAML(const std::string& filepath) {
    try {
        YAML::Node config = YAML::LoadFile(filepath);
        for (const auto& wp : config["waypoints"]) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = wp["x"].as<double>();
            pose.pose.position.y = wp["y"].as<double>();
            pose.pose.orientation.w = 1.0;
            waypoints_.push_back(pose);
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "YAML Ladefehler: %s", e.what());
        return false;
    }
}

void WaypointNavNode::publishPose(const Eigen::Vector3d& state, nav_msgs::msg::Path& path,
                                  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
                                  std::ofstream& file, int color) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = state(0);
    pose.pose.position.y = state(1);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
    path.header.stamp = this->now();
    pub->publish(path);
    file << state(0) << "," << state(1) << "\n";
    int x = static_cast<int>((state(0) - map_.info.origin.position.x) / map_.info.resolution);
    int y = static_cast<int>((state(1) - map_.info.origin.position.y) / map_.info.resolution);
    if (x >= 0 && y >= 0 && x < static_cast<int>(map_.info.width) && y < static_cast<int>(map_.info.height)) {
        map_.data[y * map_.info.width + x] = color;
    }
}

void WaypointNavNode::saveMapImage() {
    cv::Mat image(map_.info.height, map_.info.width, CV_8UC1);
    for (size_t i = 0; i < map_.data.size(); ++i)
        image.data[i] = static_cast<uint8_t>(map_.data[i]);
    cv::imwrite("filter_paths.png", image);
    RCLCPP_INFO(this->get_logger(), "Pfadbild gespeichert als filter_paths.png");
}
void WaypointNavNode::predictPF(double v, double omega, double dt) {
    for (auto& p : particles_) {
        double theta = p.state(2);
        double noisy_v = v + motion_noise_(gen_);
        double noisy_omega = omega + motion_noise_(gen_) * 0.1;

        double dx = noisy_v * dt * std::cos(theta);
        double dy = noisy_v * dt * std::sin(theta);
        double dtheta = noisy_omega * dt;

        p.state(0) += dx;
        p.state(1) += dy;
        p.state(2) += dtheta;
    }
}


void WaypointNavNode::predictKF(double v, double omega, double dt) {
    double theta = kf_state_(2);

    Eigen::Vector3d u;
    u << v * dt * std::cos(theta),
         v * dt * std::sin(theta),
         omega * dt;

    kf_state_ += u;

    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0,2) = -v * dt * std::sin(theta);
    A(1,2) =  v * dt * std::cos(theta);

    Eigen::Matrix3d Q = 0.01 * Eigen::Matrix3d::Identity();
    kf_P_ = A * kf_P_ * A.transpose() + Q;
}


void WaypointNavNode::predictEKF(double v, double omega, double dt) {
    double theta = ekf_state_(2);

    Eigen::Vector3d u;
    u << v * dt * std::cos(theta),
         v * dt * std::sin(theta),
         omega * dt;

    ekf_state_ += u;

    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0,2) = -v * dt * std::sin(theta);
    A(1,2) =  v * dt * std::cos(theta);

    Eigen::Matrix3d Q = 0.01 * Eigen::Matrix3d::Identity();
    ekf_P_ = A * ekf_P_ * A.transpose() + Q;
}


void WaypointNavNode::updateKF(double z) {
    double H = 1.0;
    double R = 0.1;
    double y = z - H * kf_state_(0);
    double S = H * kf_P_(0,0) * H + R;
    double K = kf_P_(0,0) * H / S;
    kf_state_(0) += K * y;
    kf_P_(0,0) = (1 - K * H) * kf_P_(0,0);
}

void WaypointNavNode::predictEKF(double v, double dt) {
    double theta = ekf_state_(2);
    Eigen::Vector3d u;
    u << v * dt * std::cos(theta),
         v * dt * std::sin(theta),
         0.0;

    ekf_state_ += u;

    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0,2) = -v * dt * std::sin(theta);
    A(1,2) =  v * dt * std::cos(theta);

    Eigen::Matrix3d Q = 0.01 * Eigen::Matrix3d::Identity();
    ekf_P_ = A * ekf_P_ * A.transpose() + Q;
}

void WaypointNavNode::updateEKF(double z) {
    double H = 1.0;
    double R = 0.1;
    double y = z - H * ekf_state_(0);
    double S = H * ekf_P_(0,0) * H + R;
    double K = ekf_P_(0,0) * H / S;
    ekf_state_(0) += K * y;
    ekf_P_(0,0) = (1 - K * H) * ekf_P_(0,0);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



