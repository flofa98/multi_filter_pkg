#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <random>
#include <algorithm>
#include <cmath>

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
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    bool loadWaypointsFromYAML(const std::string& filepath);
    void publishPose(const Eigen::Vector3d& state, nav_msgs::msg::Path& path,
                     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
                     std::ofstream& file, int color);
    void saveMapImage();

    void predictPF(double v, double omega, double dt);
    void predictKF(double v, double omega, double dt);
    void predictEKF(double v, double omega, double dt);
    void updateKF(double z);
    void updateEKF(double z);
    void updatePF(double z);
    double simulateRaycast(const Eigen::Vector3d& state);

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_goal_idx_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_pf_, path_pub_kf_, path_pub_ekf_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    nav_msgs::msg::Path path_pf_, path_kf_, path_ekf_;
    nav_msgs::msg::OccupancyGrid map_;
    std::ofstream file_pf_, file_kf_, file_ekf_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr nav2_plan_sub_;
    nav_msgs::msg::Path nav2_full_path_;
    std::ofstream file_nav2_;


    std::vector<Particle> particles_;
    const int N_ = 100;

    Eigen::Vector3d kf_state_, ekf_state_;
    Eigen::Matrix3d kf_P_, ekf_P_;
    bool got_cmd_ = false;
    rclcpp::Time last_cmd_time_;

    std::default_random_engine gen_;
    std::normal_distribution<double> motion_noise_{0.0, 0.02};
    std::normal_distribution<double> sensor_noise_{0.0, 0.1};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    double last_v_ = 0.0;
    double last_omega_ = 0.0;
};

WaypointNavNode::WaypointNavNode() : Node("waypoint_nav_node"), current_goal_idx_(0) {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    std::string pkg_path = ament_index_cpp::get_package_share_directory("multi_filter_pkg");
    std::string yaml_path = pkg_path + "/config/waypoints.yaml";
    if (!loadWaypointsFromYAML(yaml_path)) {
        RCLCPP_ERROR(this->get_logger(), "Fehler beim Laden der Waypoints.");
        rclcpp::shutdown();
        return;
    }

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&WaypointNavNode::cmdVelCallback, this, std::placeholders::_1));

    path_pub_pf_ = this->create_publisher<nav_msgs::msg::Path>("pf_path", 10);
    path_pub_kf_ = this->create_publisher<nav_msgs::msg::Path>("kf_path", 10);
    path_pub_ekf_ = this->create_publisher<nav_msgs::msg::Path>("ekf_path", 10);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("filter_path_map", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Warte kurz auf tf-Frames
    rclcpp::sleep_for(500ms);

    // Filter mit Groundtruth aus tf initialisieren
    geometry_msgs::msg::TransformStamped transformStamped;
    if (tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(3.0))) {
    transformStamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

    double x = transformStamped.transform.translation.x;
    double y = transformStamped.transform.translation.y;

    tf2::Quaternion q(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    Eigen::Vector3d start_pose(x, y, yaw);
    kf_state_ = ekf_state_ = start_pose;
    for (auto& p : particles_)
        p.state = start_pose;

    RCLCPP_INFO(this->get_logger(), "Filter initialisiert: x=%.2f y=%.2f yaw=%.2f", x, y, yaw);
} else {
    RCLCPP_WARN(this->get_logger(), "map → base_link nicht verfügbar nach 3 Sekunden");
}



    timer_ = this->create_wall_timer(1s, std::bind(&WaypointNavNode::sendNextGoal, this));

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

    kf_P_ = ekf_P_ = Eigen::Matrix3d::Identity();
    particles_.resize(N_);  // Platz reservieren

    nav2_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/plan", 10,
    [this](const nav_msgs::msg::Path::SharedPtr msg) {
        if (!msg->poses.empty()) {
            for (const auto& pose : msg->poses) {
                nav2_full_path_.poses.push_back(pose);
                if (file_nav2_.is_open())
                    file_nav2_ << pose.pose.position.x << "," << pose.pose.position.y << "\n";
            }
        }
    });

    nav2_full_path_.header.frame_id = "map";
    file_nav2_.open("nav2_path.csv");


    }

void WaypointNavNode::sendNextGoal() {
    if (!client_->wait_for_action_server(5s)) return;
    if (current_goal_idx_ >= waypoints_.size()) {
        saveMapImage();
        rclcpp::shutdown();
        return;
    }
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = waypoints_[current_goal_idx_];
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.result_callback = [this](auto result) {
        current_goal_idx_++;
    };
    client_->async_send_goal(goal_msg, options);
}

void WaypointNavNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (got_cmd_) {
        double dt = (this->now() - last_cmd_time_).seconds();
        double v = last_v_;
        double omega = last_omega_;

        predictPF(v, omega, dt);
        updatePF(simulateRaycast(particles_[0].state));
        predictKF(v, omega, dt);
        updateKF(simulateRaycast(kf_state_));
        predictEKF(v, omega, dt);
        updateEKF(simulateRaycast(ekf_state_));

        Eigen::Vector3d pf_est = Eigen::Vector3d::Zero();
        for (const auto& p : particles_) pf_est += p.state;
        pf_est /= N_;

        publishPose(pf_est, path_pf_, path_pub_pf_, file_pf_, 200);
        publishPose(kf_state_, path_kf_, path_pub_kf_, file_kf_, 100);
        publishPose(ekf_state_, path_ekf_, path_pub_ekf_, file_ekf_, 150);

        map_.header.stamp = this->now();
        map_pub_->publish(map_);
    }
    last_cmd_time_ = this->now();
    last_v_ = msg->linear.x;
    last_omega_ = msg->angular.z;
    got_cmd_ = true;
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
    } catch (...) {
        return false;
    }
}

void WaypointNavNode::predictPF(double v, double omega, double dt) {
    for (auto& p : particles_) {
        double theta = p.state(2);
        double dx = (v + motion_noise_(gen_)) * dt * cos(theta);
        double dy = (v + motion_noise_(gen_)) * dt * sin(theta);
        double dtheta = (omega + motion_noise_(gen_) * 0.1) * dt;
        p.state += Eigen::Vector3d(dx, dy, dtheta);
    }
}

void WaypointNavNode::predictKF(double v, double omega, double dt) {
    double theta = kf_state_(2);
    Eigen::Vector3d u(v * dt * cos(theta), v * dt * sin(theta), omega * dt);
    kf_state_ += u;
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0,2) = -v * dt * sin(theta);
    A(1,2) =  v * dt * cos(theta);
    Eigen::Matrix3d Q = 0.01 * Eigen::Matrix3d::Identity();
    kf_P_ = A * kf_P_ * A.transpose() + Q;
}

void WaypointNavNode::predictEKF(double v, double omega, double dt) {
    double theta = ekf_state_(2);
    Eigen::Vector3d u(v * dt * cos(theta), v * dt * sin(theta), omega * dt);
    ekf_state_ += u;
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0,2) = -v * dt * sin(theta);
    A(1,2) =  v * dt * cos(theta);
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
    kf_P_(0,0) *= (1 - K * H);
}

void WaypointNavNode::updateEKF(double z) {
    double H = 1.0;
    double R = 0.1;
    double y = z - H * ekf_state_(0);
    double S = H * ekf_P_(0,0) * H + R;
    double K = ekf_P_(0,0) * H / S;
    ekf_state_(0) += K * y;
    ekf_P_(0,0) *= (1 - K * H);
}

void WaypointNavNode::updatePF(double z) {
    double sum_weights = 0.0;
    for (auto& p : particles_) {
        double error = z - p.state(0);
        p.weight = std::exp(-0.5 * error * error / (0.1 * 0.1));
        sum_weights += p.weight;
    }
    for (auto& p : particles_) p.weight /= (sum_weights + 1e-6);

    // Gewichte extrahieren
    std::vector<double> weights;
    for (const auto& p : particles_) weights.push_back(p.weight);

    std::discrete_distribution<> dist(weights.begin(), weights.end());
    std::vector<Particle> new_particles;
    for (int i = 0; i < N_; ++i)
        new_particles.push_back(particles_[dist(gen_)]);

    particles_ = new_particles;
}


double WaypointNavNode::simulateRaycast(const Eigen::Vector3d& state) {
    return state(0) + sensor_noise_(gen_);
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
    for (const auto& pose : nav2_full_path_.poses) {
    int x = static_cast<int>((pose.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
    int y = static_cast<int>((pose.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);
    if (x >= 0 && y >= 0 && x < static_cast<int>(map_.info.width) && y < static_cast<int>(map_.info.height)) {
        image.at<uchar>(y, x) = 50;  // z. B. dunklerer Grauwert für Nav2
    }
}

    cv::imwrite("filter_paths.png", image);
    RCLCPP_INFO(this->get_logger(), "Pfadbild gespeichert als filter_paths.png");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



