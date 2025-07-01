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
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&WaypointNavNode::cmdVelCallback, this, std::placeholders::_1));

    path_pub_pf_ = this->create_publisher<nav_msgs::msg::Path>("pf_path", 10);
    path_pub_kf_ = this->create_publisher<nav_msgs::msg::Path>("kf_path", 10);
    path_pub_ekf_ = this->create_publisher<nav_msgs::msg::Path>("ekf_path", 10);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("filter_path_map", 10);

    map_.header.frame_id = "map";
    map_.info.resolution = 0.05;
    map_.info.width = 500;
    map_.info.height = 500;
    map_.info.origin.position.x = -12.5;
    map_.info.origin.position.y = -12.5;
    map_.data.resize(map_.info.width * map_.info.height, 0);

    kf_P_ = ekf_P_ = Eigen::Matrix3d::Identity();
    kf_state_ = ekf_state_ = Eigen::Vector3d(0, 0, 0);

    for (int i = 0; i < N_; ++i) {
        particles_.push_back({Eigen::Vector3d(0, 0, 0), 1.0 / N_});
    }

    timer_ = this->create_wall_timer(100ms, [this]() {
        if (!got_cmd_) return;
        double dt = (this->now() - last_cmd_time_).seconds();
        double z = simulateRaycast(kf_state_);

        predictPF(last_v_, last_omega_, dt);
        updatePF(z);
        Eigen::Vector3d pf_est = Eigen::Vector3d::Zero();
        for (const auto& p : particles_) pf_est += p.state;
        pf_est /= N_;
        publishPose(pf_est, path_pf_, path_pub_pf_, file_pf_, 200);

        predictKF(last_v_, last_omega_, dt);
        updateKF(z);
        publishPose(kf_state_, path_kf_, path_pub_kf_, file_kf_, 100);

        predictEKF(last_v_, last_omega_, dt);
        updateEKF(z);
        publishPose(ekf_state_, path_ekf_, path_pub_ekf_, file_ekf_, 150);

        map_.header.stamp = this->now();
        map_pub_->publish(map_);
        last_cmd_time_ = this->now();
    });
}

void WaypointNavNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_v_ = msg->linear.x;
    last_omega_ = msg->angular.z;
    got_cmd_ = true;
    last_cmd_time_ = this->now();
}

double WaypointNavNode::simulateRaycast(const Eigen::Vector3d& state) {
    double x = state(0);
    double y = state(1);
    double theta = state(2);
    double max_range = 5.0;
    double step = map_.info.resolution;

    for (double r = 0.0; r <= max_range; r += step) {
        double rx = x + r * std::cos(theta);
        double ry = y + r * std::sin(theta);

        int mx = static_cast<int>((rx - map_.info.origin.position.x) / map_.info.resolution);
        int my = static_cast<int>((ry - map_.info.origin.position.y) / map_.info.resolution);

        if (mx < 0 || my < 0 || mx >= static_cast<int>(map_.info.width) || my >= static_cast<int>(map_.info.height))
            return r;

        if (map_.data[my * map_.info.width + mx] > 50)
            return r;
    }
    return max_range;
}

void WaypointNavNode::predictPF(double v, double omega, double dt) {
    for (auto& p : particles_) {
        double theta = p.state(2);
        double noisy_v = v + motion_noise_(gen_);
        double noisy_omega = omega + motion_noise_(gen_) * 0.1;
        p.state(0) += noisy_v * dt * std::cos(theta);
        p.state(1) += noisy_v * dt * std::sin(theta);
        p.state(2) += noisy_omega * dt;
    }
}

void WaypointNavNode::updatePF(double z) {
    for (auto& p : particles_) {
        double expected_z = simulateRaycast(p.state);
        double error = z - expected_z;
        p.weight = std::exp(-0.5 * error * error / (0.1 * 0.1));
    }
    double sum_weights = 0.0;
    for (const auto& p : particles_) sum_weights += p.weight;
    for (auto& p : particles_) p.weight /= (sum_weights + 1e-6);

    std::vector<Particle> new_particles;
    std::vector<double> weights;
    for (const auto& p : particles_) weights.push_back(p.weight);
    std::discrete_distribution<int> resample(weights.begin(), weights.end());
    for (int i = 0; i < N_; ++i) new_particles.push_back(particles_[resample(gen_)]);
    particles_ = new_particles;
}

void WaypointNavNode::predictKF(double v, double omega, double dt) {
    double theta = kf_state_(2);
    Eigen::Vector3d u(v * dt * std::cos(theta), v * dt * std::sin(theta), omega * dt);
    kf_state_ += u;
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0,2) = -v * dt * std::sin(theta);
    A(1,2) =  v * dt * std::cos(theta);
    Eigen::Matrix3d Q = 0.01 * Eigen::Matrix3d::Identity();
    kf_P_ = A * kf_P_ * A.transpose() + Q;
}

void WaypointNavNode::updateKF(double z) {
    double H = 1.0, R = 0.1;
    double y = z - H * kf_state_(0);
    double S = H * kf_P_(0,0) * H + R;
    double K = kf_P_(0,0) * H / S;
    kf_state_(0) += K * y;
    kf_P_(0,0) = (1 - K * H) * kf_P_(0,0);
}

void WaypointNavNode::predictEKF(double v, double omega, double dt) {
    double theta = ekf_state_(2);
    Eigen::Vector3d u(v * dt * std::cos(theta), v * dt * std::sin(theta), omega * dt);
    ekf_state_ += u;
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0,2) = -v * dt * std::sin(theta);
    A(1,2) =  v * dt * std::cos(theta);
    Eigen::Matrix3d Q = 0.01 * Eigen::Matrix3d::Identity();
    ekf_P_ = A * ekf_P_ * A.transpose() + Q;
}

void WaypointNavNode::updateEKF(double z) {
    double H = 1.0, R = 0.1;
    double y = z - H * ekf_state_(0);
    double S = H * ekf_P_(0,0) * H + R;
    double K = ekf_P_(0,0) * H / S;
    ekf_state_(0) += K * y;
    ekf_P_(0,0) = (1 - K * H) * ekf_P_(0,0);
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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



