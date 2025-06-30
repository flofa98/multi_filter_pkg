#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
#include <Eigen/Dense>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class WaypointNavNode : public rclcpp::Node {
public:
    WaypointNavNode() : Node("waypoint_nav_node"), current_goal_idx_(0) {
        client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        std::string pkg_path = ament_index_cpp::get_package_share_directory("multi_filter_pkg");
        std::string yaml_path = pkg_path + "/config/waypoints.yaml";

        if (!loadWaypointsFromYAML(yaml_path)) {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Laden der Waypoints.");
            rclcpp::shutdown();
        }

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WaypointNavNode::scanCallback, this, std::placeholders::_1));

        path_pub_kf_ = this->create_publisher<nav_msgs::msg::Path>("kf_path", 10);
        path_pub_ekf_ = this->create_publisher<nav_msgs::msg::Path>("ekf_path", 10);
        path_pub_pf_ = this->create_publisher<nav_msgs::msg::Path>("pf_path", 10);
        
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("filter_path_map", 10);

        path_kf_.header.frame_id = "map";
        path_ekf_.header.frame_id = "map";
        path_pf_.header.frame_id = "map";

        file_kf_.open("/tmp/kf_path.csv");
        file_ekf_.open("/tmp/ekf_path.csv");
        file_pf_.open("/tmp/pf_path.csv");

        x_kf_ = Eigen::Vector3d::Zero();
        P_kf_ = Eigen::Matrix3d::Identity();
        Q_kf_ = Eigen::Matrix3d::Identity() * 0.01;

        x_ekf_ = Eigen::Vector3d::Zero();
        P_ekf_ = Eigen::Matrix3d::Identity();
        Q_ekf_ = Eigen::Matrix3d::Identity() * 0.01;

        x_pf_ = Eigen::Vector3d::Zero();
        P_pf_ = Eigen::Matrix3d::Identity();
        Q_pf_ = Eigen::Matrix3d::Identity() * 0.01;

        map_.header.frame_id = "map";
        map_.info.resolution = 0.05;
        map_.info.width = 500;
        map_.info.height = 500;
        map_.info.origin.position.x = -12.5;
        map_.info.origin.position.y = -12.5;
        map_.data.resize(map_.info.width * map_.info.height, 0);

        timer_ = this->create_wall_timer(1s, std::bind(&WaypointNavNode::sendNextGoal, this));
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_goal_idx_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_kf_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_ekf_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_pf_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    nav_msgs::msg::Path path_kf_;
    nav_msgs::msg::Path path_ekf_;
    nav_msgs::msg::Path path_pf_;
    nav_msgs::msg::OccupancyGrid map_;

    std::ofstream file_kf_;
    std::ofstream file_ekf_;
    std::ofstream file_pf_;

    Eigen::Vector3d x_kf_, x_ekf_, x_pf_;
    Eigen::Matrix3d P_kf_, Q_kf_;
    Eigen::Matrix3d P_ekf_, Q_ekf_;
    Eigen::Matrix3d P_pf_, Q_pf_;

    bool loadWaypointsFromYAML(const std::string& filepath) {
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

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        double v = 0.2;
        double omega = 0.0;
        double dt = 0.1;

        // KF
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        Eigen::Vector3d u_kf;
        u_kf << v * dt, 0, omega * dt;
        x_kf_ = F * x_kf_ + u_kf;
        P_kf_ = F * P_kf_ * F.transpose() + Q_kf_;
        publishPose(x_kf_, path_kf_, path_pub_kf_, file_kf_, 100);

        // EKF
        Eigen::Vector3d u_ekf;
        u_ekf << v * dt, 0, omega * dt;
        x_ekf_ = F * x_ekf_ + u_ekf;
        P_ekf_ = F * P_ekf_ * F.transpose() + Q_ekf_;
        publishPose(x_ekf_, path_ekf_, path_pub_ekf_, file_ekf_, 150);

        // PF
        Eigen::Vector3d u_pf;
        u_pf << v * dt + 0.01 * ((rand() % 100) / 100.0 - 0.5), 0, omega * dt;
        x_pf_ = F * x_pf_ + u_pf;
        P_pf_ = F * P_pf_ * F.transpose() + Q_pf_;
        publishPose(x_pf_, path_pf_, path_pub_pf_, file_pf_, 200);

        map_.header.stamp = this->now();
        map_pub_->publish(map_);
    }

    void publishPose(const Eigen::Vector3d& x, nav_msgs::msg::Path& path,
                     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
                     std::ofstream& file, int color_val) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = x(0);
        pose.pose.position.y = x(1);
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
        path.header.stamp = this->now();
        pub->publish(path);
        file << x(0) << "," << x(1) << "\n";

        int cx = static_cast<int>((x(0) - map_.info.origin.position.x) / map_.info.resolution);
        int cy = static_cast<int>((x(1) - map_.info.origin.position.y) / map_.info.resolution);
        if (cx >= 0 && cy >= 0 && cx < static_cast<int>(map_.info.width) && cy < static_cast<int>(map_.info.height)) {
            map_.data[cy * map_.info.width + cx] = color_val;
        }
    }

    void sendNextGoal() {
        if (!client_->wait_for_action_server(5s)) {
            RCLCPP_WARN(this->get_logger(), "Warte auf Nav2 Action-Server...");
            return;
        }

        if (current_goal_idx_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "Alle Ziele erreicht.");
            file_kf_.close();
            file_ekf_.close();
            file_pf_.close();
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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

