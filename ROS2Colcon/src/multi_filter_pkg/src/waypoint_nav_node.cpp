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
#include <opencv2/opencv.hpp>
#include <random>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

struct Particle {
    Eigen::Vector3d state;
    double weight;
};

class WaypointNavNode : public rclcpp::Node {
public:
    WaypointNavNode() : Node("waypoint_nav_node"), current_goal_idx_(0) {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        std::string pkg_path = ament_index_cpp::get_package_share_directory("multi_filter_pkg");
        std::string yaml_path = pkg_path + "/config/waypoints.yaml";

        if (!loadWaypointsFromYAML(yaml_path)) {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Laden der Waypoints.");
            rclcpp::shutdown();
        }

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WaypointNavNode::scanCallback, this, std::placeholders::_1));

        path_pub_pf_ = this->create_publisher<nav_msgs::msg::Path>("pf_path", 10);
        path_pf_.header.frame_id = "map";
        file_pf_.open("pf_path.csv");

        map_.header.frame_id = "map";
        map_.info.resolution = 0.05;
        map_.info.width = 500;
        map_.info.height = 500;
        map_.info.origin.position.x = -12.5;
        map_.info.origin.position.y = -12.5;
        map_.data.resize(map_.info.width * map_.info.height, 0);

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("filter_path_map", 10);

        std::default_random_engine gen;
        std::normal_distribution<double> dist(0.0, 0.05);
        particles_.resize(N_);
        for (auto& p : particles_) {
            p.state = Eigen::Vector3d::Zero();
            p.weight = 1.0 / N_;
        }

        timer_ = this->create_wall_timer(1s, std::bind(&WaypointNavNode::sendNextGoal, this));
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_goal_idx_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_pf_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    nav_msgs::msg::Path path_pf_;
    nav_msgs::msg::OccupancyGrid map_;
    std::ofstream file_pf_;

    std::vector<Particle> particles_;
    const int N_ = 100;

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
        double dt = 0.1;
        double v = 0.2;

        std::default_random_engine gen;
        std::normal_distribution<double> motion_noise(0.0, 0.02);

        for (auto& p : particles_) {
            p.state(0) += v * dt + motion_noise(gen);
            p.state(1) += motion_noise(gen);

            double expected_range = p.state(0);
            double measured_range = scan->ranges[scan->ranges.size() / 2];
            double error = measured_range - expected_range;
            p.weight = std::exp(-0.5 * error * error / 0.1);
        }

        double weight_sum = 0.0;
        for (auto& p : particles_) weight_sum += p.weight;
        for (auto& p : particles_) p.weight /= weight_sum;

        std::discrete_distribution<int> resample_dist(
            particles_.begin(), particles_.end(), [](const Particle& p) { return p.weight; });

        std::vector<Particle> new_particles;
        for (int i = 0; i < N_; ++i) new_particles.push_back(particles_[resample_dist(gen)]);
        particles_ = new_particles;

        Eigen::Vector3d pf_mean = Eigen::Vector3d::Zero();
        for (const auto& p : particles_) pf_mean += p.state;
        pf_mean /= N_;

        publishPose(pf_mean, path_pf_, path_pub_pf_, file_pf_, 200);
        map_.header.stamp = this->now();
        map_pub_->publish(map_);
    }

    void publishPose(const Eigen::Vector3d& state, nav_msgs::msg::Path& path,
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

    void sendNextGoal() {
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

    void saveMapImage() {
        cv::Mat image(map_.info.height, map_.info.width, CV_8UC1);
        for (size_t i = 0; i < map_.data.size(); ++i)
            image.data[i] = static_cast<uint8_t>(map_.data[i]);
        cv::imwrite("filter_paths.png", image);
        RCLCPP_INFO(this->get_logger(), "Pfadbild gespeichert als filter_paths.png");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


