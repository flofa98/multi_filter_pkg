#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class WaypointNavNode : public rclcpp::Node {
public:
    WaypointNavNode() : Node("waypoint_nav_node"), current_goal_idx_(0) {
        client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // Lade Waypoints
        std::string pkg_path = ament_index_cpp::get_package_share_directory("multi_filter_pkg");
        std::string yaml_path = pkg_path + "/config/waypoints.yaml";

        if (!loadWaypointsFromYAML(yaml_path)) {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Laden der Waypoints.");
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(1s, std::bind(&WaypointNavNode::sendNextGoal, this));
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_goal_idx_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool loadWaypointsFromYAML(const std::string& filepath) {
        try {
            YAML::Node config = YAML::LoadFile(filepath);
            for (const auto& wp : config["waypoints"]) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = wp["x"].as<double>();
                pose.pose.position.y = wp["y"].as<double>();
                pose.pose.orientation.w = 1.0;  // default
                waypoints_.push_back(pose);
            }
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML Ladefehler: %s", e.what());
            return false;
        }
    }

    void sendNextGoal() {
        if (!client_->wait_for_action_server(5s)) {
            RCLCPP_WARN(this->get_logger(), "Warte auf Nav2 Action-Server...");
            return;
        }

        if (current_goal_idx_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "Alle Ziele erreicht.");
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
