#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class WaypointNavNode : public rclcpp::Node {
public:
    WaypointNavNode() : Node("waypoint_nav_node") {
        this->client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        std::string pkg_path = ament_index_cpp::get_package_share_directory("multi_filter_pkg");
        std::string yaml_path = pkg_path + "/config/waypoints.yaml";

        if (!loadWaypointsFromYAML(yaml_path)) {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Laden der Waypoints.");
            rclcpp::shutdown();
            return;
        }

        timer_ = this->create_wall_timer(1s, std::bind(&WaypointNavNode::sendNextGoal, this));
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_goal_idx_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;

    bool loadWaypointsFromYAML(const std::string &filename) {
        try {
            YAML::Node config = YAML::LoadFile(filename);
            if (!config["waypoints"]) return false;

            for (const auto &entry : config["waypoints"]) {
                geometry_msgs::msg::PoseStamped pose;
                auto pose_arr = entry.second["pose"];
                auto orient_arr = entry.second["orientation"];

                pose.header.frame_id = "map";
                pose.pose.position.x = pose_arr[0].as<double>();
                pose.pose.position.y = pose_arr[1].as<double>();
                pose.pose.position.z = pose_arr[2].as<double>();

                pose.pose.orientation.x = orient_arr[0].as<double>();
                pose.pose.orientation.y = orient_arr[1].as<double>();
                pose.pose.orientation.z = orient_arr[2].as<double>();
                pose.pose.orientation.w = orient_arr[3].as<double>();

                waypoints_.push_back(pose);
            }
            return true;
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "YAML Fehler: %s", e.what());
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

        RCLCPP_INFO(this->get_logger(), "Sende Ziel %ld...", current_goal_idx_);
        client_->async_send_goal(goal_msg,
            [](auto) {},  // goal_response
            [](auto) {},  // feedback
            [this](auto result) {  // result
                RCLCPP_INFO(this->get_logger(), "Ziel erreicht.");
                current_goal_idx_++;
            });
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNavNode>());
    rclcpp::shutdown();
    return 0;
}

