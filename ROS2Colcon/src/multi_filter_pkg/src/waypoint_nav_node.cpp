#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <chrono>

using namespace std::chrono_literals;

class WaypointNavigator : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WaypointNavigator() : Node("waypoint_nav_node") {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        if (!client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server nicht verfügbar.");
            rclcpp::shutdown();
        }

        // YAML laden
        if (!loadWaypointsFromYAML("waypoints.yaml")) {
            RCLCPP_ERROR(this->get_logger(), "Konnte waypoints.yaml nicht laden.");
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(1s, std::bind(&WaypointNavigator::sendNextGoal, this));
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_index_ = 0;
    bool goal_active_ = false;
    rclcpp::TimerBase::SharedPtr timer_;

    bool loadWaypointsFromYAML(const std::string &filepath) {
        try {
            YAML::Node config = YAML::LoadFile(filepath);
            const auto &wps = config["waypoints"];
            for (const auto &entry : wps) {
                auto pose_array = entry.second["pose"];
                auto orient_array = entry.second["orientation"];
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = pose_array[0].as<double>();
                pose.pose.position.y = pose_array[1].as<double>();
                pose.pose.position.z = pose_array[2].as<double>();
                pose.pose.orientation.x = orient_array[0].as<double>();
                pose.pose.orientation.y = orient_array[1].as<double>();
                pose.pose.orientation.z = orient_array[2].as<double>();
                pose.pose.orientation.w = orient_array[3].as<double>();
                waypoints_.push_back(pose);
            }
            RCLCPP_INFO(this->get_logger(), "%zu Wegpunkte geladen.", waypoints_.size());
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Fehler beim Laden der YAML: %s", e.what());
            return false;
        }
    }

    void sendNextGoal() {
        if (goal_active_ || current_index_ >= waypoints_.size()) return;

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = waypoints_[current_index_];
        goal_msg.pose.header.stamp = this->now();

        RCLCPP_INFO(this->get_logger(), "Sende Ziel %zu: x=%.2f y=%.2f",
                    current_index_,
                    goal_msg.pose.pose.position.x,
                    goal_msg.pose.pose.position.y);

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.result_callback = std::bind(&WaypointNavigator::resultCallback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, options);
        goal_active_ = true;
    }

    void resultCallback(const GoalHandle::WrappedResult &result) {
        goal_active_ = false;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Ziel %zu erreicht.", current_index_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Ziel %zu NICHT erreicht.", current_index_);
        }

        current_index_++;
        if (current_index_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "Alle Wegpunkte abgefahren.");
        }
    }
};
    
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNavigator>());
    rclcpp::shutdown();
    return 0;
}

