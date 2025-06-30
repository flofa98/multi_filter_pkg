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

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        if (!got_odom_) return;
        double dt = 0.1;
        double dx = last_odom_pose_(0) - prev_odom_pose_(0);
        double dy = last_odom_pose_(1) - prev_odom_pose_(1);
        double v = std::sqrt(dx*dx + dy*dy) / dt;
        prev_odom_pose_ = last_odom_pose_;

        double z = scan->ranges[scan->ranges.size() / 2];

        predictPF(v, dt);
        updatePF(z);
        Eigen::Vector3d pf_est = Eigen::Vector3d::Zero();
        for (const auto& p : particles_) pf_est += p.state;
        pf_est /= N_;
        publishPose(pf_est, path_pf_, path_pub_pf_, file_pf_, 200);

        predictKF(v, dt);
        updateKF(z);
        publishPose(kf_state_, path_kf_, path_pub_kf_, file_kf_, 100);

        predictEKF(v, dt);
        updateEKF(z);
        publishPose(ekf_state_, path_ekf_, path_pub_ekf_, file_ekf_, 150);

        map_.header.stamp = this->now();
        map_pub_->publish(map_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_pose_(0) = msg->pose.pose.position.x;
        last_odom_pose_(1) = msg->pose.pose.position.y;
        last_odom_pose_(2) = tf2::getYaw(msg->pose.pose.orientation);
        if (!got_odom_) {
            prev_odom_pose_ = last_odom_pose_;
            got_odom_ = true;
        }
    }

    bool loadWaypointsFromYAML(const std::string& filepath);
    void publishPose(const Eigen::Vector3d& state, nav_msgs::msg::Path& path,
                     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
                     std::ofstream& file, int color);
    void saveMapImage();

    void predictKF(double v, double dt) {
        kf_state_(0) += v * dt;
        kf_P_(0, 0) += 0.01;
    }

    void updateKF(double z) {
        double H = 1.0, R = 0.1;
        double y = z - H * kf_state_(0);
        double S = H * kf_P_(0, 0) * H + R;
        double K = kf_P_(0, 0) * H / S;
        kf_state_(0) += K * y;
        kf_P_(0, 0) = (1 - K * H) * kf_P_(0, 0);
    }

    void predictEKF(double v, double dt) {
        ekf_state_(0) += v * dt;
        ekf_P_(0, 0) += 0.01;
    }

    void updateEKF(double z) {
        double H = 1.0, R = 0.1;
        double y = z - H * ekf_state_(0);
        double S = H * ekf_P_(0, 0) * H + R;
        double K = ekf_P_(0, 0) * H / S;
        ekf_state_(0) += K * y;
        ekf_P_(0, 0) = (1 - K * H) * ekf_P_(0, 0);
    }

    void predictPF(double v, double dt) {
        for (auto& p : particles_) {
            p.state(0) += v * dt + motion_noise_(gen_);
        }
    }

    void updatePF(double z) {
        double expected_range, error;
        for (auto& p : particles_) {
            expected_range = p.state(0);
            error = z - expected_range;
            p.weight = std::exp(-0.5 * error * error / 0.1);
        }
        double sum = 0.0;
        for (const auto& p : particles_) sum += p.weight;
        for (auto& p : particles_) p.weight /= sum;

        std::vector<double> weights;
        std::transform(particles_.begin(), particles_.end(), std::back_inserter(weights), [](const Particle& p) {
            return p.weight;
        });
        std::discrete_distribution<int> dist(weights.begin(), weights.end());
        std::vector<Particle> new_particles;
        for (int i = 0; i < N_; ++i) new_particles.push_back(particles_[dist(gen_)]);
        particles_ = new_particles;
    }

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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}





