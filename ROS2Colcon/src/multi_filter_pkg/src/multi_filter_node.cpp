#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <random>
#include <fstream>
#include <thread>

using std::placeholders::_1;

class MultiFilterNode : public rclcpp::Node {
public:
    MultiFilterNode() : Node("multi_filter_node"), gen_(rd_()), dist_(-0.05, 0.05) {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MultiFilterNode::odomCallback, this, _1));
        gt_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ground_truth_pose", 10, std::bind(&MultiFilterNode::gtCallback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        kf_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/kf_pose", 10);
        ekf_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf_pose", 10);
        pf_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pf_pose", 10);
        gt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gt_pose", 10);

        kf_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/kf_path", 10);
        ekf_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/ekf_path", 10);
        pf_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/pf_path", 10);

        kf_path_.header.frame_id = "odom";
        ekf_path_.header.frame_id = "odom";
        pf_path_.header.frame_id = "odom";

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&MultiFilterNode::controlLoop, this));

        last_time_ = this->now();

        // Init Filterzustände
        x_kf_ = Eigen::Vector3d::Zero();
        P_kf_ = Eigen::Matrix3d::Identity() * 0.1;
        Q_kf_ = Eigen::Matrix3d::Identity() * 0.01;
        R_kf_ = Eigen::Matrix3d::Identity() * 0.05;

        x_ekf_ = Eigen::Vector3d::Zero();
        P_ekf_ = Eigen::Matrix3d::Identity() * 0.1;
        Q_ekf_ = Eigen::Matrix3d::Identity() * 0.01;
        R_ekf_ = Eigen::Matrix3d::Identity() * 0.05;

        initParticles(100);

        // Kreiswegpunkte (Radius 2m, 8 Punkte)
        int num_points = 8;
        double radius = 2.0;
        for (int i = 0; i < num_points; ++i) {
            double angle = 2.0 * M_PI * i / num_points;
            waypoints_.push_back(Eigen::Vector2d(radius * cos(angle), radius * sin(angle)));
        }
        current_waypoint_idx_ = 0;
    }

    // saveAllPaths jetzt public, damit main() darauf zugreifen kann
    void saveAllPaths() {
        savePathToCSV(kf_path_, "kf_path.csv");
        savePathToCSV(ekf_path_, "ekf_path.csv");
        savePathToCSV(pf_path_, "pf_path.csv");
    }

private:
    // KF
    void kfPredict(double v, double omega, double dt) {
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        Eigen::Vector3d u;
        u << v * dt * cos(x_kf_(2)), v * dt * sin(x_kf_(2)), omega * dt;
        x_kf_ = F * x_kf_ + u;
        P_kf_ = F * P_kf_ * F.transpose() + Q_kf_;
    }
    void kfUpdate(const Eigen::Vector3d& z) {
        Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
        Eigen::Vector3d y = z - H * x_kf_;
        Eigen::Matrix3d S = H * P_kf_ * H.transpose() + R_kf_;
        Eigen::Matrix3d K = P_kf_ * H.transpose() * S.inverse();
        x_kf_ = x_kf_ + K * y;
        P_kf_ = (Eigen::Matrix3d::Identity() - K * H) * P_kf_;
    }

    // EKF
    void ekfPredict(double v, double omega, double dt) {
        double theta = x_ekf_(2);
        Eigen::Vector3d u;
        u << v * dt * cos(theta), v * dt * sin(theta), omega * dt;
        x_ekf_ = x_ekf_ + u;
        Eigen::Matrix3d F;
        F << 1, 0, -v * dt * sin(theta),
             0, 1,  v * dt * cos(theta),
             0, 0, 1;
        P_ekf_ = F * P_ekf_ * F.transpose() + Q_ekf_;
    }
    void ekfUpdate(const Eigen::Vector3d& z) {
        Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
        Eigen::Vector3d y = z - H * x_ekf_;
        Eigen::Matrix3d S = H * P_ekf_ * H.transpose() + R_ekf_;
        Eigen::Matrix3d K = P_ekf_ * H.transpose() * S.inverse();
        x_ekf_ = x_ekf_ + K * y;
        P_ekf_ = (Eigen::Matrix3d::Identity() - K * H) * P_ekf_;
    }

    // Particle Filter
    struct Particle {
        double x, y, theta, weight;
    };
    void initParticles(int n) {
        particles_.resize(n);
        for (auto& p : particles_) {
            p.x = 0.0; p.y = 0.0; p.theta = 0.0; p.weight = 1.0 / n;
        }
    }
    void pfPredict(double v, double omega, double dt) {
        for (auto& p : particles_) {
            double dx = v * dt * cos(p.theta) + dist_(gen_);
            double dy = v * dt * sin(p.theta) + dist_(gen_);
            double dtheta = omega * dt + dist_(gen_);
            p.x += dx; p.y += dy; p.theta += dtheta;
        }
    }
    void pfUpdate(const Eigen::Vector3d& z) {
        double weight_sum = 0.0;
        for (auto& p : particles_) {
            double dx = z(0) - p.x;
            double dy = z(1) - p.y;
            double dtheta = z(2) - p.theta;
            double dist = dx*dx + dy*dy + dtheta*dtheta;
            p.weight = std::exp(-dist * 10.0);
            weight_sum += p.weight;
        }
        for (auto& p : particles_) {
            p.weight /= weight_sum;
        }
        resampleParticles();
    }
    void resampleParticles() {
        std::vector<Particle> new_particles;
        new_particles.reserve(particles_.size());
        std::vector<double> cumulative_weights(particles_.size());
        cumulative_weights[0] = particles_[0].weight;
        for (size_t i=1; i<particles_.size(); ++i) {
            cumulative_weights[i] = cumulative_weights[i-1] + particles_[i].weight;
        }
        std::uniform_real_distribution<double> dist(0.0,1.0);
        double step = 1.0 / particles_.size();
        double r = dist(gen_) * step;
        int index = 0;
        for (size_t i=0; i<particles_.size(); ++i) {
            double u = r + i * step;
            while (u > cumulative_weights[index]) index++;
            new_particles.push_back(particles_[index]);
            new_particles.back().weight = 1.0 / particles_.size();
        }
        particles_ = new_particles;
    }
    Eigen::Vector3d pfEstimate() {
        double x=0, y=0, sin_sum=0, cos_sum=0;
        for (auto& p : particles_) {
            x += p.x * p.weight;
            y += p.y * p.weight;
            sin_sum += sin(p.theta) * p.weight;
            cos_sum += cos(p.theta) * p.weight;
        }
        double theta = atan2(sin_sum, cos_sum);
        return Eigen::Vector3d(x,y,theta);
    }

    void controlLoop() {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0) dt = 0.1;
        last_time_ = now;

        if (waypoints_.empty()) return;

        double robot_x = ground_truth_pose_.pose.pose.position.x;
        double robot_y = ground_truth_pose_.pose.pose.position.y;
        double robot_theta = getYawFromQuaternion(ground_truth_pose_.pose.pose.orientation);

        Eigen::Vector2d target = waypoints_[current_waypoint_idx_];
        double dx = target.x() - robot_x;
        double dy = target.y() - robot_y;
        double distance = std::sqrt(dx*dx + dy*dy);

        double target_angle = std::atan2(dy, dx);
        double angle_diff = normalizeAngle(target_angle - robot_theta);

        geometry_msgs::msg::Twist cmd;
        const double angle_threshold = 0.1;
        const double dist_threshold = 0.1;
        const double max_speed = 0.2;
        const double max_turn = 0.5;

        if (distance < dist_threshold) {
            current_waypoint_idx_ = (current_waypoint_idx_ + 1) % waypoints_.size();
        } else {
            if (std::abs(angle_diff) > angle_threshold) {
                cmd.angular.z = (angle_diff > 0) ? max_turn : -max_turn;
                cmd.linear.x = 0.0;
            } else {
                cmd.angular.z = 0.0;
                cmd.linear.x = max_speed;
            }
        }
        cmd_pub_->publish(cmd);

        if (odom_.header.stamp.sec != 0) {
            double v = odom_.twist.twist.linear.x;
            double omega = odom_.twist.twist.angular.z;
            Eigen::Vector3d z;
            z << robot_x, robot_y, robot_theta;

            kfPredict(v, omega, dt);
            kfUpdate(z);

            ekfPredict(v, omega, dt);
            ekfUpdate(z);

            pfPredict(v, omega, dt);
            pfUpdate(z);

            publishPose(kf_pose_pub_, x_kf_, now);
            publishPose(ekf_pose_pub_, x_ekf_, now);
            publishPose(pf_pose_pub_, pfEstimate(), now);
            publishPose(gt_pose_pub_, ground_truth_pose_);

            appendPoseToPath(kf_path_, x_kf_, now);
            kf_path_.header.stamp = now;
            kf_path_pub_->publish(kf_path_);

            appendPoseToPath(ekf_path_, x_ekf_, now);
            ekf_path_.header.stamp = now;
            ekf_path_pub_->publish(ekf_path_);

            Eigen::Vector3d pf_est = pfEstimate();
            appendPoseToPath(pf_path_, pf_est, now);
            pf_path_.header.stamp = now;
            pf_path_pub_->publish(pf_path_);
        }
    }

    void appendPoseToPath(nav_msgs::msg::Path &path,
                          const Eigen::Vector3d &pose,
                          rclcpp::Time stamp) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = stamp;
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose.position.x = pose(0);
        pose_stamped.pose.position.y = pose(1);
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = toQuaternion(pose(2));
        path.poses.push_back(pose_stamped);
    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2*M_PI;
        while (angle < -M_PI) angle += 2*M_PI;
        return angle;
    }
    geometry_msgs::msg::Quaternion toQuaternion(double yaw) {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        geometry_msgs::msg::Quaternion q_msg;
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();
        q_msg.w = q.w();
        return q_msg;
    }
    void publishPose(rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub,
                     const Eigen::Vector3d& pose, rclcpp::Time stamp) {
        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = stamp;
        p.header.frame_id = "odom";
        p.pose.position.x = pose(0);
        p.pose.position.y = pose(1);
        p.pose.position.z = 0.0;
        p.pose.orientation = toQuaternion(pose(2));
        pub->publish(p);
    }
    void publishPose(rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub,
                     const geometry_msgs::msg::PoseStamped& pose) {
        pub->publish(pose);
    }

    void savePathToCSV(const nav_msgs::msg::Path &path, const std::string &filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Kann Datei %s nicht öffnen", filename.c_str());
            return;
        }
        file << "x,y,yaw\n";
        for (const auto &pose_stamped : path.poses) {
            double x = pose_stamped.pose.position.x;
            double y = pose_stamped.pose.position.y;
            auto &q = pose_stamped.pose.orientation;
            double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            double yaw = std::atan2(siny_cosp, cosy_cosp);
            file << x << "," << y << "," << yaw << "\n";
        }
        file.close();
        RCLCPP_INFO(this->get_logger(), "Pfad in %s gespeichert", filename.c_str());
    }

    // Callback-Methoden
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
    }
    void gtCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        ground_truth_pose_ = *msg;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr kf_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pf_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_pub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr kf_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ekf_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pf_path_pub_;

    nav_msgs::msg::Path kf_path_;
    nav_msgs::msg::Path ekf_path_;
    nav_msgs::msg::Path pf_path_;

    nav_msgs::msg::Odometry odom_;
    geometry_msgs::msg::PoseStamped ground_truth_pose_;

    Eigen::Vector3d x_kf_, x_ekf_;
    Eigen::Matrix3d P_kf_, Q_kf_, R_kf_;
    Eigen::Matrix3d P_ekf_, Q_ekf_, R_ekf_;

    std::vector<Particle> particles_;
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> dist_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;

    std::vector<Eigen::Vector2d> waypoints_;
    size_t current_waypoint_idx_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiFilterNode>();

    std::thread spin_thread([&]() {
        rclcpp::spin(node);
    });

    spin_thread.join();

    // Nach dem Stoppen speichern
    node->saveAllPaths();

    rclcpp::shutdown();
    return 0;
}

