#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_map_server/map_io.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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
    bool loadWaypointsFromYAML(const std::string& filepath);
    void publishPose(const Eigen::Vector3d& state, nav_msgs::msg::Path& path,
                     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
                     std::ofstream& file, int color);
    void saveMapImage();

    void predictPF(double v, double omega, double dt);
    void predictKF(double v, double omega, double dt);
    void predictEKF(double v, double omega, double dt);
    void updateKF(const Eigen::Vector3d& z);
    void updateEKF(const Eigen::Vector3d& z);
    void updatePF(const Eigen::Vector3d& z);
    double imu_yaw_ = 0.0;
    bool got_imu_ = false;
    bool goal_active_ = false;



    


    double simulateRaycast(const Eigen::Vector3d& state, double ray_angle);


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    sensor_msgs::msg::LaserScan last_scan_;
    bool got_scan_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr nav2_pose_sub_;


    double wheel_radius_ = 0.033;       //TurtleBot3
    double wheel_separation_ = 0.16;

    double last_left_pos_ = 0.0;
    double last_right_pos_ = 0.0;
    rclcpp::Time last_joint_time_;
    bool got_joint_state_ = false;

    Eigen::Vector3d joint_odom_pose_ = Eigen::Vector3d::Zero();


    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_goal_idx_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_pf_, path_pub_kf_, path_pub_ekf_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    nav_msgs::msg::Path path_pf_, path_kf_, path_ekf_;
    nav_msgs::msg::OccupancyGrid map_;
    std::ofstream file_pf_, file_kf_, file_ekf_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr nav2_plan_sub_;
    nav_msgs::msg::Path nav2_full_path_;
    std::ofstream file_nav2_;


    std::vector<Particle> particles_;
    const int N_ = 500;

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
    std::string map_yaml_path = pkg_path + "/maps/map.yaml";
        try {
                nav2_map_server::loadMapFromYaml(map_yaml_path, map_);
                RCLCPP_INFO(this->get_logger(), "Karte erfolgreich geladen: %s", map_yaml_path.c_str());
            } catch (const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Fehler beim Laden der Karte: %s", e.what());
            }

    std::string yaml_path = pkg_path + "/config/waypoints.yaml";
    if (!loadWaypointsFromYAML(yaml_path)) {
        RCLCPP_ERROR(this->get_logger(), "Fehler beim Laden der Waypoints.");
        rclcpp::shutdown();
        return;
    }


    path_pub_pf_ = this->create_publisher<nav_msgs::msg::Path>("pf_path", 10);
    path_pub_kf_ = this->create_publisher<nav_msgs::msg::Path>("kf_path", 10);
    path_pub_ekf_ = this->create_publisher<nav_msgs::msg::Path>("ekf_path", 10);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("filter_path_map", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    particles_.resize(N_);  // Platz reservieren

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
    std::normal_distribution<double> init_noise_pos(0.0, 0.05);   // Position ±5 cm
    std::normal_distribution<double> init_noise_yaw(0.0, 0.1);    // Yaw ±0.1 rad (~5.7°)

        for (auto& p : particles_) {
            double noisy_x = x + init_noise_pos(gen_);
            double noisy_y = y + init_noise_pos(gen_);
            double noisy_yaw = yaw + init_noise_yaw(gen_);

            // Yaw normalisieren
            while (noisy_yaw > M_PI) noisy_yaw -= 2 * M_PI;
            while (noisy_yaw < -M_PI) noisy_yaw += 2 * M_PI;

            p.state = Eigen::Vector3d(noisy_x, noisy_y, noisy_yaw);
            p.weight = 1.0 / static_cast<double>(N_);
        }


    RCLCPP_INFO(this->get_logger(), "Filter initialisiert: x=%.2f y=%.2f yaw=%.2f", x, y, yaw);
} else {
    RCLCPP_WARN(this->get_logger(), "map → base_link nicht verfügbar nach 3 Sekunden");
}

joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->name.size() < 2 || msg->position.size() < 2) return;

        double left_pos = msg->position[0];
        double right_pos = msg->position[1];

        if (!got_joint_state_) {
            last_left_pos_ = left_pos;
            last_right_pos_ = right_pos;
            last_joint_time_ = msg->header.stamp;
            got_joint_state_ = true;
            return;
        }

        double d_left = left_pos - last_left_pos_;
        double d_right = right_pos - last_right_pos_;
        double dt = (this->now() - last_joint_time_).seconds();

        last_left_pos_ = left_pos;
        last_right_pos_ = right_pos;
        last_joint_time_ = this->now();

        double d_left_m = d_left * wheel_radius_;
        double d_right_m = d_right * wheel_radius_;

        double d_center = (d_left_m + d_right_m) / 2.0;
        double d_theta = (d_right_m - d_left_m) / wheel_separation_;

        double theta = joint_odom_pose_(2);
        double dx = d_center * cos(theta);
        double dy = d_center * sin(theta);

        joint_odom_pose_ += Eigen::Vector3d(dx, dy, d_theta);

        // Normalisieren
        while (joint_odom_pose_(2) > M_PI) joint_odom_pose_(2) -= 2 * M_PI;
        while (joint_odom_pose_(2) < -M_PI) joint_odom_pose_(2) += 2 * M_PI;

        // Prädiktion
        double v = d_center / dt;
        double omega = d_theta / dt;
        predictPF(v, omega, dt);
        predictKF(v, omega, dt);
        predictEKF(v, omega, dt);

        // Warten, bis IMU bereit ist
        if (!got_imu_) return;

        // x, y von Odometrie, yaw von IMU
        Eigen::Vector3d fused_pose = joint_odom_pose_;
        fused_pose(2) = imu_yaw_;

        updatePF(fused_pose);
        updateKF(fused_pose);
        updateEKF(fused_pose);

        // Ausgabe
        Eigen::Vector3d pf_est = Eigen::Vector3d::Zero();
        for (const auto& p : particles_) pf_est += p.state;
        pf_est /= N_;

        publishPose(pf_est, path_pf_, path_pub_pf_, file_pf_, 200);
        publishPose(kf_state_, path_kf_, path_pub_kf_, file_kf_, 100);
        publishPose(ekf_state_, path_ekf_, path_pub_ekf_, file_ekf_, 150);

        map_.header.stamp = this->now();
        map_pub_->publish(map_);
    });


imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        imu_yaw_ = yaw;
        got_imu_ = true;

        RCLCPP_DEBUG(this->get_logger(), "IMU yaw=%.2f", yaw);
    });



    timer_ = this->create_wall_timer(1s, std::bind(&WaypointNavNode::sendNextGoal, this));

    path_pf_.header.frame_id = path_kf_.header.frame_id = path_ekf_.header.frame_id = "map";
    file_pf_.open("pf_path.csv");
    file_kf_.open("kf_path.csv");
    file_ekf_.open("ekf_path.csv");
    file_nav2_.open("nav2_path.csv");


    map_.header.frame_id = "map";
    map_.info.resolution = 0.05;
    map_.info.width = 500;
    map_.info.height = 500;
    map_.info.origin.position.x = -12.5;
    map_.info.origin.position.y = -12.5;
    map_.data.resize(map_.info.width * map_.info.height, 0);

    kf_P_ = ekf_P_ = Eigen::Matrix3d::Identity();
  

nav2_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;

        nav2_full_path_.poses.push_back(pose);
        nav2_full_path_.header.stamp = this->now();

        if (file_nav2_.is_open())
            file_nav2_ << pose.pose.position.x << "," << pose.pose.position.y << "\n";
    });


    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        this->last_scan_ = *msg;
        this->got_scan_ = true;
    });




    }

void WaypointNavNode::publishPose(const Eigen::Vector3d& state,
                                 nav_msgs::msg::Path& path,
                                 rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
                                 std::ofstream& file,
                                 int color)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = state(0);
    pose.pose.position.y = state(1);
    pose.pose.orientation.w = 1.0; // einfache Orientierung

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

void WaypointNavNode::sendNextGoal() {
    RCLCPP_INFO(this->get_logger(), "sendNextGoal() aufgerufen.");

    // Falls noch ein Ziel aktiv ist, nichts tun
    if (goal_active_) {
        RCLCPP_INFO(this->get_logger(), "Noch auf dem Weg zum Ziel, kein neues Ziel gesendet.");
        return;
    }

    // Warte auf Action-Server
    if (!client_->wait_for_action_server(5s)) {
        RCLCPP_WARN(this->get_logger(), "Nav2-Action-Server nicht verfügbar.");
        return;
    }

    // Alle Wegpunkte erreicht?
    if (current_goal_idx_ >= waypoints_.size()) {
        RCLCPP_INFO(this->get_logger(), "Alle Wegpunkte erreicht. Beende.");
        saveMapImage();
        rclcpp::shutdown();
        return;
    }

    // Zielnachricht vorbereiten
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = waypoints_[current_goal_idx_];

    RCLCPP_INFO(this->get_logger(), "Sende Ziel %ld: x=%.2f, y=%.2f",
                current_goal_idx_,
                goal_msg.pose.pose.position.x,
                goal_msg.pose.pose.position.y);

    // Callbacks definieren
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

    options.goal_response_callback = [this](auto future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(), "Ziel %ld wurde abgelehnt.", current_goal_idx_);
            goal_active_ = false;  // wichtig!
        } else {
            RCLCPP_INFO(this->get_logger(), "Ziel %ld wurde angenommen.", current_goal_idx_);
            goal_active_ = true;  // wichtig!
        }
    };

    options.feedback_callback = [](auto, auto feedback) {
        RCLCPP_INFO(rclcpp::get_logger("WaypointNavNode"),
                    "Feedback: x=%.2f, y=%.2f",
                    feedback->current_pose.pose.position.x,
                    feedback->current_pose.pose.position.y);
    };
options.result_callback = [this](const auto& result) {
    goal_active_ = false;

    if (!result.result) {
        RCLCPP_WARN(this->get_logger(), "Kein Ergebnisobjekt im Result-Callback.");
        return;
    }

    if (result.result->error_code == 0) {
        RCLCPP_INFO(this->get_logger(), "Ziel %ld erreicht.", current_goal_idx_);
    } else {
        RCLCPP_WARN(this->get_logger(), "Ziel %ld NICHT erfolgreich erreicht. Fehlercode: %d", current_goal_idx_, result.result->error_code);
    }

    current_goal_idx_++;
};



    // Ziel senden
    client_->async_send_goal(goal_msg, options);
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
            RCLCPP_INFO(this->get_logger(), "Wegpunkt geladen: x=%.2f y=%.2f",
             pose.pose.position.x, pose.pose.position.y);

        }
        return true;
    } catch (...) {
        return false;
    }
}

void WaypointNavNode::predictPF(double v, double omega, double dt) {
    std::normal_distribution<double> noise_v(0.0, 0.02);     // lineares Rauschen
    std::normal_distribution<double> noise_omega(0.0, 0.05); // Drehgeschwindigkeit

    for (auto& p : particles_) {
        double theta = p.state(2);

        double noisy_v = v + noise_v(gen_);
        double noisy_omega = omega + noise_omega(gen_);

        double dx = noisy_v * dt * cos(theta);
        double dy = noisy_v * dt * sin(theta);
        double dtheta = noisy_omega * dt;

        p.state += Eigen::Vector3d(dx, dy, dtheta);

        // Yaw normalisieren
        while (p.state(2) > M_PI) p.state(2) -= 2 * M_PI;
        while (p.state(2) < -M_PI) p.state(2) += 2 * M_PI;
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

void WaypointNavNode::updateKF(const Eigen::Vector3d& z) {
    // Messmatrix H (Identität 3x3 für Direktmessung von x,y,yaw)
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R = 0.1 * Eigen::Matrix3d::Identity();  // Messrauschen

    Eigen::Vector3d y = z - H * kf_state_;  // Innovation
    // Winkel normalisieren (yaw)
    while (y(2) > M_PI) y(2) -= 2 * M_PI;
    while (y(2) < -M_PI) y(2) += 2 * M_PI;

    Eigen::Matrix3d S = H * kf_P_ * H.transpose() + R;
    Eigen::Matrix3d K = kf_P_ * H.transpose() * S.inverse();

    kf_state_ = kf_state_ + K * y;
    kf_P_ = (Eigen::Matrix3d::Identity() - K * H) * kf_P_;
}

void WaypointNavNode::updateEKF(const Eigen::Vector3d& z) {
    // Messfunktion h(x) = Identität
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R = 0.1 * Eigen::Matrix3d::Identity();

    Eigen::Vector3d y = z - H * ekf_state_;
    // Winkel normalisieren
    while (y(2) > M_PI) y(2) -= 2 * M_PI;
    while (y(2) < -M_PI) y(2) += 2 * M_PI;

    Eigen::Matrix3d S = H * ekf_P_ * H.transpose() + R;
    Eigen::Matrix3d K = ekf_P_ * H.transpose() * S.inverse();

    ekf_state_ = ekf_state_ + K * y;
    ekf_P_ = (Eigen::Matrix3d::Identity() - K * H) * ekf_P_;
}

void WaypointNavNode::updatePF(const Eigen::Vector3d& z) {
    if (!got_scan_) return;

    double sigma = 0.2;  // Standardabweichung des Messfehlers (m)
    double sum_weights = 0.0;

    int num_beams = 10;
    int total_beams = static_cast<int>((last_scan_.angle_max - last_scan_.angle_min) / last_scan_.angle_increment);
    int step = std::max(1, total_beams / num_beams);

    for (auto& p : particles_) {
        double weight = 1.0;

        for (int i = 0; i < total_beams; i += step) {
            double angle = last_scan_.angle_min + i * last_scan_.angle_increment;
            double measured_range = last_scan_.ranges[i];
            if (measured_range >= last_scan_.range_max || measured_range <= last_scan_.range_min) continue;

            // Transformiere Strahlwinkel ins Weltkoordinatensystem
            double global_angle = p.state(2) + angle;

            // Simuliere Messung aus Karte
          double expected_range = simulateRaycast(p.state, global_angle);


            double diff = measured_range - expected_range;
            double prob = std::exp(-0.5 * diff * diff / (sigma * sigma));
            weight *= prob;
        }

        p.weight = weight;
        sum_weights += p.weight;
    }

    // Normalisierung
    for (auto& p : particles_) {
        p.weight /= (sum_weights + 1e-6);
    }

    // Resampling (wie gehabt)
    std::vector<double> weights;
    for (const auto& p : particles_) weights.push_back(p.weight);
    std::discrete_distribution<> dist(weights.begin(), weights.end());
    std::normal_distribution<double> roughening(0.0, 0.01);

    std::vector<Particle> new_particles;
    for (int i = 0; i < N_; ++i) {
        Particle sampled = particles_[dist(gen_)];
        sampled.state += Eigen::Vector3d(
            roughening(gen_),
            roughening(gen_),
            roughening(gen_) * 0.1);
        while (sampled.state(2) > M_PI) sampled.state(2) -= 2 * M_PI;
        while (sampled.state(2) < -M_PI) sampled.state(2) += 2 * M_PI;
        new_particles.push_back(sampled);
    }

    particles_ = new_particles;
}




double WaypointNavNode::simulateRaycast(const Eigen::Vector3d& state, double ray_angle) {
    double range_max = 5.0;  // max LiDAR range
    double step = map_.info.resolution;
    double ray_length = 0.0;

    for (double r = 0.0; r < range_max; r += step) {
        double x = state(0) + r * std::cos(ray_angle);
        double y = state(1) + r * std::sin(ray_angle);

        int mx = static_cast<int>((x - map_.info.origin.position.x) / map_.info.resolution);
        int my = static_cast<int>((y - map_.info.origin.position.y) / map_.info.resolution);

        if (mx < 0 || my < 0 || mx >= static_cast<int>(map_.info.width) || my >= static_cast<int>(map_.info.height))
            break;

        int value = map_.data[my * map_.info.width + mx];
        if (value > 50) break;  // Belegung erkannt

        ray_length = r;
    }

    return ray_length;
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



