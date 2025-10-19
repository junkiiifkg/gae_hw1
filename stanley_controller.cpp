#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

static double normalize_angle(double a) {
    while (a > M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
}

class StanleyController : public rclcpp::Node {
public:
    StanleyController()
    : Node("stanley_controller"),
      k_(2.0),
      kv_(1.5),
      max_omega_(8.0),
      accumulated_lateral_error_(0.0)
    {
        // Parametre: yol sonuna gelince dur
        this->declare_parameter("stop_at_goal", true);
        this->get_parameter("stop_at_goal", stop_at_goal_);

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "path_topic", 10, std::bind(&StanleyController::path_cb, this, _1));
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&StanleyController::pose_cb, this, _1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        last_time_ = this->now();
        last_print_time_ = this->now();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&StanleyController::update, this)); // 20 Hz

        RCLCPP_INFO(this->get_logger(), "Stanley controller started, stop_at_goal=%s", 
                    stop_at_goal_ ? "true" : "false");
    }

    double get_total_error() const { return accumulated_lateral_error_; }

private:
    struct WP { double x, y, yaw; };

    void path_cb(const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = *msg;
        waypoints_.clear();
        for (size_t i = 0; i < path_.poses.size(); ++i) {
            double x = path_.poses[i].pose.position.x;
            double y = path_.poses[i].pose.position.y;
            double yaw = 0.0;
            if (i + 1 < path_.poses.size()) {
                double nx = path_.poses[i+1].pose.position.x;
                double ny = path_.poses[i+1].pose.position.y;
                yaw = atan2(ny - y, nx - x);
            } else if (i > 0) {
                double px = path_.poses[i-1].pose.position.x;
                double py = path_.poses[i-1].pose.position.y;
                yaw = atan2(y - py, x - px);
            }
            waypoints_.push_back({x, y, yaw});
        }
        RCLCPP_INFO(this->get_logger(), "Received path with %zu waypoints", waypoints_.size());
    }

    void pose_cb(const turtlesim::msg::Pose::SharedPtr msg) {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_yaw_ = msg->theta;
        current_v_ = msg->linear_velocity;
    }

    void update() {
        if (waypoints_.empty()) {
            return;
        }

        // En yakın yol noktası
        size_t nearest_idx = 0;
        double best_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            double dx = current_x_ - waypoints_[i].x;
            double dy = current_y_ - waypoints_[i].y;
            double d2 = dx*dx + dy*dy;
            if (d2 < best_dist) {
                best_dist = d2;
                nearest_idx = i;
            }
        }

        // Yolun sonuna ulaşıldıysa ve parametre açıksa dur
        const double goal_tolerance = 0.1; // 10 cm
        if (stop_at_goal_ && nearest_idx == waypoints_.size() - 1) {
            double dx = current_x_ - waypoints_.back().x;
            double dy = current_y_ - waypoints_.back().y;
            if (std::sqrt(dx*dx + dy*dy) < goal_tolerance) {
                geometry_msgs::msg::Twist cmd;
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                cmd_pub_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping turtle.");
                return;
            }
        }

        double px = waypoints_[nearest_idx].x;
        double py = waypoints_[nearest_idx].y;
        double path_yaw = waypoints_[nearest_idx].yaw;

        // Stanley kontrol
        double heading_error = normalize_angle(path_yaw - current_yaw_);
        double dx = current_x_ - px;
        double dy = current_y_ - py;
        double lateral_error = dx * std::sin(path_yaw) - dy * std::cos(path_yaw);
        double abs_lateral = std::fabs(lateral_error);

        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0) dt = 0.05;
        accumulated_lateral_error_ += abs_lateral * dt;
        last_time_ = now;

        double eps = 1e-6;
        double v = current_v_;
        if (v < 0.01) v = 0.01;
        double steering = heading_error + std::atan2(k_ * lateral_error, v + eps);
        steering = normalize_angle(steering);

        double omega = kv_ * steering;
        if (omega > max_omega_) omega = max_omega_;
        if (omega < -max_omega_) omega = -max_omega_;

        double linear_cmd = std::clamp(1.2 / (1.0 + 2.0 * std::fabs(steering)), 0.3, 1.2);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear_cmd;
        cmd.angular.z = omega;
        cmd_pub_->publish(cmd);

        if ((now - last_print_time_).seconds() > 1.0) {
            RCLCPP_INFO(this->get_logger(),
                        "idx=%zu, lat_err=%.3f, acc_lat_err=%.3f, head_err=%.3f, steer=%.3f, v=%.3f",
                        nearest_idx, lateral_error, accumulated_lateral_error_, heading_error, steering, current_v_);
            last_print_time_ = now;
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path path_;
    std::vector<WP> waypoints_;

    double current_x_{5.5}, current_y_{5.5}, current_yaw_{0.0}, current_v_{0.0};

    double k_;
    double kv_;
    double max_omega_;

    rclcpp::Time last_time_;
    rclcpp::Time last_print_time_;
    double accumulated_lateral_error_;

    bool stop_at_goal_; // Parametre ile kontrol
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StanleyController>();
    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(),
                "Final total accumulated lateral error = %.6f", node->get_total_error());

    rclcpp::shutdown();
    return 0;
}
