#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

class PathPublisher : public rclcpp::Node {
public:
    PathPublisher() : Node("path_publisher"), count_(0) {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathPublisher::publish_path, this)
        );
        build_path();
        RCLCPP_INFO(this->get_logger(), "Path Publisher started!");
    }

private:
    void build_path() {
        //kare yolu
        /*path_msg_.header.frame_id = "map";

         double center_x = 7.0;
        double center_y = 7.0;
        double side = 4.0;
        int points_per_side = 40; // Her kenar için nokta sayısı

        // Sol alt köşe (başlangıç)
        double x0 = center_x - side / 2.0;
        double y0 = center_y - side / 2.0;

        // 1. kenar: sola -> sağa
        for (int i = 0; i < points_per_side; ++i) {
            geometry_msgs::msg::PoseStamped p;
            double t = static_cast<double>(i) / points_per_side;
            p.pose.position.x = x0 + t * side;
            p.pose.position.y = y0;
            p.pose.orientation.z = sin(0.0 / 2.0); // 0 rad (sağa)
            p.pose.orientation.w = cos(0.0 / 2.0);
            path_msg_.poses.push_back(p);
        }
        // 2. kenar: aşağıdan yukarı
        for (int i = 0; i < points_per_side; ++i) {
            geometry_msgs::msg::PoseStamped p;
            double t = static_cast<double>(i) / points_per_side;
            p.pose.position.x = x0 + side;
            p.pose.position.y = y0 + t * side;
            p.pose.orientation.z = sin((M_PI/2.0) / 2.0); // +y (yukarı)
            p.pose.orientation.w = cos((M_PI/2.0) / 2.0);
            path_msg_.poses.push_back(p);
        }
        // 3. kenar: sağdan sola
        for (int i = 0; i < points_per_side; ++i) {
            geometry_msgs::msg::PoseStamped p;
            double t = static_cast<double>(i) / points_per_side;
            p.pose.position.x = x0 + side - t * side;
            p.pose.position.y = y0 + side;
            p.pose.orientation.z = sin(M_PI / 2.0); // pi rad (sola)
            p.pose.orientation.w = cos(M_PI / 2.0);
            path_msg_.poses.push_back(p);
        }
        // 4. kenar: yukarıdan aşağı
        for (int i = 0; i < points_per_side; ++i) {
            geometry_msgs::msg::PoseStamped p;
            double t = static_cast<double>(i) / points_per_side;
            p.pose.position.x = x0;
            p.pose.position.y = y0 + side - t * side;
            p.pose.orientation.z = sin((-M_PI/2.0) / 2.0); // -y (aşağı)
            p.pose.orientation.w = cos((-M_PI/2.0) / 2.0);
            path_msg_.poses.push_back(p);
        }*/
            
        
       // sinüs yolu
        path_msg_.header.frame_id = "map";
        path_msg_.poses.clear();

        const double x_start    = 1.0;
        const double x_end      = 7.0;
        const double step       = 0.1;
        const double y_offset   = 2.0;
        const double amplitude  = 1.0;
        const double wavelength = 1.0;
        const double k = 2.0 * M_PI / wavelength;

        // true: soldan sağa (X artar), false: yukarıdan aşağı (Y artar)
        const bool left_to_right = true;

        for (double s = x_start; s <= x_end; s += step) {
            double f  = y_offset + amplitude * std::sin(k * s);  // sinüs
            double df = amplitude * k * std::cos(k * s);          // türev

            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = "map";
            p.pose.position.z = 0.0;
            p.pose.orientation.x = 0.0;
            p.pose.orientation.y = 0.0;

            if (left_to_right) {
                // X = s, Y = f(s)  -> soldan sağa
                p.pose.position.x = s;
                p.pose.position.y = f;
                double yaw = std::atan2(df, 1.0); // v = (1, df)
                p.pose.orientation.z = std::sin(yaw * 0.5);
                p.pose.orientation.w = std::cos(yaw * 0.5);
            } else {
                // X = f(s), Y = s  -> yukarıdan aşağı
                p.pose.position.x = f;
                p.pose.position.y = s;
                double yaw = std::atan2(1.0, df); // v = (df, 1)
                p.pose.orientation.z = std::sin(yaw * 0.5);
                p.pose.orientation.w = std::cos(yaw * 0.5);
            }

            path_msg_.poses.push_back(p); 
        }
    }

    void publish_path() {
        path_msg_.header.stamp = this->now();
        path_pub_->publish(path_msg_);
        RCLCPP_INFO(this->get_logger(), "Path published!");
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}