
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Example_Turtlebot3 : public rclcpp::Node {
public:
  Example_Turtlebot3() : Node("example_turtlebot3_node") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&Example_Turtlebot3::timer_callback, this));
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Example_Turtlebot3::topic_callback, this, _1));
  }

private:
  void timer_callback() {

    auto message = geometry_msgs::msg::Twist();

    if (!security_stop) {
      RCLCPP_INFO(this->get_logger(), "Go forward");
      // no front obstacle bellow the security distance
      message.linear.x = 0.3;
      message.angular.z = 0.0;
    } else {
      RCLCPP_INFO(this->get_logger(), "Stop because of obstacle");
      // front obstacle detected bellow the security distance
      message.linear.x = 0.0;
      message.angular.z = 0.0;
    }

    publisher_->publish(message);
  }

  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    float front_distance = msg->ranges[0]; // Reading in front of the robot

    RCLCPP_INFO(this->get_logger(), "Distance with front obstacle : '%f'",
                front_distance);

    if (front_distance <= security_distance) {
      security_stop = true;
    } else {
      security_stop = false;
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

  // the security stop will be changed in function of the laser value
  bool security_stop = false;
  float security_distance = 1.0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Example_Turtlebot3>());
  rclcpp::shutdown();
  return 0;
}
