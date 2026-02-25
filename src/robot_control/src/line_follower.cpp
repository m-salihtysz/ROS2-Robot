#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robot_interfaces/msg/line_error.hpp>

class LineFollower : public rclcpp::Node
{
public:
  LineFollower()
  : Node("line_follower")
  {
    declare_parameter<double>("linear_speed", 0.4);
    declare_parameter<double>("kp", 2.0);

    linear_speed_ = get_parameter("linear_speed").as_double();
    kp_ = get_parameter("kp").as_double();

    error_sub_ = create_subscription<robot_interfaces::msg::LineError>(
      "line_error", 10,
      std::bind(&LineFollower::errorCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(get_logger(),
      "Line Follower başlatıldı. Giriş: robot_interfaces/LineError (lateral_error m, heading_error rad)");
  }

private:
  void errorCallback(const robot_interfaces::msg::LineError::SharedPtr msg)
  {
    const double lateral = msg->lateral_error;   // +sol, -sağ
    const double heading = msg->heading_error;   // radyan

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_speed_;
    cmd.angular.z = -kp_ * lateral;  // lateral düzeltme (heading ileride eklenebilir)

    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<robot_interfaces::msg::LineError>::SharedPtr error_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  double linear_speed_;
  double kp_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineFollower>());
  rclcpp::shutdown();
  return 0;
}
