#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "turtlesim/msg/pose.hpp"

class DistancePublisher : public rclcpp::Node
{
public:
  DistancePublisher()
  : Node("distance_publisher")
  {
    pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose",
      10,
      [this](const turtlesim::msg::Pose::SharedPtr msg)
      {
        x_ = msg->x;
        y_ = msg->y;
      });

    distance_publisher_ =
      this->create_publisher<std_msgs::msg::Float32>(
        "/turtle_distance",
        10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]()
      {
        const double distance =
          std::sqrt(x_ * x_ + y_ * y_);

        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(distance);

        distance_publisher_->publish(msg);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "Distance publisher started");
  }

private:
  double x_{0.0};
  double y_{0.0};

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr
    pose_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
    distance_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DistancePublisher>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}