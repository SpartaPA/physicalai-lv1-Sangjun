#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class DistanceSubscriber : public rclcpp::Node
{
public:
  DistanceSubscriber()
  : Node("distance_subscriber")
  {
    distance_subscriber_ =
      this->create_subscription<std_msgs::msg::Float32>(
        "/turtle_distance",
        10,
        [this](const std_msgs::msg::Float32::SharedPtr msg)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "Distance: %.2f m",
            msg->data);
        });

    RCLCPP_INFO(
      this->get_logger(),
      "Distance subscriber started");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
    distance_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DistanceSubscriber>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}