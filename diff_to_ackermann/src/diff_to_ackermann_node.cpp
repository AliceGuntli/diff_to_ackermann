#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

/* This class creates a node that subscribes to differential drive velocity
commands and publishes ackerman velocity commands */

class DiffToAck : public rclcpp::Node
{
  public:
    DiffToAck(): Node("differantial_to_ackermann_node")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("topic", 10);
      subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "topic", 10, std::bind(&DiffToAck::twistCallback, this, std::placeholders::_1));
    }

  private:

    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffToAck>());
  rclcpp::shutdown();
  return 0;
}