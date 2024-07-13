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
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "diffdrive_cmd", 10, std::bind(&DiffToAck::twistCallback, this, std::placeholders::_1));
      forward = 0;
    }

  private:

    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      double wheelbase = 0.65; // from online search (https://general-laser.at/en/shop-en/hunter-2-0-en)

      // Convert differential drive to Ackermann drive
      geometry_msgs::msg::Twist ackermann_msg;
      
      // Calculate steering angle
      if (std::abs(msg->linear.x) < 0.01 && std::abs(msg->angular.z)>0.01) {
        
        if (forward <= 10){
          ackermann_msg.linear.x = 0.5;
        } else if (forward <= 30) {
          ackermann_msg.linear.x = -1.0;
          ackermann_msg.angular.z = 20*msg->angular.z; // Pure rotation case
        } else if (forward <= 40) {
          ackermann_msg.linear.x = 0.5;
          if (forward == 40){
            forward = 0;
          }
        }
        forward++;
      } else {
        ackermann_msg.angular.z = std::atan2(wheelbase * msg->angular.z, std::abs(msg->linear.x));
        ackermann_msg.linear.x = msg->linear.x; // Linear velocity remains the same
      }
      publisher_->publish(ackermann_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    int forward; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffToAck>());
  rclcpp::shutdown();
  return 0;
}