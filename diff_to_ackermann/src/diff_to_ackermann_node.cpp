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
      ticks_ = 0;
    }

  private:

    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      double wheelbase = 0.65; // from online search (https://general-laser.at/en/shop-en/hunter-2-0-en)

      // Convert differential drive to Ackermann drive
      geometry_msgs::msg::Twist ackermann_msg;
      
      // Calculate steering angle
      if (std::abs(msg->linear.x) < 0.01 && std::abs(msg->angular.z)>0.01) {
        // Ackermann drive cannot do pure rotation, it will therefore maneuvre until it gets there:
        if (ticks_ <= 5){  // go forward a while (arbitrarily chosen to 5 commands)
          ackermann_msg.linear.x = 2*msg->angular.z;  // speed proportional to angular velocity
        } else if (ticks_ <= 15) {  // go backwards while turning double the time
          ackermann_msg.linear.x = -2*msg->angular.z;
          if (msg->angular.z > 0){
            ackermann_msg.angular.z = max_steer_angle_;
          } else {
            ackermann_msg.angular.z = -max_steer_angle_;
          }
        } else if (ticks_ <= 20) {  // go forward again to go back to starting point
          ackermann_msg.linear.x = 2*msg->angular.z;
          if (ticks_ == 20){
            ticks_ = 0;
          }
        }
        ticks_++;
      } else {
        ackermann_msg.angular.z = std::atan2(wheelbase * msg->angular.z, std::abs(msg->linear.x));
        ackermann_msg.linear.x = msg->linear.x; // Linear velocity remains the same
      }
      publisher_->publish(ackermann_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    int ticks_; 
    const double max_steer_angle_ = 0.58;  // rad/s
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffToAck>());
  rclcpp::shutdown();
  return 0;
}