#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "VtemControl.hpp"
#include "vtem_control/msg/fluid_pressures.hpp"

using namespace std::chrono_literals;

class OutputPressuresPub : public rclcpp::Node
{
public:
  vtem_control::VtemControl vtemControl_;

  OutputPressuresPub()
  : Node("output_pressures_pub")
  {
    // VTEM input pressures topic
    this->declare_parameter<std::string>("vtem_control/output_pressures_topic", "vtem_control/output_pressures");
    this->get_parameter("vtem_control/output_pressures_topic", vtem_output_pressures_topic_);

    // VTEM modbus network information
    this->declare_parameter<std::string>("vtem_control/node", "192.168.1.101");
    this->declare_parameter<std::string>("vtem_control/service", "502");
    this->get_parameter("vtem_control/node", vtem_node_);
    this->get_parameter("vtem_control/service", vtem_service_);

    publisher_ = this->create_publisher<vtem_control::msg::FluidPressures>(vtem_output_pressures_topic_.c_str(), 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&OutputPressuresPub::timer_callback, this));
    
    // Create VtemControl object
    vtem_control::VtemControl vtemControl_(vtem_node_.c_str(), vtem_service_.c_str());

    // Connect to VTEM
    if (!vtemControl_.connect()) {
      throw std::invalid_argument("Failed to connect to VTEM!");
    }
  }
  ~OutputPressuresPub() {
    vtemControl_.disconnect();
  }

private:
  void timer_callback()
  {
    // auto message = std_msgs::msg::String();
    // message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vtem_control::msg::FluidPressures>::SharedPtr publisher_;
  size_t count_;

  std::string vtem_output_pressures_topic_;
  std::string vtem_node_;
  std::string vtem_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OutputPressuresPub>());
  rclcpp::shutdown();
  return 0;
}
