#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "VtemControl.hpp"
#include "vtem_control_msgs/msg/fluid_pressures.hpp"
using std::placeholders::_1;

class InputPressuresSubscriber : public rclcpp::Node
{
public:
  vtem_control::VtemControl vtemControl_;

  InputPressuresSubscriber()
  : Node("input_pressures_sub")
  {
    // VTEM input pressures topic
    this->declare_parameter<std::string>("input_pressures_topic", "input_pressures");
    this->get_parameter("input_pressures_topic", vtem_input_pressures_topic_);

    // VTEM modbus network information
    this->declare_parameter<std::string>("modbus_node", "192.168.1.101");
    this->declare_parameter<std::string>("modbus_service", "502");
    this->get_parameter("modbus_node", modbus_node_);
    this->get_parameter("modbus_service", modbus_service_);

    subscription_ = this->create_subscription<vtem_control_msgs::msg::FluidPressures>(
      vtem_input_pressures_topic_.c_str(), 10, std::bind(&InputPressuresSubscriber::topic_callback, this, _1));
    
    // Create VtemControl object
    vtem_control::VtemControl vtemControl_(modbus_node_.c_str(), modbus_service_.c_str());

    // Connect to VTEM
    if (!vtemControl_.connect()) {
      throw std::invalid_argument("Failed to connect to VTEM!");
    }
  }
  ~InputPressuresSubscriber() {
    vtemControl_.disconnect();
  }

private:
  void topic_callback(const vtem_control_msgs::msg::FluidPressures::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I received msg with pressure[0]: %d mBar", (int) (msg->data[0].fluid_pressure/100));
    std::vector<int> input_pressures_mbar(msg->data.size(), 0);
    int idx = 0;
    for (auto it = msg->data.begin(); it != msg->data.end(); ++it) {
      sensor_msgs::msg::FluidPressure fluid_pressure_msg =  *it;
      input_pressures_mbar[idx] = (int) fluid_pressure_msg.fluid_pressure / 100;
      idx += 1;
    }
    vtemControl_.set_all_pressures(input_pressures_mbar);
  }
  rclcpp::Subscription<vtem_control_msgs::msg::FluidPressures>::SharedPtr subscription_;

  std::string vtem_input_pressures_topic_;
  std::string modbus_node_;
  std::string modbus_service_;
};

int main(int argc, char * argv[])
{
  printf("hello world vtem_control package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputPressuresSubscriber>());
  rclcpp::shutdown();
  return 0;
}
