#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "VtemControl.hpp"
#include "vtem_control/msg/input_pressures.hpp"
using std::placeholders::_1;

class InputPressuresSubscriber : public rclcpp::Node
{
public:
  vtem_control::VtemControl vtemControl_;

  InputPressuresSubscriber()
  : Node("input_pressure_subscriber")
  {
    // VTEM input pressures topic
    this->declare_parameter<std::string>("vtem_input_pressures_topic", "vtem_input_pressures");
    this->get_parameter("vtem_input_pressures_topic", vtem_input_pressures_topic_);

    // VTEM modbus network information
    this->declare_parameter<std::string>("vtem_node", "192.168.1.101");
    this->declare_parameter<std::string>("vtem_service", "502");
    this->get_parameter("vtem_node", vtem_node_);
    this->get_parameter("vtem_service", vtem_service_);

    subscription_ = this->create_subscription<vtem_control::msg::InputPressures>(
      vtem_input_pressures_topic_.c_str(), 10, std::bind(&InputPressuresSubscriber::topic_callback, this, _1));
    
    // Create VtemControl object
    vtem_control::VtemControl vtemControl_(vtem_node_.c_str(), vtem_service_.c_str());

    // Connect to VTEM
    if (!vtemControl_.connect()) {
      throw std::invalid_argument("Failed to connect to VTEM!");
    }
  }
  ~InputPressuresSubscriber() {
    vtemControl_.disconnect();
  }

private:
  void topic_callback(const vtem_control::msg::InputPressures::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I received msg with pressure[0]: %d mBar", (int) (msg->data[0]/100));
    std::vector<float> input_pressures(msg->data.begin(), msg->data.end());
    std::transform(input_pressures.begin(), input_pressures.end(), input_pressures.begin(), [](float &c){ return c / 100;});
    std::vector<int> input_pressures_mbar(input_pressures.begin(), input_pressures.end());
    vtemControl_.set_all_pressures(input_pressures_mbar);
  }
  rclcpp::Subscription<vtem_control::msg::InputPressures>::SharedPtr subscription_;

  std::string vtem_input_pressures_topic_;
  std::string vtem_node_;
  std::string vtem_service_;
};

int main(int argc, char * argv[])
{
  printf("hello world vtem_control package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputPressuresSubscriber>());
  rclcpp::shutdown();
  return 0;
}
