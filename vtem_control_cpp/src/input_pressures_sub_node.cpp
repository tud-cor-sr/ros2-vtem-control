#include <algorithm>    // std::min
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"
#include "pneumatic_actuation_msgs/msg/fluid_pressures.hpp"
#include "VtemControl.hpp"

using std::placeholders::_1;

class InputPressuresSubscriber : public rclcpp::Node
{
public:
  InputPressuresSubscriber()
  : Node("input_pressures_sub")
  {
    // VTEM input pressures topic
    this->declare_parameter<std::string>("input_pressures_topic", "input_pressures");
    this->get_parameter("input_pressures_topic", vtem_input_pressures_topic_);

    // VTEM modbus network information
    this->declare_parameter<std::string>("modbus_node", "192.168.4.3");
    this->declare_parameter<std::string>("modbus_service", "502");
    this->get_parameter("modbus_node", modbus_node_);
    this->get_parameter("modbus_service", modbus_service_);

    // VTEM valve configuration params
    this->declare_parameter<int>("num_valves", 16);
    this->get_parameter("num_valves", num_valves_);

    // Safety feature: maximum pressure allowed
    this->declare_parameter<double>("max_pressure", 300*100.0); // [Pa]
    this->get_parameter("max_pressure", max_pressure_);

    subscription_ = this->create_subscription<pneumatic_actuation_msgs::msg::FluidPressures>(
      vtem_input_pressures_topic_.c_str(), 10, std::bind(&InputPressuresSubscriber::topic_callback, this, _1));
    
    // Create VtemControl object
    this->vtemControl_ = std::unique_ptr<vtem_control::VtemControl>(new vtem_control::VtemControl(modbus_node_.c_str(), modbus_service_.c_str(), num_valves_));

    // Connect to VTEM
    RCLCPP_INFO(this->get_logger(), "Connecting to VTEM now via Modbus...");
    if (!this->vtemControl_->connect()) {
      throw std::invalid_argument("Failed to connect to VTEM!");
    }
    RCLCPP_INFO(this->get_logger(), "Connected to VTEM!");

    // acknowledge errors for all slots
    RCLCPP_INFO(this->get_logger(), "Acknowledging all errors!");
    this->vtemControl_->acknowledge_errors();

    // Set motion app for all valves to 03 (proportional pressure regulation)
    RCLCPP_INFO(this->get_logger(), "Activating pressure regulation now...");
    if (!this->vtemControl_->activate_pressure_regulation()) {
      throw std::invalid_argument("Failed to activate pressure regulation!");
    }
    RCLCPP_INFO(this->get_logger(), "Pressure regulation is activated!");
  }
  ~InputPressuresSubscriber() {
    RCLCPP_INFO(this->get_logger(), "Deactivating pressure regulation now...");
    this->vtemControl_->deactivate_pressure_regulation();
    this->vtemControl_->disconnect();
    RCLCPP_INFO(this->get_logger(), "Pressure regulation is deactivated!");
  }

  std::unique_ptr<vtem_control::VtemControl> vtemControl_;
private:
  void topic_callback(const pneumatic_actuation_msgs::msg::FluidPressures::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "I received FluidPressures with commanded pressure [0]: %d mBar", (int) (msg->data[0].fluid_pressure/100));

    // assert the num of valves configured to be equal to the number of commanded pressures
    rcpputils::require_true(num_valves_ == (int) msg->data.size());

    std::vector<int> input_pressures_mbar(msg->data.size(), 0);
    int idx = 0;
    for (auto it = msg->data.begin(); it != msg->data.end(); ++it) {
      sensor_msgs::msg::FluidPressure fluid_pressure_msg =  *it;
      double input_pressure = std::max(std::min(fluid_pressure_msg.fluid_pressure, max_pressure_), 0.0);
      input_pressures_mbar[idx] = (int) input_pressure / 100;
      idx += 1;
    }
    vtemControl_->set_all_pressures(input_pressures_mbar);
  }
  rclcpp::Subscription<pneumatic_actuation_msgs::msg::FluidPressures>::SharedPtr subscription_;

  std::string vtem_input_pressures_topic_;
  std::string modbus_node_;
  std::string modbus_service_;
  int num_valves_;
  double max_pressure_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputPressuresSubscriber>());
  rclcpp::shutdown();
  return 0;
}
