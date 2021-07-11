#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "VtemControl.hpp"
#include "vtem_control/vtem_control_msgs/msg/fluid_pressures.hpp"

using namespace std::chrono_literals;

class OutputPressuresPub : public rclcpp::Node
{
public:
  vtem_control::VtemControl vtemControl_;

  OutputPressuresPub()
  : Node("output_pressures_pub")
  {
    // VTEM input pressures topic
    this->declare_parameter<std::string>("output_pressures_topic", "output_pressures");
    this->get_parameter("output_pressures_topic", vtem_output_pressures_topic_);

    // publication frequency
    this->declare_parameter<float>("pub_freq", 50.);
    this->get_parameter("pub_freq", pub_freq_);

    // VTEM modbus network information
    this->declare_parameter<std::string>("modbus_node", "192.168.1.101");
    this->declare_parameter<std::string>("modbus_service", "502");
    this->get_parameter("modbus_node", modbus_node_);
    this->get_parameter("modbus_service", modbus_service_);

    publisher_ = this->create_publisher<vtem_control::msg::FluidPressures>(vtem_output_pressures_topic_.c_str(), 10);
    timer_ = this->create_wall_timer(std::chrono::microseconds((int) (1000000 / pub_freq_)), std::bind(&OutputPressuresPub::timer_callback, this));
    
    // Create VtemControl object
    vtem_control::VtemControl vtemControl_(modbus_node_.c_str(), modbus_service_.c_str());

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
    std::vector<int> output_pressures_mbar;
    vtemControl_.get_all_pressures(&output_pressures_mbar);

    auto msg = vtem_control::msg::FluidPressures();
    msg.header.stamp = OutputPressuresPub::get_clock()->now();
    std::vector<sensor_msgs::msg::FluidPressure> fluid_pressure_msgs(output_pressures_mbar.size());

    int idx = 0;
    for (auto it = output_pressures_mbar.begin(); it != output_pressures_mbar.end(); ++it) {
      sensor_msgs::msg::FluidPressure fluid_pressure_msg;
      fluid_pressure_msg.header.stamp =  OutputPressuresPub::get_clock()->now();
      fluid_pressure_msg.fluid_pressure =  (float) *it * 100;
      fluid_pressure_msgs[idx] = fluid_pressure_msg;
      idx += 1;
    }
    msg.data = fluid_pressure_msgs;

    publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vtem_control::msg::FluidPressures>::SharedPtr publisher_;
  size_t count_;

  std::string vtem_output_pressures_topic_;
  float pub_freq_;
  std::string modbus_node_;
  std::string modbus_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OutputPressuresPub>());
  rclcpp::shutdown();
  return 0;
}
