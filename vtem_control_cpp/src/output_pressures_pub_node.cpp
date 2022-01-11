#include <functional> 
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "pneumatic_actuation_msgs/msg/fluid_pressures.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "VtemControl.hpp"

using namespace std::chrono_literals;

class OutputPressuresPub : public rclcpp::Node
{
public:
  OutputPressuresPub()
  : Node("output_pressures_pub")
  {
    // publication frequency
    this->declare_parameter<float>("pub_freq", 50.);
    this->get_parameter("pub_freq", pub_freq_);
    timer_ = this->create_wall_timer(std::chrono::microseconds((int) (1000000 / pub_freq_)), std::bind(&OutputPressuresPub::timer_callback, this));

    // VTEM modbus network information
    this->declare_parameter<std::string>("modbus_node", "192.168.4.3");
    this->declare_parameter<std::string>("modbus_service", "502");
    this->get_parameter("modbus_node", modbus_node_);
    this->get_parameter("modbus_service", modbus_service_);

    // VTEM valve configuration params
    this->declare_parameter<int>("num_valves", 16);
    this->get_parameter("num_valves", num_valves_);

    // VTEM output pressures topic
    this->declare_parameter<std::string>("output_pressures_topic", "output_pressures");
    this->get_parameter("output_pressures_topic", output_pressures_topic_);
    this->declare_parameter<std::string>("output_pressures_array_topic", "output_pressures_array");
    this->get_parameter("output_pressures_array_topic", output_pressures_array_topic_);
    this->declare_parameter<std::string>("vtem_status_topic", "vtem_status");
    this->get_parameter("vtem_status_topic", vtem_status_topic_);

    pub_pressures_ = this->create_publisher<pneumatic_actuation_msgs::msg::FluidPressures>(output_pressures_topic_.c_str(), 10);
    pub_pressures_array_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(output_pressures_array_topic_.c_str(), 10);
    pub_vtem_status_ = this->create_publisher<std_msgs::msg::Bool>(vtem_status_topic_.c_str(), 10);
    
    // Create VtemControl object
    this->vtemControl_ = std::unique_ptr<vtem_control::VtemControl>(new vtem_control::VtemControl(modbus_node_.c_str(), modbus_service_.c_str(), num_valves_));

    // Connect to VTEM
    if (!vtemControl_->connect()) {
      throw std::invalid_argument("Failed to connect to VTEM!");
    }
  }
  ~OutputPressuresPub() {
    vtemControl_->disconnect();
  }

  std::unique_ptr<vtem_control::VtemControl> vtemControl_;
private:
  void timer_callback()
  {
    // publish vtem status
    bool vtem_status = vtemControl_->is_pressure_regulation_active();
    auto msg_vtem_status = std_msgs::msg::Bool();
    msg_vtem_status.data = vtem_status;
    pub_vtem_status_->publish(msg_vtem_status);

    std::vector<int> output_pressures_mbar(num_valves_);
    vtemControl_->get_all_pressures(&output_pressures_mbar);

    auto msg = pneumatic_actuation_msgs::msg::FluidPressures();
    msg.header.stamp = OutputPressuresPub::get_clock()->now();
    std::vector<sensor_msgs::msg::FluidPressure> fluid_pressure_msgs(output_pressures_mbar.size());

    auto msg_multi_array = std_msgs::msg::Float64MultiArray();
    msg_multi_array.layout = std_msgs::msg::MultiArrayLayout();
    msg_multi_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg_multi_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg_multi_array.layout.dim[0].label = "segments";
    msg_multi_array.layout.dim[1].label = "chambers";

    int idx = 0;
    std::vector<double> output_pressures(output_pressures_mbar.size());
    for (auto it = output_pressures_mbar.begin(); it != output_pressures_mbar.end(); ++it) {
      double pressure = (double) *it * 100;

      output_pressures[idx] = pressure;

      sensor_msgs::msg::FluidPressure fluid_pressure_msg;
      fluid_pressure_msg.header.stamp =  OutputPressuresPub::get_clock()->now();
      fluid_pressure_msg.fluid_pressure =  pressure;
      fluid_pressure_msgs[idx] = fluid_pressure_msg;

      idx += 1;
    }
    msg.data = fluid_pressure_msgs;
    msg_multi_array.data = output_pressures;

    pub_pressures_->publish(msg);
    pub_pressures_array_->publish(msg_multi_array);

    // RCLCPP_INFO(this->get_logger(), "I published FluidPressures with actual pressure [0]: %d mBar", (int) (msg.data[0].fluid_pressure/100));
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<pneumatic_actuation_msgs::msg::FluidPressures>::SharedPtr pub_pressures_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pressures_array_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_vtem_status_;
  size_t count_;

  std::string output_pressures_topic_;
  std::string output_pressures_array_topic_;
  std::string vtem_status_topic_;

  float pub_freq_;
  std::string modbus_node_;
  std::string modbus_service_;
  int num_valves_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OutputPressuresPub>());
  rclcpp::shutdown();
  return 0;
}
