#ifndef ROBOT_HARDWARE__ARDUINO_SYSTEM_HPP_
#define ROBOT_HARDWARE__ARDUINO_SYSTEM_HPP_

#include <array>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace robot_hardware
{

class ArduinoSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  bool open_serial();
  void close_serial();

  bool send_line(const std::string & line);
  void process_serial();

  bool parse_position_report(
    const std::string & line);

  static constexpr std::size_t JOINT_COUNT = 5;

  std::array<double, JOINT_COUNT> hw_positions_{};
  std::array<double, JOINT_COUNT> hw_commands_{};
  std::array<double, JOINT_COUNT> last_commands_{};

  std::string serial_port_{"/dev/ttyUSB0"};
  int baud_rate_{115200};

  int serial_fd_{-1};

  std::string rx_buffer_;

  bool homed_{false};
  bool homing_requested_{false};

  bool have_valid_position_{false};

  bool first_write_{true};

  static constexpr double STEPS_PER_REV = 23755.0;
  static constexpr double STEPS_PER_RAD =
    STEPS_PER_REV / (2.0 * 3.14159265358979323846);
};

}  // namespace robot_hardware

#endif
