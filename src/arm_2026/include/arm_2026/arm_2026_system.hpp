#ifndef ARM_2026__ARM_2026_SYSTEM_HPP_
#define ARM_2026__ARM_2026_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <phidget22.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace arm_2026
{

class Arm2026System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Arm2026System)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void actuator_state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  double radians_to_phidget_position(double radians) const;
  double phidget_position_to_radians(double position) const;
  bool phidget_ok(PhidgetReturnCode code, const char * context) const;

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<std::string> joint_names_;

  rclcpp::Node::SharedPtr comms_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::atomic<bool> executor_running_{false};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr actuator_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr actuator_state_sub_;

  std::array<double, 2> actuator_commands_{{0.0, 0.0}};
  std::array<double, 2> actuator_states_{{0.0, 0.0}};
  std::atomic<bool> actuator_state_received_{false};

  // ----- Base joint limits -----
  double base_min_ = -3.14;
  double base_max_ = 3.14;

  // ----- Shoulder / elbow limits -----
  double shoulder_min_ = -1.57;
  double shoulder_max_ = 1.57;

  double elbow_min_ = -1.57;
  double elbow_max_ = 1.57;

  // ----- Phidget base configuration -----
  PhidgetStepperHandle base_stepper_ = nullptr;
  bool base_stepper_attached_ = false;

  int base_device_serial_ = 766944;
  int base_hub_port_ = 0;
  int base_channel_ = 0;

  // Standard 1.8 deg stepper -> 200 full steps/rev
  // Phidget stepper default microstepping is 1/16
  // Base output uses the measured Phidget rescale factor below.
  double base_steps_per_rev_ = 200.0;
  double base_microstep_factor_ = 16.0;
  double base_gear_ratio_ = 27.0;
  double base_rescale_factor_deg_ = 0.00416666667;

  // Keep the hardware profile aligned with the base joint URDF limit of 1.0 rad/s.
  // The Phidget is rescaled to output-shaft degrees, so convert here once.
  double base_velocity_limit_deg_ = 180.0 / M_PI;
  double base_acceleration_deg_ = 360.0 / M_PI;
};

}  // namespace arm_2026

#endif  // ARM_2026__ARM_2026_SYSTEM_HPP_
