#include "arm_2026/arm_2026_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_2026
{

hardware_interface::CallbackReturn Arm2026System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  for (const auto & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  const std::size_t num_joints = joint_names_.size();
  hw_commands_.assign(num_joints, 0.0);
  hw_states_.assign(num_joints, 0.0);

  if (num_joints != 3)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Arm2026System"),
      "Expected exactly 3 joints, but got %zu.",
      num_joints);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Arm2026System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joint_names_.size());

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_states_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Arm2026System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joint_names_.size());

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    command_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }

  return command_interfaces;
}

bool Arm2026System::phidget_ok(PhidgetReturnCode code, const char * context) const
{
  if (code == EPHIDGET_OK)
  {
    return true;
  }

  const char * error_description = "Unknown Phidget error";
  Phidget_getErrorDescription(code, &error_description);

  RCLCPP_ERROR(
    rclcpp::get_logger("Arm2026System"),
    "Phidget call failed in %s: %s",
    context,
    error_description);

  return false;
}

double Arm2026System::radians_to_phidget_counts(double radians) const
{
  const double counts_per_rev =
    base_steps_per_rev_ * base_microstep_factor_ * base_gear_ratio_;
  return radians * counts_per_rev / (2.0 * M_PI);
}

double Arm2026System::phidget_counts_to_radians(double counts) const
{
  const double counts_per_rev =
    base_steps_per_rev_ * base_microstep_factor_ * base_gear_ratio_;
  return counts * (2.0 * M_PI) / counts_per_rev;
}

hardware_interface::CallbackReturn Arm2026System::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Arm2026System"), "Configuring Arm2026System...");

  // ----- ROS bridge for shoulder/elbow -----
  comms_node_ = std::make_shared<rclcpp::Node>("arm_2026_hw_bridge");
  actuator_cmd_pub_ = comms_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/arm_actuator_targets", 10);

  actuator_state_sub_ = comms_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/arm_actuator_states",
    10,
    std::bind(&Arm2026System::actuator_state_callback, this, std::placeholders::_1));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(comms_node_);

  executor_running_.store(true);
  executor_thread_ = std::thread([this]()
  {
    while (executor_running_.load())
    {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  // ----- Phidget base setup -----
  if (!phidget_ok(PhidgetStepper_create(&base_stepper_), "PhidgetStepper_create"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setDeviceSerialNumber(
          reinterpret_cast<PhidgetHandle>(base_stepper_), base_device_serial_),
        "Phidget_setDeviceSerialNumber"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setHubPort(
          reinterpret_cast<PhidgetHandle>(base_stepper_), base_hub_port_),
        "Phidget_setHubPort"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_setChannel(
          reinterpret_cast<PhidgetHandle>(base_stepper_), base_channel_),
        "Phidget_setChannel"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!phidget_ok(
        Phidget_openWaitForAttachment(
          reinterpret_cast<PhidgetHandle>(base_stepper_), 5000),
        "Phidget_openWaitForAttachment"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  base_stepper_attached_ = true;
  RCLCPP_INFO(
    rclcpp::get_logger("Arm2026System"),
    "Base Phidget attached on serial %d, hub port %d, channel %d",
    base_device_serial_,
    base_hub_port_,
    base_channel_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Arm2026System::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Arm2026System"), "Activating Arm2026System...");

  // Initialize software command state from measured state
  for (std::size_t i = 0; i < hw_commands_.size(); ++i)
  {
    hw_commands_[i] = hw_states_[i];
  }

  // Configure Phidget base
  if (base_stepper_ != nullptr && base_stepper_attached_)
  {
    if (!phidget_ok(
          PhidgetStepper_setAcceleration(base_stepper_, base_acceleration_counts_),
          "PhidgetStepper_setAcceleration"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setVelocityLimit(base_stepper_, base_velocity_limit_counts_),
          "PhidgetStepper_setVelocityLimit"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!phidget_ok(
          PhidgetStepper_setEngaged(base_stepper_, 1),
          "PhidgetStepper_setEngaged"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    double current_counts = 0.0;
    if (!phidget_ok(
          PhidgetStepper_getPosition(base_stepper_, &current_counts),
          "PhidgetStepper_getPosition"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_states_[0] = phidget_counts_to_radians(current_counts);
    hw_commands_[0] = hw_states_[0];

    if (!phidget_ok(
          PhidgetStepper_setTargetPosition(base_stepper_, current_counts),
          "PhidgetStepper_setTargetPosition"))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Arm2026System::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Arm2026System"), "Deactivating Arm2026System...");

  // Safely disengage the Phidget stepper
  if (base_stepper_ != nullptr)
  {
    phidget_ok(PhidgetStepper_setEngaged(base_stepper_, 0), "PhidgetStepper_setEngaged(0)");
    phidget_ok(
      Phidget_close(reinterpret_cast<PhidgetHandle>(base_stepper_)),
      "Phidget_close");
    phidget_ok(PhidgetStepper_delete(&base_stepper_), "PhidgetStepper_delete");
    base_stepper_ = nullptr;
    base_stepper_attached_ = false;
  }

  executor_running_.store(false);

  if (executor_thread_.joinable())
  {
    executor_thread_.join();
  }

  if (executor_ && comms_node_)
  {
    executor_->remove_node(comms_node_);
  }

  actuator_cmd_pub_.reset();
  actuator_state_sub_.reset();
  executor_.reset();
  comms_node_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

void Arm2026System::actuator_state_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("Arm2026System"),
      "Received /arm_actuator_states with fewer than 2 values.");
    return;
  }

  actuator_states_[0] = msg->data[0];
  actuator_states_[1] = msg->data[1];
  actuator_state_received_.store(true);
}

hardware_interface::return_type Arm2026System::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // ----- Joint 0: base_yaw from real Phidget -----
  if (base_stepper_ != nullptr && base_stepper_attached_)
  {
    double current_counts = 0.0;
    if (phidget_ok(
          PhidgetStepper_getPosition(base_stepper_, &current_counts),
          "PhidgetStepper_getPosition"))
    {
      hw_states_[0] = phidget_counts_to_radians(current_counts);
    }
  }

  // ----- Joints 1 and 2: shoulder and elbow from ROS bridge -----
  if (actuator_state_received_.load())
  {
    hw_states_[1] = actuator_states_[0];
    hw_states_[2] = actuator_states_[1];
  }
  else
  {
    hw_states_[1] = hw_commands_[1];
    hw_states_[2] = hw_commands_[2];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Arm2026System::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Clamp commands first
  hw_commands_[0] = std::clamp(hw_commands_[0], base_min_, base_max_);
  hw_commands_[1] = std::clamp(hw_commands_[1], shoulder_min_, shoulder_max_);
  hw_commands_[2] = std::clamp(hw_commands_[2], elbow_min_, elbow_max_);

  // ----- Base to Phidget -----
  if (base_stepper_ != nullptr && base_stepper_attached_)
  {
    const double target_counts = radians_to_phidget_counts(hw_commands_[0]);
    phidget_ok(
      PhidgetStepper_setTargetPosition(base_stepper_, target_counts),
      "PhidgetStepper_setTargetPosition");
  }

  // ----- Shoulder + elbow through ROS bridge -----
  actuator_commands_[0] = hw_commands_[1];
  actuator_commands_[1] = hw_commands_[2];

  if (actuator_cmd_pub_)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {actuator_commands_[0], actuator_commands_[1]};
    actuator_cmd_pub_->publish(msg);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace arm_2026

PLUGINLIB_EXPORT_CLASS(
  arm_2026::Arm2026System,
  hardware_interface::SystemInterface)
