#include "Ros2Bridge.h"

#ifdef ENABLE_ROS2_BRIDGE

#include <functional>
#include <iostream>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace chrono;
using namespace chrono::vehicle;

std::unique_ptr<Ros2Bridge> Ros2Bridge::Create(const Ros2BridgeConfig &config)
{
  try
  {
    return std::unique_ptr<Ros2Bridge>(new Ros2Bridge(config));
  }
  catch (const std::exception &exc)
  {
    std::cerr << "[Ros2Bridge] Failed to initialize ROS2 bridge: " << exc.what() << std::endl;
    return nullptr;
  }
}

Ros2Bridge::Ros2Bridge(const Ros2BridgeConfig &config) : config_(config)
{
  context_ = std::make_shared<rclcpp::Context>();
  const char * argv[] = {"proj_HIL_scm_teleop"};
  context_->init(1, argv);

  rclcpp::NodeOptions options;
  options.context(context_);
  node_ = std::make_shared<rclcpp::Node>(config_.node_name, options);
  SetupPublishers();
  SetupSubscribers();

  rclcpp::ExecutorOptions exec_options;
  exec_options.context = context_;
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(exec_options);
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });
}

Ros2Bridge::~Ros2Bridge()
{
  if (executor_)
  {
    executor_->cancel();
  }
  if (executor_thread_.joinable())
  {
    executor_thread_.join();
  }
  if (context_)
  {
    context_->shutdown("Teleop ROS2 bridge shutting down");
  }
}

void Ros2Bridge::SetupPublishers()
{
  driver_pub_ = node_->create_publisher<teleop_bridge_msgs::msg::DriverInput>(config_.driver_topic, 10);
  ego_pub_ = node_->create_publisher<teleop_bridge_msgs::msg::EgoState>(config_.ego_state_topic, 10);
  actors_pub_ = node_->create_publisher<teleop_bridge_msgs::msg::TrackedVehicleArray>(config_.actors_topic, 10);
}

void Ros2Bridge::SetupSubscribers()
{
  command_sub_ = node_->create_subscription<teleop_bridge_msgs::msg::ControlCommand>(
      config_.command_topic, 10, std::bind(&Ros2Bridge::ControlCommandCallback, this, std::placeholders::_1));
  warning_sub_ = node_->create_subscription<teleop_bridge_msgs::msg::WarningStatus>(
      config_.warning_topic, 10, std::bind(&Ros2Bridge::WarningCallback, this, std::placeholders::_1));
}

void Ros2Bridge::PublishDriverInput(double time, int auto_mode, const DriverInputs &raw_inputs,
                                    const DriverInputs &applied_inputs)
{
  if (!driver_pub_)
    return;

  teleop_bridge_msgs::msg::DriverInput msg;
  msg.stamp = node_->get_clock()->now();
  msg.auto_mode = auto_mode != 0;
  msg.raw_steering = static_cast<float>(raw_inputs.m_steering);
  msg.raw_throttle = static_cast<float>(raw_inputs.m_throttle);
  msg.raw_braking = static_cast<float>(raw_inputs.m_braking);
  msg.steering = static_cast<float>(applied_inputs.m_steering);
  msg.throttle = static_cast<float>(applied_inputs.m_throttle);
  msg.braking = static_cast<float>(applied_inputs.m_braking);
  driver_pub_->publish(msg);
}

void Ros2Bridge::PublishEgoState(double time, const WheeledVehicle &vehicle, double steering, double beta_estimate)
{
  if (!ego_pub_)
    return;
  teleop_bridge_msgs::msg::EgoState msg;
  msg.stamp = node_->get_clock()->now();

  auto chassis = vehicle.GetChassisBody();
  msg.pose = ToPose(chassis->GetPos(), chassis->GetRot());
  geometry_msgs::msg::Twist twist;
  auto vel = chassis->GetPosDt();
  twist.linear.x = vel.x();
  twist.linear.y = vel.y();
  twist.linear.z = vel.z();
  auto ang_vel = chassis->GetAngVelLocal();
  twist.angular.x = ang_vel.x();
  twist.angular.y = ang_vel.y();
  twist.angular.z = ang_vel.z();
  msg.twist = twist;
  msg.speed = static_cast<float>(vehicle.GetSpeed());
  msg.steering = static_cast<float>(steering);
  msg.beta = static_cast<float>(beta_estimate);
  ego_pub_->publish(msg);
}

void Ros2Bridge::PublishActors(double time, const std::vector<TrackedVehicleState> &actors)
{
  if (!actors_pub_)
    return;

  teleop_bridge_msgs::msg::TrackedVehicleArray array_msg;
  array_msg.stamp = node_->get_clock()->now();
  array_msg.vehicles.reserve(actors.size());
  for (const auto &actor : actors)
  {
    teleop_bridge_msgs::msg::TrackedVehicle item;
    item.stamp = array_msg.stamp;
    item.id = actor.id;
    item.label = actor.label;
    item.active = actor.active;
    item.pose = ToPose(actor.pos, actor.rot);
    item.twist = ToTwist(actor.lin_vel);
    array_msg.vehicles.push_back(item);
  }
  actors_pub_->publish(array_msg);
}

std::optional<teleop_bridge_msgs::msg::ControlCommand> Ros2Bridge::GetSafetyCommand()
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!latest_command_)
    return std::nullopt;
  auto cmd = latest_command_;
  latest_command_.reset();
  return cmd;
}

std::optional<teleop_bridge_msgs::msg::WarningStatus> Ros2Bridge::GetWarningStatus()
{
  std::lock_guard<std::mutex> lock(warning_mutex_);
  if (!latest_warning_)
    return std::nullopt;
  auto warning = latest_warning_;
  latest_warning_.reset();
  return warning;
}

void Ros2Bridge::ControlCommandCallback(const teleop_bridge_msgs::msg::ControlCommand::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  latest_command_ = *msg;

  rclcpp::Time now = node_->get_clock()->now();
  rclcpp::Time msg_time(msg->stamp);
  double latency = (now - msg_time).seconds();
  // Log every 100th message to avoid spam
  static int count = 0;
  if (count++ % 100 == 0) {
    RCLCPP_INFO(node_->get_logger(), "Round-trip latency: %.4f s", latency);
  }
}

void Ros2Bridge::WarningCallback(const teleop_bridge_msgs::msg::WarningStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(warning_mutex_);
  latest_warning_ = *msg;
}

builtin_interfaces::msg::Time Ros2Bridge::ToRosTime(double time) const
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(time);
  stamp.nanosec = static_cast<uint32_t>((time - stamp.sec) * 1e9);
  return stamp;
}

geometry_msgs::msg::Pose Ros2Bridge::ToPose(const ChVector3d &pos, const ChQuaterniond &rot) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = pos.x();
  pose.position.y = pos.y();
  pose.position.z = pos.z();
  pose.orientation.w = rot.e0();
  pose.orientation.x = rot.e1();
  pose.orientation.y = rot.e2();
  pose.orientation.z = rot.e3();
  return pose;
}

geometry_msgs::msg::Twist Ros2Bridge::ToTwist(const ChVector3d &vel) const
{
  geometry_msgs::msg::Twist tw;
  tw.linear.x = vel.x();
  tw.linear.y = vel.y();
  tw.linear.z = vel.z();
  return tw;
}

#endif
