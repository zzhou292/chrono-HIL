#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "chrono/core/ChFrameMoving.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

struct Ros2BridgeConfig
{
  bool enabled = false;
  std::string node_name = "chrono_teleop_bridge";
  std::string driver_topic = "teleop/raw_input";
  std::string ego_state_topic = "teleop/ego_state";
  std::string actors_topic = "teleop/actors";
  std::string command_topic = "teleop/safety_cmd";
  std::string warning_topic = "teleop/warning";
};

struct TrackedVehicleState
{
  uint32_t id = 0;
  std::string label;
  bool active = false;
  chrono::ChVector3d pos;
  chrono::ChQuaterniond rot;
  chrono::ChVector3d lin_vel;
};

#ifdef ENABLE_ROS2_BRIDGE

#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "teleop_bridge_msgs/msg/control_command.hpp"
#include "teleop_bridge_msgs/msg/driver_input.hpp"
#include "teleop_bridge_msgs/msg/ego_state.hpp"
#include "teleop_bridge_msgs/msg/tracked_vehicle_array.hpp"
#include "teleop_bridge_msgs/msg/warning_status.hpp"

class Ros2Bridge
{
public:
  static std::unique_ptr<Ros2Bridge> Create(const Ros2BridgeConfig &config);
  ~Ros2Bridge();

  void PublishDriverInput(double time, int auto_mode, const chrono::vehicle::DriverInputs &raw_inputs,
                          const chrono::vehicle::DriverInputs &applied_inputs);
  void PublishEgoState(double time, const chrono::vehicle::WheeledVehicle &vehicle, double steering,
                       double beta_estimate = 0.0);
  void PublishActors(double time, const std::vector<TrackedVehicleState> &actors);

  std::optional<teleop_bridge_msgs::msg::ControlCommand> GetSafetyCommand();
  std::optional<teleop_bridge_msgs::msg::WarningStatus> GetWarningStatus();

private:
  explicit Ros2Bridge(const Ros2BridgeConfig &config);

  void SetupPublishers();
  void SetupSubscribers();
  builtin_interfaces::msg::Time ToRosTime(double time) const;
  geometry_msgs::msg::Pose ToPose(const chrono::ChVector3d &pos, const chrono::ChQuaterniond &rot) const;
  geometry_msgs::msg::Twist ToTwist(const chrono::ChVector3d &vel) const;

  void ControlCommandCallback(const teleop_bridge_msgs::msg::ControlCommand::SharedPtr msg);
  void WarningCallback(const teleop_bridge_msgs::msg::WarningStatus::SharedPtr msg);

  Ros2BridgeConfig config_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  rclcpp::Publisher<teleop_bridge_msgs::msg::DriverInput>::SharedPtr driver_pub_;
  rclcpp::Publisher<teleop_bridge_msgs::msg::EgoState>::SharedPtr ego_pub_;
  rclcpp::Publisher<teleop_bridge_msgs::msg::TrackedVehicleArray>::SharedPtr actors_pub_;

  rclcpp::Subscription<teleop_bridge_msgs::msg::ControlCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<teleop_bridge_msgs::msg::WarningStatus>::SharedPtr warning_sub_;

  std::mutex command_mutex_;
  std::optional<teleop_bridge_msgs::msg::ControlCommand> latest_command_;

  std::mutex warning_mutex_;
  std::optional<teleop_bridge_msgs::msg::WarningStatus> latest_warning_;
};

#endif
