#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

namespace desk_wiping_controller {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix6x7d = Eigen::Matrix<double, 6, 7>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Matrix7x6d = Eigen::Matrix<double, 7, 6>;
using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

/**
 * Wipes a cross pattern on a table with a Panda arm.
 *
 * On activation the controller moves above center_xy, descends until the
 * filtered Z contact force exceeds contact_threshold, ramps up to
 * desired_force, wipes the cross while regulating the normal force, and
 * returns to the initial pose. Motion uses a Cartesian impedance law; during
 * contact the Z axis is force-controlled instead.
 */
class DeskWipingController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  static constexpr int kNumberOfJoints = 7;
  static constexpr std::size_t kWaypointCount = 9;

  enum class Phase { MOVE_ABOVE, DESCEND, FORCE_RAMP, CROSS, RETURN, HOLD };

  void startSegment(const Vector3d& start, const Vector3d& goal, double duration);
  bool updateSegment(double dt);
  void startCross(double contact_z);
  bool updateCross(double dt);
  void updateReference(double dt, const Vector3d& current_position, double force);

  std::string arm_id_{"panda"};
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> robot_model_;

  Phase phase_{Phase::HOLD};
  Vector3d initial_position_{Vector3d::Zero()};
  Eigen::Vector2d center_xy_{0.45, 0.0};
  Vector3d desired_position_{Vector3d::Zero()};
  Vector3d desired_velocity_{Vector3d::Zero()};
  Eigen::Quaterniond desired_orientation_{Eigen::Quaterniond::Identity()};

  Vector3d segment_start_{Vector3d::Zero()};
  Vector3d segment_goal_{Vector3d::Zero()};
  double segment_time_{0.0};
  double segment_duration_{1.0};

  std::array<Vector3d, kWaypointCount> waypoints_{};
  std::size_t waypoint_index_{0};

  Matrix6d stiffness_{Matrix6d::Zero()};
  Matrix6d damping_{Matrix6d::Zero()};
  Vector7d preferred_q_{Vector7d::Zero()};
  Vector3d force_bias_{Vector3d::Zero()};
  // Makes the measured contact force positive when pressing on the table
  // (+1 in simulation, -1 on the real robot).
  double force_sign_{1.0};
  double filtered_force_{0.0};
  double force_ramp_time_{0.0};
  double commanded_force_{0.0};

  double forward_distance_{0.15};
  double lateral_distance_{0.15};
  double move_duration_{4.0};
  double cross_segment_duration_{2.0};
  double return_duration_{4.0};
  double descent_speed_{0.01};
  double kp_xy_{300.0};
  double kd_xy_{34.64};
  double kp_z_{40.0};
  double kd_z_{12.65};
  double kp_orientation_{20.0};
  double kd_orientation_{8.94};
  double kp_nullspace_{10.0};
  double kd_nullspace_{6.32};
  double pseudoinverse_damping_{0.05};
  double desired_force_{1.0};
  double contact_threshold_{0.25};
  double force_ramp_duration_{1.5};
  double kp_force_{1.0};
  double kd_force_{10.0};
  double force_filter_time_constant_{0.05};
};

}  // namespace desk_wiping_controller
