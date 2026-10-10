#include "desk_wiping_controller/desk_wiping_controller.hpp"

#include <franka/model.h>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

namespace desk_wiping_controller {
void DeskWipingController::startSegment(const Vector3d& start,
                                        const Vector3d& goal,
                                        double duration) {
  segment_start_ = start;
  segment_goal_ = goal;
  segment_duration_ = duration;
  segment_time_ = 0.0;
  desired_position_ = start;
  desired_velocity_.setZero();
}

bool DeskWipingController::updateSegment(double dt) {
  segment_time_ += dt;
  const double t = std::min(segment_time_ / segment_duration_, 1.0);
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;
  const double position_scale = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
  const double velocity_scale = 30.0 * t2 - 60.0 * t3 + 30.0 * t4;
  const Vector3d displacement = segment_goal_ - segment_start_;

  desired_position_ = segment_start_ + position_scale * displacement;
  desired_velocity_ = velocity_scale / segment_duration_ * displacement;

  if (t < 1.0) {
    return false;
  }
  desired_position_ = segment_goal_;
  desired_velocity_.setZero();
  return true;
}

void DeskWipingController::startCross(double contact_z) {
  const Vector3d x(forward_distance_, 0.0, 0.0);
  const Vector3d y(0.0, lateral_distance_, 0.0);
  const Vector3d center(center_xy_.x(), center_xy_.y(), contact_z);

  waypoints_ = {center,     center + x, center,     center + y, center,
                center - y, center,     center - x, center};
  waypoint_index_ = 1;
  startSegment(waypoints_[0], waypoints_[1], cross_segment_duration_);
}

bool DeskWipingController::updateCross(double dt) {
  if (!updateSegment(dt)) {
    return false;
  }
  if (++waypoint_index_ == waypoints_.size()) {
    return true;
  }
  startSegment(waypoints_[waypoint_index_ - 1], waypoints_[waypoint_index_],
               cross_segment_duration_);
  return false;
}

void DeskWipingController::updateReference(double dt,
                                           const Vector3d& current_position,
                                           double force) {
  switch (phase_) {
    case Phase::MOVE_ABOVE:
      if (updateSegment(dt)) {
        phase_ = Phase::DESCEND;
        RCLCPP_INFO(get_node()->get_logger(), "[PHASE] MOVE_ABOVE -> DESCEND");
      }
      break;

    case Phase::DESCEND:
      desired_position_.z() -= descent_speed_ * dt;
      desired_velocity_ = -descent_speed_ * Vector3d::UnitZ();
      if (force >= contact_threshold_) {
        RCLCPP_INFO(get_node()->get_logger(),
                    "[PHASE] DESCEND -> FORCE_RAMP | force=%.3f, threshold=%.3f", force,
                    contact_threshold_);
        phase_ = Phase::FORCE_RAMP;
        force_ramp_time_ = 0.0;
        commanded_force_ = 0.0;
        desired_position_.z() = current_position.z();
        desired_velocity_.setZero();
      }
      break;

    case Phase::FORCE_RAMP: {
      force_ramp_time_ += dt;
      const double ramp = std::min(force_ramp_time_ / force_ramp_duration_, 1.0);
      commanded_force_ = ramp * desired_force_;
      if (ramp == 1.0) {
        RCLCPP_INFO(get_node()->get_logger(), "[PHASE] FORCE_RAMP -> CROSS | commanded_force=%.3f",
                    commanded_force_);
        startCross(current_position.z());
        phase_ = Phase::CROSS;
      }
      break;
    }

    case Phase::CROSS:
      commanded_force_ = desired_force_;
      if (updateCross(dt)) {
        RCLCPP_INFO(get_node()->get_logger(), "[PHASE] CROSS -> RETURN");
        startSegment(current_position, initial_position_, return_duration_);
        phase_ = Phase::RETURN;
      }
      break;

    case Phase::RETURN:
      commanded_force_ = 0.0;
      if (updateSegment(dt)) {
        phase_ = Phase::HOLD;
      }
      break;

    case Phase::HOLD:
      commanded_force_ = 0.0;
      desired_position_ = initial_position_;
      desired_velocity_.setZero();
      break;
  }
}

controller_interface::InterfaceConfiguration DeskWipingController::command_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int joint = 1; joint <= kNumberOfJoints; ++joint) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(joint) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration DeskWipingController::state_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& name : robot_model_->get_state_interface_names()) {
    config.names.push_back(name);
  }
  return config;
}

CallbackReturn DeskWipingController::on_init() {
  try {
    auto_declare<double>("force_sign", force_sign_);
    auto_declare<std::string>("arm_id", arm_id_);
    auto_declare<std::vector<double>>("center_xy", {center_xy_.x(), center_xy_.y()});
    auto_declare<double>("forward_distance", forward_distance_);
    auto_declare<double>("lateral_distance", lateral_distance_);
    auto_declare<double>("move_duration", move_duration_);
    auto_declare<double>("cross_segment_duration", cross_segment_duration_);
    auto_declare<double>("return_duration", return_duration_);
    auto_declare<double>("descent_speed", descent_speed_);
    auto_declare<double>("Kp_xy", kp_xy_);
    auto_declare<double>("Kd_xy", kd_xy_);
    auto_declare<double>("Kp_z", kp_z_);
    auto_declare<double>("Kd_z", kd_z_);
    auto_declare<double>("Kp_orientation", kp_orientation_);
    auto_declare<double>("Kd_orientation", kd_orientation_);
    auto_declare<double>("Kp_nullspace", kp_nullspace_);
    auto_declare<double>("Kd_nullspace", kd_nullspace_);
    auto_declare<double>("pseudoinverse_damping", pseudoinverse_damping_);
    auto_declare<std::vector<double>>("preferred_joint_position",
                                      {0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398});
    auto_declare<double>("desired_force", desired_force_);
    auto_declare<double>("contact_threshold", contact_threshold_);
    auto_declare<double>("force_ramp_duration", force_ramp_duration_);
    auto_declare<double>("Kp_force", kp_force_);
    auto_declare<double>("Kd_force", kd_force_);
    auto_declare<double>("force_filter_time_constant", force_filter_time_constant_);
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", exception.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DeskWipingController::on_configure(const rclcpp_lifecycle::State&) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  const auto center = get_node()->get_parameter("center_xy").as_double_array();
  if (center.size() != 2) {
    RCLCPP_ERROR(get_node()->get_logger(), "center_xy must have 2 values");
    return CallbackReturn::ERROR;
  }
  center_xy_ << center[0], center[1];

  const auto preferred_q = get_node()->get_parameter("preferred_joint_position").as_double_array();
  if (preferred_q.size() != kNumberOfJoints) {
    RCLCPP_ERROR(get_node()->get_logger(), "preferred_joint_position needs 7 values");
    return CallbackReturn::ERROR;
  }
  for (int joint = 0; joint < kNumberOfJoints; ++joint) {
    preferred_q_(joint) = preferred_q[joint];
  }

  const auto read_double = [this](const std::string& name) {
    return get_node()->get_parameter(name).as_double();
  };
  force_sign_ = read_double("force_sign");
  forward_distance_ = read_double("forward_distance");
  lateral_distance_ = read_double("lateral_distance");
  move_duration_ = read_double("move_duration");
  cross_segment_duration_ = read_double("cross_segment_duration");
  return_duration_ = read_double("return_duration");
  descent_speed_ = read_double("descent_speed");
  kp_xy_ = read_double("Kp_xy");
  kd_xy_ = read_double("Kd_xy");
  kp_z_ = read_double("Kp_z");
  kd_z_ = read_double("Kd_z");
  kp_orientation_ = read_double("Kp_orientation");
  kd_orientation_ = read_double("Kd_orientation");
  kp_nullspace_ = read_double("Kp_nullspace");
  kd_nullspace_ = read_double("Kd_nullspace");
  pseudoinverse_damping_ = read_double("pseudoinverse_damping");
  desired_force_ = read_double("desired_force");
  contact_threshold_ = read_double("contact_threshold");
  force_ramp_duration_ = read_double("force_ramp_duration");
  kp_force_ = read_double("Kp_force");
  kd_force_ = read_double("Kd_force");
  force_filter_time_constant_ = read_double("force_filter_time_constant");

  stiffness_.setZero();
  damping_.setZero();
  stiffness_.diagonal() << kp_xy_, kp_xy_, kp_z_, kp_orientation_, kp_orientation_, kp_orientation_;
  damping_.diagonal() << kd_xy_, kd_xy_, kd_z_, kd_orientation_, kd_orientation_, kd_orientation_;

  robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_id_ + "/robot_model", arm_id_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn DeskWipingController::on_activate(const rclcpp_lifecycle::State&) {
  robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  const auto pose_array = robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  const Eigen::Map<const Eigen::Matrix4d> pose(pose_array.data());
  initial_position_ = pose.block<3, 1>(0, 3);
  desired_orientation_ = Eigen::Quaterniond(pose.block<3, 3>(0, 0));

  const auto* state = robot_model_->getRobotState();
  const Eigen::Map<const Vector6d> wrench(state->O_F_ext_hat_K.data());
  force_bias_ = wrench.head<3>();
  filtered_force_ = 0.0;

  Vector3d above_center = initial_position_;
  above_center.x() = center_xy_.x();
  above_center.y() = center_xy_.y();
  startSegment(initial_position_, above_center, move_duration_);
  phase_ = Phase::MOVE_ABOVE;
  return CallbackReturn::SUCCESS;
}

CallbackReturn DeskWipingController::on_deactivate(const rclcpp_lifecycle::State&) {
  robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type DeskWipingController::update(const rclcpp::Time&,
                                                               const rclcpp::Duration& period) {
  const double dt = period.seconds();
  const auto pose_array = robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  const Eigen::Map<const Eigen::Matrix4d> pose(pose_array.data());
  const Vector3d current_position = pose.block<3, 1>(0, 3);
  Eigen::Quaterniond current_orientation(pose.block<3, 3>(0, 0));

  const auto jacobian_array = robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  const Eigen::Map<const Matrix6x7d> jacobian(jacobian_array.data());
  const auto coriolis_array = robot_model_->getCoriolisForceVector();
  const Eigen::Map<const Vector7d> coriolis(coriolis_array.data());
  const auto* state = robot_model_->getRobotState();
  const Eigen::Map<const Vector7d> dq(state->dq.data());
  const Eigen::Map<const Vector6d> wrench(state->O_F_ext_hat_K.data());

  const double raw_force = force_sign_ * (wrench(2) - force_bias_.z());
  const double alpha = dt / (force_filter_time_constant_ + dt);
  filtered_force_ += alpha * (raw_force - filtered_force_);

  RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                        "Filtered Z force: %.3f N", filtered_force_);

  updateReference(dt, current_position, filtered_force_);

  Vector6d error = Vector6d::Zero();
  error.head<3>() = current_position - desired_position_;
  if (desired_orientation_.coeffs().dot(current_orientation.coeffs()) < 0.0) {
    current_orientation.coeffs() *= -1.0;
  }
  const Eigen::Quaterniond rotation_error(current_orientation * desired_orientation_.inverse());
  const Eigen::AngleAxisd angle_axis(rotation_error);
  error.tail<3>() = angle_axis.axis() * angle_axis.angle();

  Vector6d desired_twist = Vector6d::Zero();
  desired_twist.head<3>() = desired_velocity_;
  const Vector6d current_twist = jacobian * dq;
  Vector6d task_wrench = -stiffness_ * error - damping_ * (current_twist - desired_twist);

  if (phase_ == Phase::FORCE_RAMP || phase_ == Phase::CROSS) {
    task_wrench.z() = -commanded_force_ - kp_force_ * (commanded_force_ - filtered_force_) -
                      kd_force_ * current_twist.z();
  }

  Matrix6d regularized = jacobian * jacobian.transpose();
  regularized.diagonal().array() += pseudoinverse_damping_ * pseudoinverse_damping_;
  const Matrix7x6d pseudoinverse = jacobian.transpose() * regularized.inverse();
  const Matrix7d nullspace =
      Matrix7d::Identity() - jacobian.transpose() * pseudoinverse.transpose();

  const Eigen::Map<const Vector7d> q(state->q.data());
  const Vector7d nullspace_torque =
      nullspace * (kp_nullspace_ * (preferred_q_ - q) - kd_nullspace_ * dq);
  const Vector7d torque = jacobian.transpose() * task_wrench + nullspace_torque + coriolis;

  if (!torque.allFinite()) {
    return controller_interface::return_type::ERROR;
  }
  for (int joint = 0; joint < kNumberOfJoints; ++joint) {
    command_interfaces_[joint].set_value(torque(joint));
  }
  return controller_interface::return_type::OK;
}

}  // namespace desk_wiping_controller

PLUGINLIB_EXPORT_CLASS(desk_wiping_controller::DeskWipingController,
                       controller_interface::ControllerInterface)
