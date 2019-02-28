#ifndef ROS_CONTROL__IIWA_HARDWARE_INTERFACE_HPP
#define ROS_CONTROL__IIWA_HARDWARE_INTERFACE_HPP

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>

#include <kuka/fri/ClientApplication.h>
#include <kuka/fri/LBRClient.h>
#include <kuka/fri/UdpConnection.h>

namespace iiwa_hardware_interface
{
  // static const double POSITION_STEP_FACTOR = 10;
  // static const double VELOCITY_STEP_FACTOR = 10;

  class IIWAHardwareInterface : public hardware_interface::RobotHW, public kuka::fri::LBRClient
  {
  public:
    IIWAHardwareInterface(ros::NodeHandle& nh);
    ~IIWAHardwareInterface();

    // FRI
    void waitForCommand() override;
    void command() override;

    // ROS control
    void init();
    // void update(const ros::TimerEvent& e);
    void update();
    void read();
    // void write(ros::Duration elapsed_time);
    void write();

    ros::Duration get_period();

  protected:
    ros::NodeHandle nh_;
    ros::Timer non_realtime_loop_;
    ros::Duration control_period_;
    ros::Duration elapsed_time_;
    ros::Time last_time_;
    // PositionJointInterface positionJointInterface;
    // PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
    double loop_hz_;
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    double p_error_, v_error_, e_error_;

    // Interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;

    joint_limits_interface::EffortJointSaturationInterface effort_joint_saturation_interface_;
    joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_limits_interface_;
    joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
    joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;
    joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
    joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limits_interface_;

    // Shared memory
    int num_joints_;
    int joint_mode_; // position, velocity, or effort
    std::vector<std::string> joint_names_;
    std::vector<int> joint_types_;
    std::vector<double> joint_position_,
                        joint_velocity_,
                        joint_effort_,
                        joint_position_command_,
                        joint_velocity_command_,
                        joint_effort_command_,
                        joint_lower_limits_,
                        joint_upper_limits_,
                        joint_effort_limits_;

  }; // class
  
} // namespace

#endif // ROS_CONTROL__IIWA_HARDWARE_INTERFACE_HPP