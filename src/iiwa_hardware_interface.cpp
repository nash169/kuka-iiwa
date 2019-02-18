#include <kuka_iiwa/iiwa_hardware_interface.h>

namespace iiwa_hardware_interface
{
  IIWAHardwareInterface::IIWAHardwareInterface(ros::NodeHandle& nh) : nh_(nh)
  {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    // nh_.param("/iiwa/hardware_interface/loop_hz", loop_hz_, 0.1);
    // ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    // non_realtime_loop_ = nh_.createTimer(update_freq, &IIWAHardwareInterface::update, this);
    last_time_ = ros::Time::now();
  }

  IIWAHardwareInterface::~IIWAHardwareInterface()
  {

  }

  void IIWAHardwareInterface::waitForCommand()
  {
    kuka::fri::LBRClient::waitForCommand();

    if (robotState().getClientCommandMode() == kuka::fri::TORQUE)
      robotCommand().setTorque(joint_effort_command_.data());
  }

  void IIWAHardwareInterface::command()
  {
    kuka::fri::LBRClient::command();
    update();
  }

  void IIWAHardwareInterface::init()
  {
    // Get joint names
    nh_.getParam("/iiwa/hardware_interface/joints", joint_names_);
    num_joints_ = joint_names_.size();

    // Resize vectors
    joint_position_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_effort_.resize(num_joints_);
    joint_position_command_.resize(num_joints_);
    joint_velocity_command_.resize(num_joints_);
    joint_effort_command_.resize(num_joints_);

    // Initialize Controller
    for (int i = 0; i < num_joints_; ++i) {
      joint_position_[i] = joint_velocity_[i] = joint_effort_[i] = 0.;
      // Create joint state interface
      hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
      joint_state_interface_.registerHandle(jointStateHandle);

      // Create position joint interface
      hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
      // joint_limits_interface::JointLimits limits;
      // joint_limits_interface::SoftJointLimits softLimits;
      // joint_limits_interface::getJointLimits(joint.name, nh_, limits);
      // joint_limits_interface::PositionJointSoftLimitsHandle jointLimitHandle(jointPositionHandle, limits, softLimits);
      // position_joint_limits_interface_.registerHandle(jointLimitHandle);
      position_joint_interface_.registerHandle(jointPositionHandle);

      // Create effort joint interface
      hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
      effort_joint_interface_.registerHandle(jointEffortHandle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&effort_joint_interface_);
    // registerInterface(&position_joint_limits_interface_);
  }

  // void IIWAHardwareInterface::update(const ros::TimerEvent& e)
  // {
  //   elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  //   read();
  //   controller_manager_->update(ros::Time::now(), elapsed_time_);
  //   write(elapsed_time_);   
  // }

  void IIWAHardwareInterface::update()
  {
    read();
    controller_manager_->update(ros::Time::now(), get_period());
    write();
  }

  void IIWAHardwareInterface::read()
  {
    for (int i = 0; i < num_joints_; i++) {
      joint_position_[i] = robotState().getMeasuredJointPosition()[i];
    }
  }

  void IIWAHardwareInterface::write() // ros::Duration elapsed_time
  {
    // position_joint_limits_interface_.enforceLimits(elapsed_time);
    if (robotState().getClientCommandMode() == kuka::fri::TORQUE) {
      robotCommand().setTorque(joint_effort_command_.data());
      robotCommand().setJointPosition(joint_position_.data());
    }
    else if (robotState().getClientCommandMode() == kuka::fri::POSITION)
      robotCommand().setJointPosition(joint_position_command_.data());
    // else ERROR
  }

  ros::Duration IIWAHardwareInterface::get_period()
  {
    ros::Time current_time = ros::Time::now();
    ros::Duration period =  current_time - last_time_;
    last_time_ = current_time;
    return period;
  }
}