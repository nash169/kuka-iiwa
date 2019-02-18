#ifndef ROS_CONTROL__IIWA_HARDWARE_INTERFACE_HPP
#define ROS_CONTROL__IIWA_HARDWARE_INTERFACE_HPP

#include <kuka_iiwa/iiwa_hardware.h>

#include <kuka/fri/ClientApplication.h>
#include <kuka/fri/LBRClient.h>
#include <kuka/fri/UdpConnection.h>

namespace iiwa_hardware_interface
{
  // static const double POSITION_STEP_FACTOR = 10;
  // static const double VELOCITY_STEP_FACTOR = 10;

  class IIWAHardwareInterface : public iiwa_hardware_interface::IIWAHardawre, public kuka::fri::LBRClient
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
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    double p_error_, v_error_, e_error_;
  };
  
}

#endif // ROS_CONTROL__IIWA_HARDWARE_INTERFACE_HPP