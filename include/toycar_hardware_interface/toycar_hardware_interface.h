//#ifndef ROS_CONTROL__TOYCAR_HARDWARE_INTERFACE_H
//#define ROS_CONTROL__TOYCAR_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

class ToyCar:public hardware_interface::RobotHW
{
public:
    ToyCar(ros::NodeHandle& nh);
    ~ToyCar();
    void init();
    void update(const ros::TimerEvent& e);
    void read();
    void write(ros::Duration elapsed_time);
protected:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
    joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
    joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
    

    double joint_effort_[2];
    double joint_position_[2];
    double joint_velocity_[2];
    double joint_effort_command_[2];
    double joint_velocity_command_[2];
    
    ros::NodeHandle nh_;
    ros::Timer my_control_loop_;
    ros::Duration elapsed_time_;
    double loop_hz_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

//#endif
