#include <toycar_hardware_interface/toycar_hardware_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>

ToyCar::ToyCar(ros::NodeHandle& nh): nh_(nh){
    // Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot
    init();
    // Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_)); 
    //Set the frequency of the control loop.
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    //Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &ToyCar::update, this);
}

ToyCar::~ToyCar(){
}

void ToyCar::init()
{
    // Create joint_state_interface for FrontLeftWheelJoint
    hardware_interface::JointStateHandle jointStateHandleFrontLeftWheel("FrontLeftWheelJoint", &joint_velocity_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandleFrontLeftWheel);
    // Create effort joint interface as FrontLeftWheelJoint accepts effort command.
    hardware_interface::JointHandle jointEffortHandleFrontLeftWheel(jointStateHandleFrontLeftWheel, &joint_effort_command_[0]);
    effort_joint_interface_.registerHandle(jointEffortHandleFrontLeftWheel);
    // Create Joint Limit interface for FrontLeftWheelJoint
    joint_limits_interface::getJointLimits("FrontLeftWheelJoint", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandleFrontLeftWheel(jointEffortHandleFrontLeftWheel, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandleFrontLeftWheel);

    // Create joint_state_interface for FrontRightWheelJoint
    hardware_interface::JointStateHandle jointStateHandleFrontRightWheel("FrontRightWheelJoint", &joint_velocity_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(jointStateHandleFrontRightWheel);
    // Create effort joint interface as FrontRightWheelJoint accepts effort command..
    hardware_interface::JointHandle jointEffortHandleFrontRightWheel(jointStateHandleFrontRightWheel, &joint_effort_command_[1]);
    effort_joint_interface_.registerHandle(jointEffortHandleFrontRightWheel);
    // Create Joint Limit interface for FrontRightWheelJoint
    joint_limits_interface::getJointLimits("FrontRightWheelJoint", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandleFrontRightWheel(jointEffortHandleFrontRightWheel, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandleFrontRightWheel);

    // Create joint_state_interface for right_frontaxisjoint
    hardware_interface::JointStateHandle jointStateHandleLeftFrontAxis("right_frontaxisjoint", &joint_velocity_[2], &joint_velocity_[2], &joint_effort_[2]);
    joint_state_interface_.registerHandle(jointStateHandleLeftFrontAxis);
    // Create velocity joint interface as right_frontaxisjoint accepts velocity command.
    hardware_interface::JointHandle jointVelocityHandleLeftFrontAxis(jointStateHandleLeftFrontAxis, &joint_velocity_command_[0]);
    velocity_joint_interface_.registerHandle(jointVelocityHandleLeftFrontAxis);
    // Create Joint Limit interface for right_frontaxisjoint
    joint_limits_interface::getJointLimits("right_frontaxisjoint", nh_, limits);
    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandleLeftFrontAxis(jointVelocityHandleLeftFrontAxis, limits);
    velocityJointSaturationInterface.registerHandle(jointLimitsHandleLeftFrontAxis);

    // Create joint_state_interface for right_frontaxisjoint
    hardware_interface::JointStateHandle jointStateHandleRightFrontAxis("right_frontaxisjoint", &joint_velocity_[2], &joint_velocity_[2], &joint_effort_[2]);
    joint_state_interface_.registerHandle(jointStateHandleRightFrontAxis);
    // Create velocity joint interface as right_frontaxisjoint accepts velocity command.
    hardware_interface::JointHandle jointVelocityHandleRightFrontAxis(jointStateHandleRightFrontAxis, &joint_velocity_command_[1]);
    velocity_joint_interface_.registerHandle(jointVelocityHandleRightFrontAxis);
    // Create Joint Limit interface for right_frontaxisjoint
    joint_limits_interface::getJointLimits("right_frontaxisjoint", nh_, limits);
    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandleRightFrontAxis(jointVelocityHandleRightFrontAxis, limits);
    velocityJointSaturationInterface.registerHandle(jointLimitsHandleRightFrontAxis);

    // Register all joints interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&effortJointSaturationInterface);
    registerInterface(&velocityJointSaturationInterface);
}

//This is the control loop
void ToyCar::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ToyCar::read() {

  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to get the current joint velocity and/or velocity and/or effort       

  //from robot.
  // and fill JointStateHandle variables joint_velocity_[i], joint_velocity_[i] and joint_effort_[i]
}

void ToyCar::write(ros::Duration elapsed_time) {
  // Safety
  effortJointSaturationInterface.enforceLimits(elapsed_time);   // enforce limits for JointA and JointB
  velocityJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointC


  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
  // the output commands need to send are joint_effort_command_[0] for JointA, joint_effort_command_[1] for JointB and 

  //joint_velocity_command_ for JointC.

}

int main(int argc, char** argv)
{

    //Initialze the ROS node.
    ros::init(argc, argv, "toycar_hardware_interface_node");
    ros::NodeHandle nh;
    
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(2);
    
    
    // Create the object of the robot hardware_interface class and spin the thread. 
    ToyCar toycar(nh);
    spinner.spin();
    
    return 0;
}
