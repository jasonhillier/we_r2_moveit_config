
#include <we_r2_hw_interface.h>

WER2HwInterface::WER2HwInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  _joint_position_publisher = new ros::Publisher(nh.advertise<sensor_msgs::JointState>("/dd/joint_states",3));
  
  ROS_INFO_NAMED("we_r2_hw_interface", "WER2HwInterface Ready.");
}

void WER2HwInterface::init()
{
  ros_control_boilerplate::GenericHWInterface::init();

  ROS_INFO_NAMED("we_r2_hw_interface", "init_joints=%ld", num_joints_);
}

void WER2HwInterface::read(ros::Duration &elapsed_time)
{
  // Robot publishes directory to /joint_state topic
  // noop
}

void WER2HwInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    joint_position_[joint_id] += joint_position_command_[joint_id];

  sensor_msgs::JointState joint_state;
  joint_state.name = joint_names_;
  joint_state.position = joint_position_;

  _joint_position_publisher->publish(joint_state);

  ROS_INFO_NAMED("we_r2_hw_interface", "write_pos=%f", joint_position_[0]);
}

void WER2HwInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}
