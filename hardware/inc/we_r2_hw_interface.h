
#ifndef WE_R2_HW_INTERFACE
#define WE_R2_HW_INTERFACE

#include <generic_hw_interface.h>
#include <sensor_msgs/JointState.h>

/// \brief Hardware interface for a robot
class WER2HwInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  WER2HwInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Initialize the hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration &elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration &elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration &period);

protected:
  ros::Publisher* _joint_position_publisher;

};  // class

#endif
