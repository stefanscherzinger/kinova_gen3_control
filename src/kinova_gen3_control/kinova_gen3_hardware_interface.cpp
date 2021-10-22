#include "kinova_gen3_control/kinova_gen3_hardware_interface.h"
#include "angles/angles.h"
#include "ros/node_handle.h"
#include <array>

void
InitializeLowLevelControl(std::shared_ptr<KinovaNetworkConnection> network_connection)
{
  std::cout << "Initialize low-level control" << std::endl;

  auto servoing_mode = Kinova::Api::Base::ServoingModeInformation();
  servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  auto control_mode_message = Kinova::Api::ActuatorConfig::ControlModeInformation();
  control_mode_message.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::TORQUE);
  try
  {
    ROS_INFO("Set the servoing mode");
    // Set the base in low-level servoing mode
    network_connection->BaseSetServoingMode(servoing_mode);
    // Set last actuator in torque mode now that the command is equal to measure
    for (int i = 0; i < NUMBER_OF_JOINTS; i++)
    {
      // actuator_device_id is one-indexed
      int actuator_device_id_offset = FIRST_JOINT_INDEX;
      ROS_INFO("Set actuator %d control mode to low-level servoing", actuator_device_id_offset + i);
      network_connection->ActuatorSetControlMode(control_mode_message, actuator_device_id_offset + i);
    }
  }
  catch (Kinova::Api::KDetailedException& ex)
  {
    std::cout << "API error: " << ex.what() << std::endl;
    throw;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "Error: " << ex2.what() << std::endl;
    throw;
  }
  ROS_INFO("Low-level control initialized");
}

void
EndLowLevelControl(std::shared_ptr<KinovaNetworkConnection> network_connection)
{
  ROS_INFO("End low-level control");
  std::cout << "End low-level control" << std::endl;
  auto servoing_mode = Kinova::Api::Base::ServoingModeInformation();
  servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  auto control_mode_message = Kinova::Api::ActuatorConfig::ControlModeInformation();
  control_mode_message.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
  try
  {
    // Set the base back to regular servoing mode
    network_connection->BaseSetServoingMode(servoing_mode);
    // Set the actuators to position mode 
    for (int i = 0; i < NUMBER_OF_JOINTS; i++)
    {
      // actuator_device_id is one-indexed
      int actuator_device_id_offset = FIRST_JOINT_INDEX;
      ROS_INFO("Set actuator %d control mode to position control", actuator_device_id_offset + i);
      network_connection->ActuatorSetControlMode(control_mode_message, actuator_device_id_offset + i);
    }
  }
  catch (Kinova::Api::KDetailedException& ex)
  {
    std::cout << "API error: " << ex.what() << std::endl;
    throw;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "Error: " << ex2.what() << std::endl;
    throw;
  }
  ROS_INFO("Low-level control ended");
}

KinovaGen3HardwareInterface::KinovaGen3HardwareInterface(
  std::vector<std::string> joint_names,
  std::vector<joint_limits_interface::JointLimits> limits,
  std::shared_ptr<KinovaNetworkConnection> network_connection) : joint_names_(joint_names), limits_(limits)
{
  network_connection_ = network_connection;

  InitializeLowLevelControl(network_connection_);

  ROS_INFO("Register hardware interface");

  int array_index_offset = FIRST_JOINT_INDEX - 1;
  // connect and register the joint state interfaces
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    hardware_interface::JointStateHandle state_handle(joint_names_[i + array_index_offset], &pos_[i], &vel_[i], &eff_[i]);
    jnt_state_interface_.registerHandle(state_handle);
  }

  registerInterface(&jnt_state_interface_);

  // connect and register the joint position interfaces
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // note how a hardware_interface::JointHandle requires a hardware_interface::JointStateHandle in its constructor
    // in case you're wondering how this for loop relates to the for loop above
    hardware_interface::JointHandle eff_handle(jnt_state_interface_.getHandle(joint_names_[i + array_index_offset]), &eff_cmd_[i]);
    jnt_eff_interface_.registerHandle(eff_handle);

    hardware_interface::JointHandle pos_handle(jnt_state_interface_.getHandle(joint_names_[i + array_index_offset]), &pos_cmd_[i]);
    jnt_pos_interface_.registerHandle(pos_handle);

    hardware_interface::JointHandle cmd_handle(jnt_state_interface_.getHandle(joint_names_[i + array_index_offset]), &cmd_[i]);
    jnt_cmd_interface_.registerHandle(cmd_handle);
  }


  // set up the joint limiting interface so we can use it in "write"
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // Register handle in joint limits interface
    joint_limits_interface::EffortJointSaturationHandle cmd_limit_handle(jnt_cmd_interface_.getHandle(joint_names_[i + array_index_offset]), // We read the state and read/write the command
                                         limits_[i + array_index_offset]);       // Limits struct, copy constructor copies this
    jnt_cmd_limit_interface_.registerHandle(cmd_limit_handle);
  }

  // Setup pid controllers for the joint impedance rendering.
  // Each joint exposes a dynamic reconfigure server for its PID parameters.
  pid_controllers_.resize(NUMBER_OF_JOINTS);
  ros::NodeHandle nh("~");
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    const std::string config_ns = "joint_impedance/" + joint_names[i];
    if (!pid_controllers_[i].init(ros::NodeHandle(nh, config_ns)))
    {
      // default gains without dynamic reconfigure options
      pid_controllers_[i].initPid(1.0, 0, 0, 0, 0);
      ROS_WARN("%s: Couldn't find PID gains, using default: p:%f i:%f d:%f",
          joint_names[i].c_str(),
          pid_controllers_[i].getGains().p_gain_,
          pid_controllers_[i].getGains().i_gain_,
          pid_controllers_[i].getGains().d_gain_);
    }
  }

  registerInterface(&jnt_eff_interface_);
  registerInterface(&jnt_pos_interface_);
  ROS_INFO("Hardware interfaces registered");
}

KinovaGen3HardwareInterface::~KinovaGen3HardwareInterface()
{
  EndLowLevelControl(network_connection_);
}

void KinovaGen3HardwareInterface::write(const ros::Duration& period)
{
  if (NUMBER_OF_JOINTS == 7)
  {
    ROS_DEBUG_THROTTLE(1, "Effort command is %f, %f, %f, %f, %f, %f, %f", eff_cmd_[0], eff_cmd_[1], eff_cmd_[2], eff_cmd_[3], eff_cmd_[4], eff_cmd_[5], eff_cmd_[6]);
    ROS_DEBUG_THROTTLE(1, "Position command is %f, %f, %f, %f, %f, %f, %f", pos_cmd_[0], pos_cmd_[1], pos_cmd_[2], pos_cmd_[3], pos_cmd_[4], pos_cmd_[5], pos_cmd_[6]);
  }
  else
  {
   ROS_DEBUG_THROTTLE(1, "Effort command is %f", eff_cmd_[0]);
   ROS_DEBUG_THROTTLE(1, "Position command is %f", pos_cmd_[0]);
  }

  std::array<double, NUMBER_OF_JOINTS> dynamics;
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // Feedback linearization + feedforward torque + pid position control
    cmd_[i] = dynamics[i] + eff_cmd_[i] + pid_controllers_[i].computeCommand(pos_cmd_[i] - pos_[i], period);
  }

  Kinova::Api::BaseCyclic::Command  base_command;
  jnt_cmd_limit_interface_.enforceLimits(period);

  if (NUMBER_OF_JOINTS == 7)
  {
    ROS_DEBUG_THROTTLE(1, "Writing an effort of %f, %f, %f, %f, %f, %f, %f", cmd_[0], cmd_[1], cmd_[2], cmd_[3], cmd_[4], cmd_[5], cmd_[6]);
  }
  else
  {
    ROS_DEBUG_THROTTLE(1, "Writing an effort of %f", cmd_[0]);
  }


  // HARDCODE the total number of joints to 7 because we want to always
  // add position to each joint (even if only intentionally actuating fewer
  for (int i = 0; i < 7; i++)
  {
    // Keep joint position commands in sync with the current motion.
    // The Kinova actuator API requires this. If commanded and measured
    // positions diverge too much, the robot goes into an error state.
    // Always sending the current state lets the actuators appear backdrivable
    // and the torque control interface behaves as expected.
    base_command.add_actuators()->set_position(base_feedback_.actuators(i).position());
  }
  int array_index_offset = FIRST_JOINT_INDEX - 1;
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // Note that mutable_actuators is a 0-indexed array
    base_command.mutable_actuators(i + array_index_offset)->set_torque_joint(cmd_[i]);
  }

  try
  {
    network_connection_->CyclicRefresh(base_command);
  } 
  catch (Kinova::Api::KDetailedException& ex)
  {
    std::cout << "API error: " << ex.what() << std::endl;
    throw; 
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "Error: " << ex2.what() << std::endl;
    throw; 
  }
}

void KinovaGen3HardwareInterface::read()
{
  base_feedback_ = network_connection_->CyclicRefreshFeedback();

  int array_index_offset = FIRST_JOINT_INDEX - 1;
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    // Note that actuators is a 0-indexed array
    pos_[i] = angles::normalize_angle(angles::from_degrees(base_feedback_.actuators(i + array_index_offset).position())); // originally degrees
    vel_[i] = angles::from_degrees(base_feedback_.actuators(i + array_index_offset).velocity()); // originally degrees per second
    // NOTA BENE: The torque returned by the feedback call is ``wrong''.
    // It is -1 times what you would expect.
    // That is, if the feedback returns a _positive_ torque, it means the motor
    // is pushing the arm toward a more _negative_ position.
    // (You can check this by putting the arm in a static position 
    // and seeing which way the joint must be pushing to counteract gravity)
    // We fix this oddity by multiplying the feedback by -1 here to give the
    // correct torque that the joint is currently applying.
    // An additional benefit of this fix is that you can now take the read effort
    // and write that to the robot as a command to have the robot maintain its current torque.
    // Note that this oddity is also reflected on the Kinova Dashboard 
    // (a positive torque there also means the joint is trying to make the position more negative)
    eff_[i] = -base_feedback_.actuators(i + array_index_offset).torque(); // originally Newton * meters

    // Starting point for the impedance control law in each cycle.
    pos_cmd_[i] = pos_[i]; // nominal position
    eff_cmd_[i] = 0.0; // feedforward torque
  }
  
  if (NUMBER_OF_JOINTS == 7)
  {
    ROS_DEBUG_THROTTLE(1, "Read an effort of %f, %f, %f, %f, %f, %f, %f", eff_cmd_[0], eff_cmd_[1], eff_cmd_[2], eff_cmd_[3], eff_cmd_[4], eff_cmd_[5], eff_cmd_[6]);
  }
  else
  {
    ROS_DEBUG_THROTTLE(1, "Read effort %03.4f, vel %03.4f, pos %03.4f", eff_cmd_[0], vel_[0], pos_[0]);
  }
}
