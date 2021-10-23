#ifndef KINOVA_GEN3_HARDWARE_INTERFACE_H 
#define KINOVA_GEN3_HARDWARE_INTERFACE_H 

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <control_toolbox/pid.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chain.hpp>

#include <joint_limits_interface/joint_limits_interface.h>

#include <kinova_gen3_control/kinova_network_connection.h>


// 1-indexed first and last joint indices
// When not testing individual joints, this should be 1
#define FIRST_JOINT_INDEX 1
// When not testing individual joints, this should be 7
#define NUMBER_OF_JOINTS 7

class KinovaGen3HardwareInterface : public hardware_interface::RobotHW
{
  public:
    KinovaGen3HardwareInterface(
      std::vector<std::string> joint_names,
      std::vector<joint_limits_interface::JointLimits> limits,
      std::shared_ptr<KinovaNetworkConnection> network_connection);
    ~KinovaGen3HardwareInterface();
    void write(const ros::Duration& period);
    void read();

  private:
    std::shared_ptr<KinovaNetworkConnection> network_connection_;
    std::vector<std::string> joint_names_;
    std::vector<joint_limits_interface::JointLimits> limits_;
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::EffortJointInterface jnt_eff_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;
    hardware_interface::EffortJointInterface jnt_cmd_interface_;
    joint_limits_interface::EffortJointSaturationInterface jnt_cmd_limit_interface_;
    // no need to re-create this object every time it's used
    Kinova::Api::BaseCyclic::Feedback base_feedback_;
    double eff_cmd_[NUMBER_OF_JOINTS];
    double pos_cmd_[NUMBER_OF_JOINTS];
    double cmd_[NUMBER_OF_JOINTS];
    double pos_[NUMBER_OF_JOINTS];
    double vel_[NUMBER_OF_JOINTS];
    double eff_[NUMBER_OF_JOINTS];
    std::vector<control_toolbox::Pid> pid_controllers_;
    std::unique_ptr<KDL::ChainIdSolver_RNE> dynamics_solver_;
    KDL::Chain robot_chain_;
};
#endif // KINOVA_GEN3_HARDWARE_INTERFACE_H 
