#ifndef RRBOT_HARDWARE_INTERFACE__RRBOT_HARDWARE_INTERFACE_HPP_
#define RRBOT_HARDWARE_INTERFACE__RRBOT_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"

namespace rrbot_hardware_interface
{
class HeraHardwareInterface : public hardware_interface::Hera
{
public:
  // três funções principais para usar na interface:
  bool init(ros::NodeHandle & root_nh, ros::NodeHandle & robot_hw_nh); // A função init() será usada para criar e registrar as interfaces conjuntas.

  bool read(const ros::Time time, const ros::Duration period); // A função read() será usada para ler os estados das juntas do hardware.

  bool write(const ros::Time time, const ros::Duration period); // A função write() será usada para escrever os comandos conjuntos no hardware.

private:
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_command_interface_;

  std::vector<double> hw_position_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;

  std::vector<std::string> joint_names_;
};

}  // namespace Hera_hardware_interface

#endif  // HERA_HARDWARE_INTERFACE_HPP_