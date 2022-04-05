#include "rrbot_hardware_interface/rrbot_hardware_interface.hpp"

#include <limits>
#include <vector>

namespace rrbot_hardware_interface
{

bool HeraHardwareInterface::init(ros::NodeHandle & /*root_nh*/, ros::NodeHandle & robot_hw_nh)
{   
     if (!robot_hw_nh.getParam("joint_names", joint_names_))
     {
          ROS_ERROR("Cannot find required parameter 'joint_names' on the parameter server.");
          throw std::runtime_error("Cannot find required parameter "
          "'joint_names' on the parameter server.");
     }

  size_t num_joints = joint_names_.size();
  ROS_INFO_NAMED("HeraHardwareInterface", "Found %zu joints.", num_joints);

  hw_position_states_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_position_commands_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_effort_states_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());

  // Cria interfaces ros_control
  // percorrendo todas as juntas do robô e registrando ambas as interfaces para cada uma das juntas.
  for (size_t i = 0; i < num_joints; ++i)
  {
     // Cria uma interface de estado de junta para todas as juntas
     joint_state_interface_.registerHandle(
          hardware_interface::JointStateHandle(
               joint_names_[i], &hw_position_states_[i], &hw_velocity_states_[i], &hw_effort_states_[i]));

     // Cria interface de controle de posição conjunta
     position_command_interface_.registerHandle(
          hardware_interface::JointHandle(
               joint_state_interface_.getHandle(joint_names_[i]), &hw_position_commands_[i]));
     }

     registerInterface(&joint_state_interface_);
     registerInterface(&position_command_interface_);

     // execução de estatísticas no hardware
     ROS_INFO_NAMED("HeraHardwareInterface", "Starting...");

     // neste exemplo simples redefine o estado para as posições iniciais
     for (size_t i = 0; i < num_joints; ++i){
          hw_position_states_[i] = 0.0;  // INITIAL POSITION is ZERO
          hw_position_commands_[i] = hw_position_states_[i];
     }

     return true;
}

bool HeraHardwareInterface::read(
     // fazendo um loop sobre todas as juntas do robô e imprimindo o estado.
     const ros::Time time, const ros::Duration period)
{
     // lê os estados do robô do hardware, neste exemplo apenas imprime
     ROS_INFO_NAMED("HeraHardwareInterface", "Reading...");

     // escreve o comando no hardware, neste exemplo espelha o comando nos estados
     for (size_t i = 0; i < hw_position_states_.size(); ++i) 
     {
          ROS_INFO_NAMED("HeraHardwareInterface",
                   "Got state %.2f for joint %zu!", hw_position_states_[i], i);
     }

     return true;
}

bool HeraHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
     // faça um loop sobre todas as juntas do robô e escreva o comando de posição desejado nas juntas.
     // escreve o comando no hardware, neste exemplo espelha o comando nos estados
     for (size_t i = 0; i < hw_position_commands_.size(); ++i) 
     {
          hw_position_states_[i] = hw_position_states_[i] +
                             (hw_position_commands_[i] - hw_position_states_[i]) / 100.0;
     }

     return true;
}

}