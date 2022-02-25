#include "controller_manager/controller_manager.h"
#include "rrbot_hardware_interface/rrbot_hardware_interface.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
     ros::init(argc, argv, "rrbot_hardware_interface");

     // NOTA: Executamos o loop ROS em uma thread separada como chamadas externas, como
     // como callbacks de serviço para carregar controladores podem bloquear o loop de controle (principal)
     ros::AsyncSpinner spinner(3);
     spinner.start();

     ros::NodeHandle root_nh;
     ros::NodeHandle robot_nh("~");

     rrbot_hardware_interface::RRBotHardwareInterface rrbot_hardware_interface;
     controller_manager::ControllerManager controller_manager(&rrbot_hardware_interface, root_nh);

     //Configura temporizadores
     ros::Time timestamp;
     ros::Duration period;
     auto stopwatch_last = std::chrono::steady_clock::now();
     auto stopwatch_now = stopwatch_last;

     rrbot_hardware_interface.init(root_nh, robot_nh);

     ros::Rate loop_rate(100);

     while(ros::ok())
     {
          // Recebe o estado atual do robô
          if (!rrbot_hardware_interface.read(timestamp, period)) {
               ROS_FATAL_NAMED("rrbot_hardware_interface",
                         "Failed to read state from robot. Shutting down!");
               ros::shutdown();
     }

     // Obtém a hora atual e o tempo decorrido desde a última leitura
     timestamp = ros::Time::now();
     stopwatch_now = std::chrono::steady_clock::now();
     period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(
          stopwatch_now - stopwatch_last).count());
     stopwatch_last = stopwatch_now;


     // Atualiza os controladores
     controller_manager.update(timestamp, period);

     // Envia novo setpoint para o robô
     rrbot_hardware_interface.write(timestamp, period);

     loop_rate.sleep();
     }

     spinner.stop();
     ROS_INFO_NAMED("rrbot_hardware_interface", "Shutting down.");

     return 0;
}