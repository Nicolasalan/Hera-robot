#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>


// incluindo alguns arquivos de outros pacotes que são necessários para criar o controlador e declarando um namespace para esta classe 
// (namespaces fornecem um método para evitar conflitos de nomes em projetos grandes).
namespace controller_ns {

// Aqui você está apenas declarando a classe, que será herdada de hardware_interface::EffortJointInterface. 
// Isso significa que este controlador poderá controlar apenas as articulações que usam uma interface de esforço.
class PositionController : public controller_interface::Controller<
                               hardware_interface::EffortJointInterface> {
public:
     // função init(), que será chamada quando seu controller for carregado pelo controller manager. Dentro desta função, você obterá o nome da junta que 
     // você controlará primeiro do Parameter Server
     bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
          std::string my_joint;
          if (!n.getParam("joint", my_joint)) {
               ROS_ERROR("Could not find joint name");
          return false;
     }

     joint_ = hw->getHandle(my_joint); // lança em caso de falha
     return true;
     }
     // definindo o comando que você vai enviar para o seu conjunto. Nesse caso, é um produto entre uma 
     // variável de erro e uma variável de ganho_. A variável de erro é definida como a diferença entre a posição atual (joint_.getPosition()) da articulação e a posição final (setpoint_) da articulação.
     void update(const ros::Time &time, const ros::Duration &period) {
          double error = setpoint_ - joint_.getPosition();
          joint_.setCommand(error * gain_);
     }
     // iniciando e parando o controlador. Nesse caso, não há nada a fazer.
     void starting(const ros::Time &time) {}
     void stopping(const ros::Time &time) {}

private:
     hardware_interface::JointHandle joint_;
     static constexpr double gain_ = 2.25;
     static constexpr double setpoint_ = 1.00;
};
// chamando o plug-in de macro especial PLUGINLIB_EXPORT_CLASS para permitir que esta classe seja carregada dinamicamente.
PLUGINLIB_EXPORT_CLASS(controller_ns::PositionController,
                    controller_interface::ControllerBase);
}