// Author: Lucas Iervolino Gazignato

#include <Sabertooth.h>
#include <Encoder.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>

#define WHEEL_DISTANCE_X 0.2045335 // Half of the distance between front wheels
#define WHEEL_DISTANCE_Y 0.206375 // Half of the distance between front wheel and the rear wheels

#define ODOM_PERIOD 50 // 20 Hz

// importa o driver do sabertooth
Sabertooth STFront(128); 
Sabertooth STRear(129); /

float V1, V2, V3, V4;

// importa o driver do encoder
Encoder Encoder1(22, 23);  
Encoder Encoder2(32, 33);
Encoder Encoder3(42, 43);
Encoder Encoder4(50, 51);

long odom_timer; // timer para o odometro

// Calcule a diferença entre o valor que queremos mapear para o intervalo de entrada e o valor mínimo do intervalo de entrada 
float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Converte o valor de entrada de joystick para motor
int convertToMotor(float value) {
  float r = map(value, -1.085, 1.117, -127.0, 127.0);
  r = constrain(r, -127.0, 127.0);
  return (int) round(r);
}
// Ajusta a velocidade dos motores para os valores de Power1, Power2, Power3 e Power4
void moveMotors(int Power1, int Power2, int Power3, int Power4) {
  STFront.motor(1, Power1);
  STFront.motor(2, Power2);
  STRear.motor(1, Power3);
  STRear.motor(2, Power4);
}
// Ajusta os motores para potência zero
void stopMotors() {
  STFront.motor(1, 0);
  STFront.motor(2, 0);
  STRear.motor(1, 0);
  STRear.motor(2, 0);
}

// Recebe o comando de velocidade do tópico cmd_vel e converte para velocidade do motor
void cmd_vel_callback(const geometry_msgs::Twist& vel)
{
  // Mecanum Kinematics
  V1 = vel.linear.x - vel.linear.y - (WHEEL_DISTANCE_X + WHEEL_DISTANCE_Y) * vel.angular.z;
  V2 = vel.linear.x + vel.linear.y + (WHEEL_DISTANCE_X + WHEEL_DISTANCE_Y) * vel.angular.z;
  V3 = vel.linear.x + vel.linear.y - (WHEEL_DISTANCE_X + WHEEL_DISTANCE_Y) * vel.angular.z;
  V4 = vel.linear.x - vel.linear.y + (WHEEL_DISTANCE_X + WHEEL_DISTANCE_Y) * vel.angular.z;

  moveMotors(convertToMotor(V1), convertToMotor(V2), convertToMotor(V3), convertToMotor(V4));
}

// Crie um assinante que escute o tópico "cmd_vel"
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_callback);

// Define os codificadores para 0
void clearEncoders() {
  Encoder1.write(0);
  Encoder2.write(0);
  Encoder3.write(0);
  Encoder4.write(0);
}

std_msgs::Int32MultiArray enc_msg;
ros::Publisher pub("/robot_base/encoders", &enc_msg);

ros::NodeHandle nh; // Serial0

void setup() {
  nh.getHardware()->setBaud(38400);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  SabertoothTXPinSerial.begin(9600); // Serial1 (RX, TX) --> (19, 18)

  stopMotors();

  enc_msg.data = (long *)malloc(sizeof(long) * 4); // Arduino Due -> long = int32 = 4 bytes
  enc_msg.data_length = 4;

  clearEncoders();

  odom_timer = millis();
}
// Leia os valores do codificador e coloque-os no encoder_msg
void loop() {
  if (millis() - odom_timer >= ODOM_PERIOD) {
    enc_msg.data[0] = Encoder1.read();
    enc_msg.data[1] = Encoder2.read();
    enc_msg.data[2] = Encoder3.read();
    enc_msg.data[3] = Encoder4.read();

    odom_timer = millis();

    pub.publish(&enc_msg);

    clearEncoders();
  }
  // Cria um nó com o nome "talke"
  nh.spinOnce();
}
