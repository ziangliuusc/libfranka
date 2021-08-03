// Dylan Losey, November 14, 2019.
// A velocity controller for the Panda


#include <iostream>
#include <cmath>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sstream>
#include <string>
#include <cstring>
#define PORT 8080

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/gripper.h>


class VelocityController {

  public:

    VelocityController(franka::Model& model);
    franka::CartesianVelocities operator()(const franka::RobotState& robot_state, franka::Duration period);

  private:

    franka::Model* modelPtr;
    double time = 0.0;
    std::array<double, 100> buffer1;
    std::array<double, 100> buffer2;
    std::array<double, 100> buffer3;
    std::array<double, 100> buffer4;
    std::array<double, 100> buffer5;
    std::array<double, 100> buffer6;
    std::array<double, 6> xdot;
    std::array<double, 6> human_input;
    double MovingAverage(std::array<double, 100>& buffer, double input);

    int sock = 0;
    int valread;
    struct sockaddr_in serv_addr;
    char buffer[256] = {0};
    int steps = 0;

};


VelocityController::VelocityController(franka::Model& model) {

  modelPtr = &model;

  for (int i = 0; i < 6; i++) {
    human_input[i] = 0.0;
  }

  for (int i = 0; i < 100; i++) {
    buffer1[i] = 0;
    buffer2[i] = 0;
    buffer3[i] = 0;
    buffer4[i] = 0;
    buffer5[i] = 0;
    buffer6[i] = 0;
  }

  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    std::cout << "Socket Creation Error!" << std::endl;
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT);

  if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
    std::cout << "Invalid Address / Address Not Supported" << std::endl;
  }

  if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    std::cout << "Connection Failed" << std::endl;
  }

}


double VelocityController::MovingAverage(std::array<double, 100>& buffer, double input) {

  double filtered_input = 0.0;
  for (int i = 0; i < 100; i++) {
    filtered_input = filtered_input + buffer[i];
  }
  filtered_input = filtered_input / 100.0;
  for (int i = 0; i < 99; i++) {
    buffer[i] = buffer[i + 1];
  }
  buffer[99] = input;
  return filtered_input;

}


franka::CartesianVelocities VelocityController::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {

  time += period.toSec();

  std::array<double, 7> joint_position = robot_state.q;
  std::array<double, 7> joint_velocity = robot_state.dq;
  std::array<double, 7> applied_torque = robot_state.tau_ext_hat_filtered;
  std::array<double, 16> ee_pose = robot_state.O_T_EE;
  std::array<double, 42> jacobian = modelPtr->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  double send_rate = robot_state.control_command_success_rate;

  std::string state;
  for (int i = 0; i < 7; i++) {
    state.append(std::to_string(joint_position[i]));
    state.append(",");
  }
  for (int i = 0; i < 7; i++) {
    state.append(std::to_string(joint_velocity[i]));
    state.append(",");
  }
  for (int i = 0; i < 7; i++) {
    state.append(std::to_string(applied_torque[i]));
    state.append(",");
  }
  state.append(std::to_string(ee_pose[12]));
  state.append(",");
  state.append(std::to_string(ee_pose[13]));
  state.append(",");
  state.append(std::to_string(ee_pose[14]));
  for (int i = 0; i < 42; i++) {
    state.append(",");
    state.append(std::to_string(jacobian[i]));
  }
  char cstr[state.size() + 1];
  std::copy(state.begin(), state.end(), cstr);
  cstr[state.size()] = '\0';

  if (steps == 0) {
    send(sock, cstr, strlen(cstr), 0);
  } else if (steps > 500 && steps % 5 == 0) {
    std::cout << "control_command_success_rate: " << send_rate << std::endl;
    valread = read(sock, buffer, 256);
    send(sock, cstr, strlen(cstr), 0);
    std::stringstream ss(buffer);
    bool first = false;
    while (not first) {
      std::string substr;
      getline(ss, substr, ',');
      if ( substr[0] == 's') {
        first = true;
      }
    }
    for (int i = 0; i < 6; i++) {
      std::string substr;
      getline(ss, substr, ',');
      double term = std::stod(substr);
      human_input[i] = term;
    }
  }

  double xdot1 = MovingAverage(buffer1, human_input[0]);
  double xdot2 = MovingAverage(buffer2, human_input[1]);
  double xdot3 = MovingAverage(buffer3, human_input[2]);
  double xdot4 = MovingAverage(buffer4, human_input[3]);
  double xdot5 = MovingAverage(buffer5, human_input[4]);
  double xdot6 = MovingAverage(buffer6, human_input[5]);

  xdot = {{xdot1, xdot2, xdot3, xdot4, xdot5, xdot6}};
  franka::CartesianVelocities velocity(xdot);

  steps = steps + 1;
  return velocity;

}


int main() {

  try {

    franka::Robot robot("172.16.0.3");
    franka::Model model = robot.loadModel();
    robot.automaticErrorRecovery();

    VelocityController motion_generator(model);

    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});
    robot.setCartesianImpedance({{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}});

    robot.control(motion_generator, franka::ControllerMode::kCartesianImpedance);

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;

}
