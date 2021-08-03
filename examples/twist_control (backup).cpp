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
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <fcntl.h>


class VelocityController {

  public:

    VelocityController(int port);
    franka::CartesianVelocities operator()(const franka::RobotState& robot_state, franka::Duration period);

  private:

    double time = 0.0;
    std::array<double, 200> buffer1;
    std::array<double, 200> buffer2;
    std::array<double, 200> buffer3;
    std::array<double, 200> buffer4;
    std::array<double, 200> buffer5;
    std::array<double, 200> buffer6;
    std::array<double, 200> bufferRate;
    std::array<double, 6> xdot;
    std::array<double, 6> human_input;
    double MovingAverage(std::array<double, 200>& buffer, double input);

    int sock = 0;
    int valread;
    struct sockaddr_in serv_addr;
    char buffer[200] = {0};
    int steps = 0;

};


VelocityController::VelocityController(int port) {

  for (int i = 0; i < 6; i++) {
    human_input[i] = 0.0;
  }

  for (int i = 0; i < 200; i++) {
    buffer1[i] = 0;
    buffer2[i] = 0;
    buffer3[i] = 0;
    buffer4[i] = 0;
    buffer5[i] = 0;
    buffer6[i] = 0;
    bufferRate[i] = 0;
  }

  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    std::cout << "Socket Creation Error!" << std::endl;
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(port);

  if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
    std::cout << "Invalid Address / Address Not Supported" << std::endl;
  }

  if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    std::cout << "Connection Failed" << std::endl;
  }

  int status = fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
  if (status == -1) {
    std::cout << "Failed to Make the Socket Non-Blocking" << std::endl;
  }

}


double VelocityController::MovingAverage(std::array<double, 200>& buffer, double input) {

  double filtered_input = 0.0;
  for (int i = 0; i < 200; i++) {
    filtered_input = filtered_input + buffer[i];
  }
  filtered_input = filtered_input / 200.0;
  for (int i = 0; i < 199; i++) {
    buffer[i] = buffer[i + 1];
  }
  buffer[199] = input;
  return filtered_input;

}


franka::CartesianVelocities VelocityController::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {

  time += period.toSec();

  std::array<double, 7> joint_position = robot_state.q;
  double send_rate = robot_state.control_command_success_rate;

  std::string state = "s,";
  for (int i = 0; i < 7; i++) {
    state.append(std::to_string(joint_position[i]));
    state.append(",");
  }
  char cstr[state.size() + 1];
  std::copy(state.begin(), state.end(), cstr);
  cstr[state.size()] = '\0';

  if (steps % 5 < 1) {
    valread = read(sock, buffer, 200);
    send(sock, cstr, strlen(cstr), 0);
    if (valread > 0) {
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
    } else {
      for (int i = 0; i < 6; i++) {
        human_input[i] = 0.000001;
      }
    }
  }


  double xdot1 = MovingAverage(buffer1, human_input[0]);
  double xdot2 = MovingAverage(buffer2, human_input[1]);
  double xdot3 = MovingAverage(buffer3, human_input[2]);
  double xdot4 = MovingAverage(buffer4, human_input[3]);
  double xdot5 = MovingAverage(buffer5, human_input[4]);
  double xdot6 = MovingAverage(buffer6, human_input[5]);
  double comm_rate = MovingAverage(bufferRate, send_rate);

  xdot = {{xdot1, xdot2, xdot3, xdot4, xdot5, xdot6}};
  franka::CartesianVelocities velocity(xdot);

  steps = steps + 1;
  if (steps % 200 < 1) {
    std::cout << "control_command_success_rate: " << comm_rate << std::endl;
    std::cout << "input buffer: " << buffer << std::endl;
  }
  return velocity;

}


int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << std::endl
              << "IP of robot" << std::endl
              << "Port for socket channel" << std::endl
              << "example IP is [172.16.0.2]." << std::endl
              << "example Port is [8080]." << std::endl;
    return -1;
  }

  try {

    franka::Robot robot(argv[1]);
    robot.automaticErrorRecovery();

    int port = std::stoi(argv[2]);
    VelocityController motion_generator(port);

    robot.setCollisionBehavior(
        {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
        {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
        {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
        {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    // robot.setCartesianImpedance({{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}});

    robot.control(motion_generator);//, franka::ControllerMode::kCartesianImpedance);

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;

}
