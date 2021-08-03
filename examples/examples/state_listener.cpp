// Dylan Losey, December 3, 2019.
// A joint position and end-effector state listener for the Panda


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


int main() {

  try {

    franka::Robot robot("172.16.0.2");

    int sock = 0;
    int valread;
    struct sockaddr_in serv_addr;
    char buffer[256] = {0};

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

    robot.read([&sock, &valread, &buffer](const franka::RobotState& robot_state) {

      std::array<double, 7> joint_position = robot_state.q;
      std::array<double, 16> ee_position = robot_state.O_T_EE;

      std::string joint_position_string;
      for (int i = 0; i < 7; i++) {
        joint_position_string.append(std::to_string(joint_position[i]));
        joint_position_string.append(",");
      }
      joint_position_string.append(std::to_string(ee_position[12]));
      joint_position_string.append(",");
      joint_position_string.append(std::to_string(ee_position[13]));
      joint_position_string.append(",");
      joint_position_string.append(std::to_string(ee_position[14]));
      char cstr[joint_position_string.size() + 1];
      std::copy(joint_position_string.begin(), joint_position_string.end(), cstr);
      cstr[joint_position_string.size()] = '\0';

      valread = read(sock, buffer, 256);
      send(sock, cstr, strlen(cstr), 0);
      return 1;

    });

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;

}

