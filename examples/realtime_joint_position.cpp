#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <franka/exception.h>
#include <franka/robot.h>
// #include <franka/gripper.h>
#include <franka/model.h>
// #include <Eigen/Core>
/**
 * A minimal script for controlling the robot using joint velocity commands
 * the robot sends back position, velocity, torque, and jacobian
 * Dylan Losey, September 2020
 * Modified by Ziang Liu, August 2021
 */
class JointPositionController {
  public:
    JointPositionController(franka::Model& model, int port);
    franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);
  private:
    franka::Model* modelPtr;
    double time = 0.0;
    double time_start = 0.0;
    double time_goal = 0.0;
    double time_inter;
    double smooth_inter;
    int curr_buffer_index = -1;
    int curr_exec_index = -1;
    std::array<double, 7> q_start;
    // std::array<double, 7> q_goal;
    std::array<double, 7> q_inter;
    std::array<double, 7> delta_q;

    std::vector<std::vector<double> > buffer;
    //
    std::array<double, 100> buffer1;
    std::array<double, 100> buffer2;
    std::array<double, 100> buffer3;
    std::array<double, 100> buffer4;
    std::array<double, 100> buffer5;
    std::array<double, 100> buffer6;
    std::array<double, 100> buffer7;
    std::array<double, 100> bufferRate;
    std::array<double, 7> q;
    std::array<double, 7> qdot;
    std::array<double, 8> control_input;
    double MovingAverage(std::array<double, 100>& buffer, double input);
    int sock = 0;
    int valread;
    struct sockaddr_in serv_addr;
    char msg_buffer[200] = {0};
    int steps = 0;
};
JointPositionController::JointPositionController(franka::Model& model, int port) {
  // initialize buffer
  buffer.resize(8);
  for (int i = 0; i < 8; ++i) {
    buffer[i].resize(100);
  }

  modelPtr = &model;
  //
  // std::array<double, 7> gravity_array = model->gravity(initial_state);
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state.tau_J.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(gravity_array.data());
  // initial_tau_ext = initial_tau_measured - initial_gravity;

  for (int i = 0; i < 7; i++) {
    control_input[i] = 0.0;
  }
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 100; ++j) {
      buffer[i][j] = 0.0;
      bufferRate[j] = 0.0;
    }
  }
  for (int i = 0; i < 100; i++) {
    buffer1[i] = 0.0;
    buffer2[i] = 0.0;
    buffer3[i] = 0.0;
    buffer4[i] = 0.0;
    buffer5[i] = 0.0;
    buffer6[i] = 0.0;
    buffer7[i] = 0.0;
    bufferRate[i] = 0.0;
  }
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    std::cout << "Socket Creation Error!" << std::endl;
  }
  std::cout << "Created Socket" << std::endl;
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
  std::cout << "Socket Connection Succeeded!" << std::endl;
}

double JointPositionController::MovingAverage(std::array<double, 100>& buffer, double input) {
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

franka::JointPositions JointPositionController::operator()(const franka::RobotState& robot_state,franka::Duration period) {
  time += period.toSec();
  // std::cout << time << std::endl;
  // std::cout << "buffer: " << curr_buffer_index << std::endl;
  // if (time > 3) {
  //   return franka::MotionFinished(franka::JointPositions(robot_state.q));
  // }
  // publish current robot state to socket
  std::array<double, 7> joint_position = robot_state.q;
  std::array<double, 7> joint_velocity = robot_state.dq;
  std::array<double, 7> applied_torque = robot_state.tau_ext_hat_filtered;
  // std::array<double, 7> gravity = modelPtr->gravity(robot_state);

  std::array<double, 42> jacobian = modelPtr->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  double send_rate = robot_state.control_command_success_rate;
  std::string state = "s,";
  for (int i = 0; i < 7; i++) {
    state.append(std::to_string(joint_position[i]));
    state.append(",");
  }
  for (int i = 0; i < 7; i++) {
    state.append(std::to_string(joint_velocity[i]));
    state.append(",");
  }
  for (int i = 0; i < 7; i++) {
    state.append(std::to_string(applied_torque[i]));  // - gravity[i]));
    state.append(",");
  }
  for (int i = 0; i < 42; i++) {
    state.append(std::to_string(jacobian[i]));
    state.append(",");
  }
  char cstr[state.size() + 1];
  std::copy(state.begin(), state.end(), cstr);
  cstr[state.size()] = '\0';
  // std::cout << "Finished processing state" << std::endl;
  // std::cout << state << std::endl;
  // send(sock, cstr, strlen(cstr), 0);

  // read from socket

  if (steps % 5 < 1) {
    // std::cout << "Attempting to read message" << std::endl;
    valread = read(sock, msg_buffer, 200);
    // std::cout << "Finished reading message" << std::endl;
    send(sock, cstr, strlen(cstr), 0);
    if (valread > 0) {
      std::cout << "C++: read messge from socket" << std::endl;
      // std::cout << msg_buffer << std::endl;
      std::stringstream ss(msg_buffer);
      bool first = false;
      while (not first) {
        std::string substr;
        getline(ss, substr, ',');
        if ( substr[0] == 's') {
          first = true;
        }
      }
      for (int i = 0; i < 8; i++) {
        std::string substr;
        // std::cout << substr << std::endl;
        getline(ss, substr, ',');
        double term = std::stod(substr);
        // std::cout << term << " ";
        control_input[i] = term;
      }
      // std::cout << std::endl;
      // // only add to buffer if there is some socket message
      ++curr_buffer_index;
      for (int i = 0; i < 8; ++i) {
        buffer[i][curr_buffer_index] = control_input[i];
      }
      bufferRate[curr_buffer_index] = send_rate;
      // // std::cout << "CMD[:3]: " << control_input[0] << " " << control_input[1] << " " << control_input[2] << std::endl;
    } else {
      for (int i = 0; i < 7; i++) {
        control_input[i] = joint_position[i];
      }
      control_input[7] = 2.0; // time to execute
    }
  }
  // std::cout << "C++: finished reading/skipped" << std::endl;
  double comm_rate = MovingAverage(bufferRate, send_rate);
  steps = steps + 1;
  if (steps % 1000 < 1) {
    std::cout << "control_command_success_rate: " << comm_rate << std::endl;
    std::cout << "Time: " << time << std::endl;
    std::cout << "--------------------" << std::endl;
  }
  // execute joint position control
  if (curr_buffer_index == -1) {
    return franka::JointPositions(robot_state.q_d);
  } else {
    if (time == time_start) {
      // std::cout << "condition 1" << std::endl;
      q_start = robot_state.q_d;
      for (int i = 0; i < 7; i++) {
        delta_q[i] = buffer[i][curr_exec_index] - q_start[i];
      }
      franka::JointPositions q_c(q_start);
      return q_c;
      // return franka::MotionFinished(franka::JointPositions(robot_state.q));
    } else if (time >= time_goal) {
      // std::cout << "condition 2" << std::endl;
      if (curr_buffer_index <= curr_exec_index) {
        franka::JointPositions q_c(robot_state.q_d);
        return q_c;
        // return franka::MotionFinished(q_c);


      } else {
        ++curr_exec_index;
        time_start = time;
        time_goal = time_start + buffer[7][curr_exec_index];
        q_start = robot_state.q_d;
        for (int i = 0; i < 7; i++) {
          delta_q[i] = buffer[i][curr_exec_index] - q_start[i];
        }
        franka::JointPositions q_c(q_start);
        // std::cout << time_start << " " << time_goal << std::endl;
        return q_c;
      }
      // return franka::MotionFinished(franka::JointPositions(robot_state.q));
    } else {
      // std::cout << "condition 3" << std::endl;
      time_inter = (time - time_start) / (time_goal - time_start);
      // std::cout << "time_inter: " << time_inter << std::endl;
      smooth_inter = 10 * pow(time_inter,3.0) - 15 * pow(time_inter,4.0) + 6 * pow(time_inter,5.0);
      // std::cout << "smooth: : " << smooth_inter << std::endl;
      for (int i = 0; i < 7; i++) {
        q_inter[i] = q_start[i] + delta_q[i] * smooth_inter;
        // std::cout << q_inter[i] << " ";
        // std::cout << delta_q[i] << " ";
      }
      // std::cout << std::endl;
      franka::JointPositions q_c(q_inter);
      return q_c;
    }
  }
  // return franka::Joint(robot_state.dq);
  // time += period.toSec();
  // std::array<double, 7> joint_position = robot_state.q;
  // std::array<double, 7> joint_velocity = robot_state.dq;
  // std::array<double, 7> applied_torque = robot_state.tau_ext_hat_filtered;
  // // std::array<double, 7> gravity = modelPtr->gravity(robot_state);
  //
  // std::array<double, 42> jacobian = modelPtr->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  // double send_rate = robot_state.control_command_success_rate;
  // std::string state = "s,";
  // for (int i = 0; i < 7; i++) {
  //   state.append(std::to_string(joint_position[i]));
  //   state.append(",");
  // }
  // for (int i = 0; i < 7; i++) {
  //   state.append(std::to_string(joint_velocity[i]));
  //   state.append(",");
  // }
  // for (int i = 0; i < 7; i++) {
  //   state.append(std::to_string(applied_torque[i]));  // - gravity[i]));
  //   state.append(",");
  // }
  // for (int i = 0; i < 42; i++) {
  //   state.append(std::to_string(jacobian[i]));
  //   state.append(",");
  // }
  // char cstr[state.size() + 1];
  // std::copy(state.begin(), state.end(), cstr);
  // cstr[state.size()] = '\0';
  // std::cout << state << std::endl;
  // if (steps % 5 < 1) {
  //   valread = read(sock, msg_buffer, 200);
  //   send(sock, cstr, strlen(cstr), 0);
  //   if (valread > 0) {
  //     std::stringstream ss(msg_buffer);
  //     bool first = false;
  //     while (not first) {
  //       std::string substr;
  //       getline(ss, substr, ',');
  //       if ( substr[0] == 's') {
  //         first = true;
  //       }
  //     }
  //     for (int i = 0; i < 7; i++) {
  //       std::string substr;
  //       getline(ss, substr, ',');
  //       double term = std::stod(substr);
  //       control_input[i] = term;
  //     }
  //     // std::cout << "CMD[:3]: " << control_input[0] << " " << control_input[1] << " " << control_input[2] << std::endl;
  //
  //   } else {
  //     for (int i = 0; i < 7; i++) {
  //       control_input[i] = 0.0;
  //     }
  //   }
  // }
  // double qdot1 = MovingAverage(buffer1, control_input[0]);
  // double qdot2 = MovingAverage(buffer2, control_input[1]);
  // double qdot3 = MovingAverage(buffer3, control_input[2]);
  // double qdot4 = MovingAverage(buffer4, control_input[3]);
  // double qdot5 = MovingAverage(buffer5, control_input[4]);
  // double qdot6 = MovingAverage(buffer6, control_input[5]);
  // double qdot7 = MovingAverage(buffer7, control_input[6]);
  // double comm_rate = MovingAverage(bufferRate, send_rate);
  // // qdot = {{qdot1, qdot2, qdot3, qdot4, qdot5, qdot6, qdot7}};
  // // franka::JointVelocities velocity(qdot);
  // steps = steps + 1;
  // if (steps % 1000 < 1) {
  //   std::cout << "control_command_success_rate: " << comm_rate << std::endl;
  // }
  // return franka::JointPositions(robot_state.q);
}

int main() {
  int port = 8080;
  try {
    franka::Robot robot("172.16.0.3");
    franka::Model model = robot.loadModel();

    robot.automaticErrorRecovery();
    // robot.setCollisionBehavior(
    //     {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
    //     {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
    //     {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
    //     {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}});
    // robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});
    robot.setJointImpedance(
	{{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    std::cout << "Set robot joint positions" << std::endl;

    JointPositionController motion_generator(model, port);
    robot.control(motion_generator);
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}
