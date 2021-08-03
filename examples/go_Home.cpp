/**
 * go_Home.cpp
 *
 * A minimal script for sending the robot back to its home position. The Robot interpolates along each joint.
 * Dylan Losey, September 2020
 */

#include <iostream>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>


class JointPositionController {

  public:

    JointPositionController(const std::array<double, 7> q);
    franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

  private:

    double time = 0.0;
    double total_time;
    double time_inter;
    double alpha;
    std::array<double, 7> start;
    std::array<double, 7> home;
    std::array<double, 7> inter;
    std::array<double, 7> delta;

};

JointPositionController::JointPositionController(const std::array<double, 7> q) {
  home = q;
  total_time = 10.0;
}

franka::JointPositions JointPositionController::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time += period.toSec();

  if (time <= 0.0) {
    start = robot_state.q_d;
    for (int i = 0; i < 7; i++) {
      delta[i] = home[i] - start[i];
    }
    franka::JointPositions q(start);
    return q;

  } else if (time >= total_time) {
    franka::JointPositions q(home);
    return franka::MotionFinished(q);

  } else {
    time_inter = time / total_time;
    alpha = 10 * pow(time_inter,3.0) - 15 * pow(time_inter,4.0) + 6 * pow(time_inter,5.0);
    for (int i = 0; i < 7; i++) {
      inter[i] = start[i] + delta[i] * alpha;
    }
    franka::JointPositions q(inter);
    return q;
  }

}


int main(int argc, char** argv) {
  if (argc > 8) {
    std::cerr << "Usage: "
              << "<joint1> <joint2> <joint3> <joint4> <joint5> <joint6> <joint7>"
              << "joint values are in [rad]." << std::endl;
    return -1;
  }
  try {

    franka::Robot robot("172.16.0.3");
    robot.automaticErrorRecovery();

    robot.setCollisionBehavior(
        {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
        {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
        {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
        {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

    std::array<double, 7> home = {{0, -M_PI_4, 0, -3*M_PI_4, 0, M_PI_2, M_PI_4}};
    for (int i = 0; i < argc-1; i++) {
      home[i] = std::stod(argv[i + 1]);
    }
    JointPositionController motion_generator(home);

    robot.control(motion_generator);

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;

}
