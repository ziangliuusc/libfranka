// Dylan Losey, June 27, 2019.
// print the current robot state


#include <iostream>
#include <Eigen/Core>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>


int main() {

  try {
    franka::Robot robot("172.16.0.3");
    franka::Model model = robot.loadModel();

    franka::RobotState robot_state = robot.readOnce();

    Eigen::Map<const Eigen::Matrix<double, 4, 4> > T(robot_state.O_T_EE.data());
    std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
    Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
    // Eigen::Map<const Eigen::Matrix<double, 7, 1> > robot_state.q.data();
    // Eigen::VectorXd xdot = jacobian * qdot;


    std::cout << "Here is the current robot state: \n" << std::endl;
    std::cout << robot_state << std::endl;

    std::cout << "\nHere is the current end-effector pose: \n" << std::endl;
    std::cout << T << std::endl;

    std::cout << "\nHere is the robot joint value: \n" << std::endl;
    // std::cout << q << std::endl;


  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
