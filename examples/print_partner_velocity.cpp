// Dylan Losey, June 27, 2019.
// Show the difference between expected and actual
// End effector velocity (x - y - z).


#include <iostream>
#include <cmath>
#include <Eigen/Core>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>


class CartesianVelocityController {
  public:
    CartesianVelocityController(franka::Model& model);
    franka::CartesianVelocities operator()(const franka::RobotState& robot_state, franka::Duration period);
  private:
    franka::Model* modelPtr;
    double time = 0.0;
    double radius = 0.1;
    double frequency = 2*M_PI*0.05;
    double threshold = 0.04;
    std::array<double, 3> pdot_est;
};

CartesianVelocityController::CartesianVelocityController(franka::Model& model) {
  modelPtr = &model;
}

franka::CartesianVelocities CartesianVelocityController::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {

  time += period.toSec();

  std::array<double, 42> jacobian_array = modelPtr->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > qdot(robot_state.dq.data());
  Eigen::VectorXd xdot = jacobian * qdot;
  std::array<double, 6> xdot_c(robot_state.O_dP_EE_c);

  pdot_est = {{xdot[0], xdot[1], xdot[2]}};

  for (size_t i = 0; i < 3; i++) {
    if (std::abs(xdot[i] - xdot_c[i]) > threshold) {
      pdot_est[i] = 2*xdot[i] - xdot_c[i];
      std::cout << i << " axis = " << pdot_est[i] << " at time " << time << std::endl;
    }
  }
  
  double v_x = tanh(10.0 * time) * radius * std::cos(frequency*time) * frequency;
  double v_z = tanh(10.0 * time) * radius * -std::sin(frequency*time) * frequency;
  franka::CartesianVelocities xidot_c = {{v_x, v_x, v_z, 0.0, 0.0, 0.0}};
  return xidot_c;

}


int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    franka::Model model = robot.loadModel();
    robot.setLoad(0.2, {{0,0,0}}, {{1,0,0,0,1,0,0,0,1}});
    robot.automaticErrorRecovery();

    robot.setCollisionBehavior(
        {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0}}, {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0}},
        {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0}}, {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0}});
    robot.setCartesianImpedance(
	{{100.0, 100.0, 100.0, 1.0, 1.0, 1.0}});

    CartesianVelocityController motion_generator(model);
    robot.control(motion_generator, franka::ControllerMode::kCartesianImpedance);

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}


