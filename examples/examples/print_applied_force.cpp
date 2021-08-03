// Dylan Losey, July 1, 2019.
// Print measure force at EE


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
    double threshold = 0.0;
    std::array<double, 3> pdot_est;
};

CartesianVelocityController::CartesianVelocityController(franka::Model& model) {
  modelPtr = &model;
}

franka::CartesianVelocities CartesianVelocityController::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {

  time += period.toSec();

  std::array<double, 6> What = robot_state.O_F_ext_hat_K;
  std::array<double, 3> Fhat = {{What[0], What[1], What[2]}};

  for (size_t i = 2; i < 3; i++) {
    if (std::abs(Fhat[i]) > threshold) {
      std::cout << i << " axis = " << Fhat[i] << " at time " << time << std::endl;
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


