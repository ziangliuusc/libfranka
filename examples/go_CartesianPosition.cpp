// Dylan Losey, June 25, 2019.
// go to a specific xyz position --
// Using a minimum jerk trajectory!


#include <iostream>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>


class CartesianVelocityController {
  public:
    CartesianVelocityController();
    CartesianVelocityController(const std::array<double, 3> p);
    CartesianVelocityController(const std::array<double, 3> p, const double t);
    franka::CartesianVelocities operator()(const franka::RobotState& robot_state, franka::Duration period);
  private:
    double time = 0.0;
    double time_goal;
    double time_inter;
    double smooth_inter;
    std::array<double, 3> p_start;
    std::array<double, 3> p_goal;
    std::array<double, 3> pdot_inter;
    std::array<double, 3> delta_p;
};

CartesianVelocityController::CartesianVelocityController() {
  p_goal = {{0.3, 0.0, 0.5}};
  time_goal = 5.0;
}

CartesianVelocityController::CartesianVelocityController(const std::array<double, 3> p) {
  p_goal = p;
  time_goal = 5.0;
}

CartesianVelocityController::CartesianVelocityController(const std::array<double, 3> p, const double t) {
  p_goal = p;
  time_goal = t;
}

franka::CartesianVelocities CartesianVelocityController::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time += period.toSec();

  if (time == 0.0) {
    std::array<double, 16> T_start = robot_state.O_T_EE;
    p_start = {{T_start[12], T_start[13], T_start[14]}};
    for (int i = 0; i < 3; i++) {
      delta_p[i] = p_goal[i] - p_start[i];
    }
    franka::CartesianVelocities xidot_c = {{0,0,0,0,0,0}};
    return xidot_c;

  } else if (time >= time_goal) {
    franka::CartesianVelocities xidot_c = {{0,0,0,0,0,0}};
    return franka::MotionFinished(xidot_c);

  } else {
    time_inter = time / time_goal;
    smooth_inter = 30*pow(time_inter,2.0) - 60*pow(time_inter,3.0) + 30.0*pow(time_inter,4.0);
    for (int i = 0; i < 3; i++) {
      pdot_inter[i] = 1/time_goal * delta_p[i] * smooth_inter;
    }
    franka::CartesianVelocities xidot_c = {{pdot_inter[0],pdot_inter[1],pdot_inter[2],0,0,0}};
    return xidot_c;
  } 

}


int main(int argc, char** argv) {
  if (argc != 1 && argc != 4 && argc != 5) {
    std::cerr << "Usage: "
              << "<x> <y> <z>"
              << "<motion-time>" << std::endl
              << "x, y, z are in [m]." << std::endl
              << "motion-time is in [sec]." << std::endl;
    return -1;
  }

  try {
    franka::Robot robot("172.16.0.3");
    robot.automaticErrorRecovery();

    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});
    robot.setCartesianImpedance(
	{{200, 200, 200, 1, 1, 1}});

    CartesianVelocityController motion_generator;
    std::array<double, 3> p_goal;
    double time_goal;

    if (argc >= 4) {
      for (size_t i = 0; i < 3; i++) {
        p_goal[i] = std::stod(argv[i + 1]);
      }
      CartesianVelocityController motion_generator_g(p_goal);
      motion_generator = motion_generator_g;

      if (argc == 5) {
        time_goal = std::stod(argv[4]);
        CartesianVelocityController motion_generator_gt(p_goal, time_goal);
        motion_generator = motion_generator_gt;
      }
    }

    robot.control(motion_generator, franka::ControllerMode::kCartesianImpedance);

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}


