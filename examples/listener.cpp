// Dylan Losey, July 1, 2019.
// Listener (using Forces)


#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>

#include <franka/exception.h>
#include <franka/robot.h>


std::ofstream recorder;


class PotentialFieldController {

  public:

    PotentialFieldController();
    franka::CartesianVelocities operator()(const franka::RobotState& robot_state, franka::Duration period);

  private:

    double maximum_speed = 0.05;
    double threshold_upper = 5.0;
    double threshold_lower = 4.0;
    double time_end = 20.0;

    std::array<double, 3> p_obs;
    std::array<double, 3> shift;
    double z_goal;
    double r_obs;
    double time;

    double distance(std::array<double, 3> x, std::array<double, 3> y);
    void writer(double time, double speaker, const franka::RobotState& robot_state);

};


PotentialFieldController::PotentialFieldController() {
  p_obs = {{0.2, 0.25, 0.3}};
  shift = {{0.0, 0.0, 0.0}};
  z_goal = 0.2;
  r_obs = 0.2;
  time = 0.0;
  recorder.open("test_listener.csv");
}


double PotentialFieldController::distance(std::array<double, 3> x, std::array<double, 3> y) {

  return sqrt(pow(x[0] - y[0],2.0) + pow(x[1] - y[1],2.0) + pow(x[2] - y[2],2.0));

}


void PotentialFieldController::writer(double time, double speaker, const franka::RobotState& robot_state) {

  recorder << time << ",";
  recorder << speaker << ",";

  std::array<double, 16> T = robot_state.O_T_EE;  
  recorder << T[12] << "," << T[13] << "," << T[14] << ",";

  std::array<double, 6> What = robot_state.O_F_ext_hat_K;
  recorder << What[0] << "," << What[1] << "," << What[2] << ",";
  recorder << What[3] << "," << What[4] << "," << What[5] << ",";

  std::array<double, 7> q = robot_state.q;
  recorder << q[0] << "," << q[1] << "," << q[2] << ",";
  recorder << q[3] << "," << q[4] << "," << q[5] << "," << q[6] << ",";

  
  recorder << "\n";

}


franka::CartesianVelocities PotentialFieldController::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {

  time += period.toSec();

  std::array<double, 6> What = robot_state.O_F_ext_hat_K;
  std::array<double, 3> Fhat = {{What[0], What[1], What[2]}};
  std::array<double, 16> T = robot_state.O_T_EE;
  std::array<double, 3> p = {{T[12], T[13], T[14]}};

  for (size_t i = 0; i < 3; i++) {
    if (std::abs(Fhat[i]) > threshold_upper) {
      if (Fhat[i] > 0) {
        shift[i] -= 0.0005;
      } else {
        shift[i] += 0.0005;
      }
    } else if (std::abs(Fhat[i]) < threshold_lower) {
      if (shift[i] > 0) {
        shift[i] -= 0.0001;
      } else if (shift[i] < 0) {
        shift[i] += 0.0001;
      }
    }
    if (std::abs(shift[i]) > 10.0*maximum_speed) {
      if (shift[i] > 0) {
        shift[i] = 10*maximum_speed;
      } else {
        shift[i] = -10*maximum_speed;
      }
    }
  }

  if (int(time*1000 + 0.5)%10 == 0) {
    writer(time, 0, robot_state);
  }

  if (time > time_end) {
    recorder.close();
    franka::CartesianVelocities xi1dot = {{0,0,0,0,0,0}};
    return franka::MotionFinished(xi1dot);
  }

  double dist2goal = std::abs(z_goal - p[2]);
  double magnitude_att;
  if (dist2goal < maximum_speed) {
    magnitude_att = 1.0;
  } else {
    magnitude_att = maximum_speed / dist2goal;
  }
  std::array<double, 3> att = {{0, 0, (z_goal-p[2])*magnitude_att}};

  double dist2obs = distance(p, p_obs);
  double magnitude_rep = 5.0*maximum_speed*(-0.5*tanh(10.0 * (dist2obs - r_obs)) + 0.5); 
  std::array<double, 3> rep = {{(p[0]-p_obs[0])*magnitude_rep, (p[1]-p_obs[1])*magnitude_rep, (p[2]-p_obs[2])*magnitude_rep}};

  double dx = att[0] + rep[0] + shift[0];
  double dy = att[1] + rep[1] + shift[1];
  double dz = att[2] + rep[2] + shift[2];
  dx *= tanh(2.0 * time);
  dy *= tanh(2.0 * time);
  dz *= tanh(2.0 * time);
  franka::CartesianVelocities xi1dot = {{dx,dy,dz,0,0,0}};
  return xi1dot;

}


bool synchronize_start() {

  auto time_point = std::chrono::system_clock::now().time_since_epoch();
  long int time = (std::chrono::duration_cast< std::chrono::milliseconds >(time_point)).count();
  int time_10s = (time % 10000);
  while (time_10s) {
    time_point = std::chrono::system_clock::now().time_since_epoch();
    time = (std::chrono::duration_cast< std::chrono::milliseconds >(time_point)).count();
    time_10s = (time % 10000);
  }
  return true;

}


int main() {

  //if (argc != 2) {
  //  std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
  //  return -1;
  //}

  try {

    franka::Robot robot("172.16.0.2");
    robot.automaticErrorRecovery();

    PotentialFieldController motion_generator;

    robot.setCollisionBehavior(
        {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0}}, {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0}},
        {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0}}, {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0}});
    robot.setCartesianImpedance(
	{{200.0, 200.0, 200.0, 1.0, 1.0, 1.0}});

    synchronize_start();
    robot.control(motion_generator, franka::ControllerMode::kCartesianImpedance, true, 25);

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;

}


