// Dylan Losey, July 3, 2019.
// Dynamic Role Switching


#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>

#include <franka/exception.h>
#include <franka/robot.h>


std::ofstream recorder;


class PotentialFieldController {

  public:

    PotentialFieldController(int leader, double period);
    franka::CartesianVelocities operator()(const franka::RobotState& robot_state, franka::Duration period);

  private:

    std::string name = "test.csv";
    double maximum_att_speed = 0.02;
    double maximum_rep_speed = 0.04;
    double threshold_upper = 3.5;
    double threshold_lower = 1.5;
    double time_end = 25.0;
    std::array<double, 3> p_obs = {{0.4, 0.25, 0.3}};
    double r_obs = 0.1;

    std::array<double, 3> shift;
    double z_goal;
    double time;
    int speaker_start;
    double switch_period;

    double distance(std::array<double, 3> x, std::array<double, 3> y);
    void shift_field(std::array<double, 3> Fhat);
    void writer(double time, double speaker, const franka::RobotState& robot_state);

};


PotentialFieldController::PotentialFieldController(int leader, double period) {
  recorder.open(name);
  shift = {{0.0, 0.0, 0.0}};
  z_goal = 0.2;
  time = 0.0;
  speaker_start = leader;
  switch_period = period;
}


double PotentialFieldController::distance(std::array<double, 3> x, std::array<double, 3> y) {

  return sqrt(pow(x[0] - y[0],2.0) + pow(x[1] - y[1],2.0) + pow(x[2] - y[2],2.0));

}


void PotentialFieldController::shift_field(std::array<double, 3> Fhat) {

  for (size_t i = 0; i < 3; i++) {
    if (std::abs(Fhat[i]) > threshold_upper) {
      if (Fhat[i] > 0) {
        shift[i] -= maximum_rep_speed / 100;
      } else {
        shift[i] += maximum_rep_speed / 100;
      }
    } else if (std::abs(Fhat[i]) < threshold_lower) {
      if (shift[i] > 0) {
        shift[i] -= maximum_rep_speed / 100;
      } else if (shift[i] < 0) {
        shift[i] += maximum_rep_speed / 100;
      }
    }
    if (std::abs(shift[i]) > maximum_rep_speed) {
      if (shift[i] > 0) {
        shift[i] = maximum_rep_speed;
      } else {
        shift[i] = -maximum_rep_speed;
      }
    }
  }

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
  double changes = floor(time / switch_period);
  int speaker = speaker_start - (int)(time / switch_period) % 2;
  double time_to_change = (changes + 1) * switch_period - time;

  std::array<double, 6> What = robot_state.O_F_ext_hat_K;
  std::array<double, 3> Fhat = {{What[0], What[1], What[2]}};
  std::array<double, 16> T = robot_state.O_T_EE;
  std::array<double, 3> p = {{T[12], T[13], T[14]}};

  if (speaker) {
    shift = {{0.0, 0.0, 0.0}};
  } else {
    if (time_to_change > 0.1) {
      shift_field(Fhat);
    } else {
      for (size_t i = 0; i < 3; i++) {
        if (shift[i] > 0) {
          shift[i] -= maximum_rep_speed / 100;
        } else {
          shift[i] += maximum_rep_speed / 100;
        }
      }
    }
  }

  if (int(time*1000 + 0.5)%10 == 0) {
    writer(time, speaker, robot_state);
  }

  if (time > time_end) {
    recorder.close();
    franka::CartesianVelocities xi1dot = {{0,0,0,0,0,0}};
    return franka::MotionFinished(xi1dot);
  }

  double dist2goal = std::abs(z_goal - p[2]);
  double magnitude_att;
  if (dist2goal < maximum_att_speed) {
    magnitude_att = 1.0;
  } else {
    magnitude_att = maximum_att_speed / dist2goal;
  }
  std::array<double, 3> att = {{0, 0, (z_goal-p[2])*magnitude_att}};

  double dist2obs = distance(p, p_obs);
  double magnitude_rep = maximum_rep_speed * (-0.5*tanh(10.0 * (dist2obs - r_obs)) + 0.5) / dist2obs; 
  std::array<double, 3> rep = {{(p[0]-p_obs[0])*magnitude_rep, (p[1]-p_obs[1])*magnitude_rep, 0}};

  std::cout << rep[0] << "  " << shift[0] << std::endl;

  double dx = att[0] + rep[0] + shift[0];
  double dy = att[1] + rep[1] + shift[1];
  double dz = att[2] + rep[2] + shift[2];
  dx *= tanh(5.0 * time);
  dy *= tanh(5.0 * time);
  dz *= tanh(5.0 * time);
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


int main(int argc, char** argv) {

  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <SpeakerStart><SwitchPeriod>" << std::endl;
    return -1;
  }

  try {

    franka::Robot robot("172.16.0.2");
    robot.automaticErrorRecovery();

    int speaker_start = std::stod(argv[1]);
    double switch_period = std::stod(argv[2]);

    PotentialFieldController motion_generator(speaker_start, switch_period);

    robot.setCollisionBehavior(
        {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0}}, {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0}},
        {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0}}, {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0}});
    robot.setCartesianImpedance(
	{{500.0, 500.0, 500.0, 1.0, 1.0, 1.0}});

    synchronize_start();
    robot.control(motion_generator, franka::ControllerMode::kCartesianImpedance, true, 25);

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;

}


