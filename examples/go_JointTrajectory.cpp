// Dylan Losey, July 10, 2019.
// read in a trajectory from txt file
// smooth with a cubic spline


#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>

#include "spline.h"

#include <franka/exception.h>
#include <franka/robot.h>


double TO_START_TIME = 2.0;


class JointPositionTracker {
  public:
    JointPositionTracker(std::array<double,7> q_d, std::string filename);
    franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);
  private:
    double time = 0.0;
    double time_goal;
    std::array<double, 7> q_start;
    std::array<double, 7> q_curr;
    std::string trajfile;
    tk::spline s1, s2, s3, s4, s5, s6, s7;
    int getNumDoubles();
    std::vector<double> importTraj(int n_doubles);
    void constructTraj(int n_waypoints, std::vector<double> data);
};

JointPositionTracker::JointPositionTracker(std::array<double,7> q_d, std::string filename) {
  q_start = q_d;
  trajfile = filename;
  int n_doubles = getNumDoubles();
  int n_waypoints = (int)(n_doubles / 8.0 + 0.5);
  std::vector<double> data = importTraj(n_doubles);
  constructTraj(n_waypoints, data);
}

int JointPositionTracker::getNumDoubles() {
  std::ifstream inFile (trajfile);
  double x;
  int length = 0;
  while (inFile >> x) {
    length ++;
  }
  inFile.close();
  return length;
}

std::vector<double> JointPositionTracker::importTraj(int n_doubles) {
  std::ifstream inFile (trajfile);
  double x;
  int i = 0;
  std::vector<double> data(n_doubles);
  while (inFile >> x) {
    data[i] = x;
    i++;
  }
  inFile.close();
  return data;
}

void JointPositionTracker::constructTraj(int n_waypoints, std::vector<double> data) {
  std::vector<double> t(n_waypoints+4);
  std::vector<double> q1(n_waypoints+4);
  std::vector<double> q2(n_waypoints+4);
  std::vector<double> q3(n_waypoints+4);
  std::vector<double> q4(n_waypoints+4);
  std::vector<double> q5(n_waypoints+4);
  std::vector<double> q6(n_waypoints+4);
  std::vector<double> q7(n_waypoints+4);
  t[0] = 0.0;
  t[1] = 0.0 + 0.001;
  t[2] = TO_START_TIME - 0.001;
  q1[0] = q_start[0];
  q2[0] = q_start[1];
  q3[0] = q_start[2];
  q4[0] = q_start[3];
  q5[0] = q_start[4];
  q6[0] = q_start[5];
  q7[0] = q_start[6];
  q1[1] = q_start[0];
  q2[1] = q_start[1];
  q3[1] = q_start[2];
  q4[1] = q_start[3];
  q5[1] = q_start[4];
  q6[1] = q_start[5];
  q7[1] = q_start[6];
  q1[2] = data[1];
  q2[2] = data[2];
  q3[2] = data[3];
  q4[2] = data[4];
  q5[2] = data[5];
  q6[2] = data[6];
  q7[2] = data[7];
  for (int i = 0; i < n_waypoints; i++) {
    t[i+3] = data[i*8] + TO_START_TIME;
    q1[i+3] = data[i*8 + 1];
    q2[i+3] = data[i*8 + 2];
    q3[i+3] = data[i*8 + 3];
    q4[i+3] = data[i*8 + 4];
    q5[i+3] = data[i*8 + 5];
    q6[i+3] = data[i*8 + 6];
    q7[i+3] = data[i*8 + 7];
  }
  t[n_waypoints+3] = t[n_waypoints+2] + 0.001;
  q1[n_waypoints+3] = q1[n_waypoints+2];
  q2[n_waypoints+3] = q2[n_waypoints+2];
  q3[n_waypoints+3] = q3[n_waypoints+2];
  q4[n_waypoints+3] = q4[n_waypoints+2];
  q5[n_waypoints+3] = q5[n_waypoints+2];
  q6[n_waypoints+3] = q6[n_waypoints+2];
  q7[n_waypoints+3] = q7[n_waypoints+2];
  time_goal = t[n_waypoints+2];
  s1.set_points(t,q1);
  s2.set_points(t,q2);
  s3.set_points(t,q3);
  s4.set_points(t,q4);
  s5.set_points(t,q5);
  s6.set_points(t,q6);
  s7.set_points(t,q7);
}

franka::JointPositions JointPositionTracker::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time += period.toSec();

  if (time == 0.0) {
    q_start = robot_state.q_d;
    q_curr = q_start;
    franka::JointPositions q_c(q_curr);
    return q_c;

  } else if (time >= time_goal) {
    franka::JointPositions q_c(q_curr);
    return franka::MotionFinished(q_c);

  } else {
    q_curr[0] = s1(time);
    q_curr[1] = s2(time);
    q_curr[2] = s3(time);
    q_curr[3] = s4(time);
    q_curr[4] = s5(time);
    q_curr[5] = s6(time);
    q_curr[6] = s7(time);
    franka::JointPositions q_c(q_curr);
    return q_c;
  }

}


int main(int argc, char** argv) {

  try {
    franka::Robot robot("172.16.0.3");
    robot.automaticErrorRecovery();

    std::string filename = argv[1];

    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});
    robot.setJointImpedance(
	{{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    franka::RobotState robot_state = robot.readOnce();

    JointPositionTracker motion_generator(robot_state.q_d, filename);
    robot.control(motion_generator);

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
