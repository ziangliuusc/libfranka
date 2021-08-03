// Dylan Losey, June 27, 2019.
// Hold an object


#include <iostream>

#include <franka/exception.h>
#include <franka/gripper.h>


int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: "
	      << "<width [in m]> <force [in N]>" << std::endl;
    return -1;
  }

  try {
    franka::Gripper gripper("172.16.0.2");
    double grasping_width = std::stod(argv[1]);
    double grasp_force = std::stod(argv[2]);
    double grasp_speed = 0.05;

    gripper.homing();

    gripper.grasp(grasping_width, grasp_speed, grasp_force);
    franka::GripperState gripper_state = gripper.readOnce();

    if(gripper_state.is_grasped) {
      std::cout << "i'm holding the object!" << std::endl;
    } else {
      std::cout << "let me refine my grasp..." << std::endl;
      gripper.grasp(grasping_width, grasp_speed, grasp_force);
      gripper_state = gripper.readOnce();
      if(gripper_state.is_grasped) {
        std::cout << "i'm holding the object!" << std::endl;
      } else {
        std::cout << "i failed to grasp the object!" << std::endl;
        gripper.stop();
      }
    }

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
