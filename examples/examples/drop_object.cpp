// Dylan Losey, June 27, 2019.
// Drop an object


#include <iostream>

#include <franka/exception.h>
#include <franka/gripper.h>


int main() {

  try {
    franka::Gripper gripper("172.16.0.2");

    gripper.stop();
    std::cout << "i stopped grasping the object!" << std::endl;

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
