// Dylan Losey, June 27, 2019.
// script to check time synch with other robot


#include <iostream>
#include <chrono>


int main() {
  
  auto time_point = std::chrono::system_clock::now().time_since_epoch();
  long int time = (std::chrono::duration_cast< std::chrono::milliseconds >(time_point)).count();
  int time_10s = (time % 10000);
  while (time_10s) {
    time_point = std::chrono::system_clock::now().time_since_epoch();
    time = (std::chrono::duration_cast< std::chrono::milliseconds >(time_point)).count();
    time_10s = (time % 10000);
  }

  std::cout << "\nThe current time since the epoch is: " << time << std::endl;

  return 0;
}
