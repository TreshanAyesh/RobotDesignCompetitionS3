#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 32
#define MAX_SPEED 10
// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  Motor *left_motor = robot->getMotor("left_motor");
  Motor *right_motor = robot->getMotor("right_motor");
  

  
  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  
  left_motor->setVelocity(0.0);
  right_motor->setVelocity(0.0);
  
 
  double left_speed = MAX_SPEED;
  double right_speed = MAX_SPEED;
  
  
  while (robot->step(TIME_STEP) != -1) {
    left_motor->setVelocity(left_speed);
    right_motor->setVelocity(right_speed);
  };

  

  delete robot;
  return 0;
}
