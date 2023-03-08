// File:          Pololo_Controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 64
#define MAX_SPEED -6
// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  //int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  
  Motor *leftMotor = robot->getMotor("Motor2");
  Motor *rightMotor = robot->getMotor("Motor1");
  Motor *leftMotor_1 = robot->getMotor("Motor4");
  Motor *rightMotor_2 = robot->getMotor("Motor3");

  
 
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor_1->setPosition(INFINITY);
  rightMotor_2->setPosition(INFINITY);
  
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  leftMotor_1->setVelocity(0.0);
  rightMotor_2->setVelocity(0.0);
  
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  //while (robot->step(timeStep) != -1) {
  while (robot->step(TIME_STEP) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    
    leftMotor->setVelocity(MAX_SPEED);
    rightMotor->setVelocity(MAX_SPEED*0.7);
    leftMotor_1->setVelocity(MAX_SPEED);
    rightMotor_2->setVelocity(MAX_SPEED*0.7);
    
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
