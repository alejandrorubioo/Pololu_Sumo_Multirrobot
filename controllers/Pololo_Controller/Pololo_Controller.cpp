// File:          Pololo_Controller.cpp
// Date:
// Description:
// Author: 
// Modifications:

#include <stdio.h>

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 64
#define MAX_SPEED -10

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

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
  
  // Motors setup 
  Motor *front_left_motor = robot->getMotor("MOTOR_FRONT_LEFT");
  Motor *front_right_motor = robot->getMotor("MOTOR_FRONT_RIGHT");
  Motor *back_left_motor = robot->getMotor("MOTOR_BACK_LEFT");
  Motor *back_right_motor = robot->getMotor("MOTOR_BACK_RIGHT");

  // Motors initialization
  front_left_motor->setPosition(INFINITY);
  front_right_motor->setPosition(INFINITY);
  back_left_motor->setPosition(INFINITY);
  back_right_motor->setPosition(INFINITY);
  
  front_left_motor->setVelocity(0.0);
  front_right_motor->setVelocity(0.0);
  back_left_motor->setVelocity(0.0);
  back_right_motor->setVelocity(0.0);
    
  // Distance sensors initializations
  //DistanceSensor *leftDS = robot->getDistanceSensor()
  DistanceSensor *ds_front_right = robot->getDistanceSensor("front_right_ds");
  DistanceSensor *ds_front_left = robot->getDistanceSensor("front_left_ds");
  DistanceSensor *ds_left = robot->getDistanceSensor("left_ds");
  DistanceSensor *ds_right = robot->getDistanceSensor("right_ds");
  
  //IR sensors: black(439) white(88)
  DistanceSensor *ds_IR_0 = robot->getDistanceSensor("IR_0");
  DistanceSensor *ds_IR_1 = robot->getDistanceSensor("IR_1");
  DistanceSensor *ds_IR_2 = robot->getDistanceSensor("IR_2");
  DistanceSensor *ds_IR_3 = robot->getDistanceSensor("IR_3");
  DistanceSensor *ds_IR_4 = robot->getDistanceSensor("IR_4");
  DistanceSensor *ds_IR_5 = robot->getDistanceSensor("IR_5");
  
  
  // Distance sensors activation
  ds_front_right->enable(TIME_STEP);
  ds_front_left->enable(TIME_STEP);
  ds_left->enable(TIME_STEP);
  ds_right->enable(TIME_STEP);
  
  ds_IR_0->enable(TIME_STEP);
  ds_IR_1->enable(TIME_STEP);
  ds_IR_2->enable(TIME_STEP);
  ds_IR_3->enable(TIME_STEP);
  ds_IR_4->enable(TIME_STEP);
  ds_IR_5->enable(TIME_STEP);
  
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  //while (robot->step(timeStep) != -1) {
  while (robot->step(TIME_STEP) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.
    //double ds_front_right_val = ds_front_right->getValue();
    //double ds_front_left_val = ds_front_left->getValue();
    //double ds_left_val = ds_left->getValue();
    //double ds_right_val = ds_right->getValue();
    double ds_IR_val[6];
    ds_IR_val[0] = ds_IR_0->getValue();
    ds_IR_val[1] = ds_IR_1->getValue();
    ds_IR_val[2] = ds_IR_2->getValue();
    ds_IR_val[3] = ds_IR_3->getValue();
    ds_IR_val[4] = ds_IR_4->getValue();
    ds_IR_val[5] = ds_IR_5->getValue();
    
    /*
    cout << "Front_l distance " << ds_front_left_val 
         << "  Front_r distance " << ds_front_right_val 
         << "  Left distance " << ds_left_val 
         << "  Right distance " << ds_right_val << endl;
    */
    
    cout << "Front_l distance " << ds_IR_val[0] << endl;
    
    
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
     
  
    /*       
    front_left_motor->setVelocity(0.0);
    front_right_motor->setVelocity(0.0);
    back_left_motor->setVelocity(0.0);
    back_right_motor->setVelocity(0.0);
    */
 
    if(ds_IR_val[0] > 400){
    
      front_left_motor->setVelocity(MAX_SPEED);
      front_right_motor->setVelocity(MAX_SPEED);
      back_left_motor->setVelocity(MAX_SPEED);
      back_right_motor->setVelocity(MAX_SPEED);
    }
    else{
      
      front_left_motor->setVelocity(MAX_SPEED*-0.5);
      front_right_motor->setVelocity(MAX_SPEED*0.5);
      back_left_motor->setVelocity(MAX_SPEED*-0.5);
      back_right_motor->setVelocity(MAX_SPEED*0.5);
    } 
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
