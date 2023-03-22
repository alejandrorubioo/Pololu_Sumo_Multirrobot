// File:          my_controller.cpp
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
  DistanceSensor *ds_front = robot->getDistanceSensor("front_ds");
  DistanceSensor *ds_left = robot->getDistanceSensor("left_ds");
  DistanceSensor *ds_right = robot->getDistanceSensor("right_ds");
  
  //IR sensors: black(439) white(88)
  DistanceSensor *IR_dch = robot->getDistanceSensor("IR_1");
  DistanceSensor *IR_front = robot->getDistanceSensor("IR_2");
  DistanceSensor *IR_izq = robot->getDistanceSensor("IR_4");
  
  
  // Distance sensors activation
  ds_front->enable(TIME_STEP);
  ds_left->enable(TIME_STEP);
  ds_right->enable(TIME_STEP);
  
  IR_dch->enable(TIME_STEP);
  IR_front->enable(TIME_STEP);
  IR_izq->enable(TIME_STEP);
  
  //Definimos las variables de los sensores
  double DS_front_val;
  double DS_right_val;
  double DS_left_val;
    
  double IR_dch_val;
  double IR_front_val;
  double IR_izq_val;
  
  //Definimos los centineas
  bool laser_central = false;
  bool laser_izq = false;
  bool laser_der = false;
  
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  //while (robot->step(timeStep) != -1) {
  while (robot->step(TIME_STEP) != -1) {
    //Obtenemos los valores de las distancias
    DS_front_val = ds_front->getValue();
    DS_right_val = ds_right->getValue();
    DS_left_val = ds_left->getValue();  
    /*-------------------------------*/
    IR_dch_val = IR_dch->getValue();
    IR_front_val = IR_front->getValue();
    IR_izq_val = IR_izq->getValue();
    
    //Seteamos las velocidades al maximo permitido
    if(!laser_central && !laser_izq && !laser_der){
      front_left_motor->setVelocity(MAX_SPEED/2);
      front_right_motor->setVelocity(MAX_SPEED/2);
      back_left_motor->setVelocity(MAX_SPEED/2);
      back_right_motor->setVelocity(MAX_SPEED/2);
    }
    
    cout << "Distancia Central: " << DS_front_val << endl;
    cout << "Distancia Lateral Izquierda: " << DS_right_val << endl;
    cout << "Distancia Lateral Derecha: " << DS_left_val << endl;
      
    if( DS_front_val < 901){
      laser_central = true;
    }
    if( DS_left_val < 700){
      laser_izq = true;
    }
    if( DS_right_val < 700){
      laser_der = true;
    }
    
    if(laser_central){
      //while(DS_left_val > 1000){
        front_left_motor->setVelocity(-MAX_SPEED/4);
        front_right_motor->setVelocity(MAX_SPEED/4);
        back_left_motor->setVelocity(-MAX_SPEED/4);
        back_right_motor->setVelocity(MAX_SPEED/4);
      //}
      if(DS_right_val < 1000){
        laser_central = false;
      }
    }
    
    if(laser_der){
      //while(DS_left_val > 1000){
        front_left_motor->setVelocity(-MAX_SPEED/4);
        front_right_motor->setVelocity(MAX_SPEED/4);
        back_left_motor->setVelocity(-MAX_SPEED/4);
        back_right_motor->setVelocity(MAX_SPEED/4);
      //}
      if(DS_right_val < 1000){
        laser_der = false;
      }
    }
    
    if(laser_izq){
      //while(DS_left_val > 1000){
        front_left_motor->setVelocity(MAX_SPEED/4);
        front_right_motor->setVelocity(-MAX_SPEED/4);
        back_left_motor->setVelocity(MAX_SPEED/4);
        back_right_motor->setVelocity(-MAX_SPEED/4);
      //}
      if(DS_left_val < 1000){
        laser_izq = false;
      }
    }
    
    if(laser_der && laser_izq){
      //while(DS_left_val > 1000){
        front_left_motor->setVelocity(-MAX_SPEED/4);
        front_right_motor->setVelocity(MAX_SPEED/4);
        back_left_motor->setVelocity(-MAX_SPEED/4);
        back_right_motor->setVelocity(MAX_SPEED/4);
      //}
      if(DS_right_val < 1000){
        laser_der = false;
      }
    }
    
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
