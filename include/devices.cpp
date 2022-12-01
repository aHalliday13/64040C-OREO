/**
 * \file devices.cpp
 * This is where we define all our devices. It isn't strictly neccessary to do this,
 * but it just saves space in main.cpp
 * \author aHalliday13
 */

#include "main.h"
using namespace pros;

// Driver's Controller
Controller master(E_CONTROLLER_MASTER);

Motor left_front(1,E_MOTOR_GEAR_GREEN,1);
Motor left_rear(9,E_MOTOR_GEAR_GREEN,1);
// Motor group for the left side of the robot (Where the flywheel represents the front of the robot)
Motor_Group left_drive({left_front,left_rear});

Motor right_front(2,E_MOTOR_GEAR_GREEN);
Motor right_rear(10,E_MOTOR_GEAR_GREEN);
// Motor group for the right side of the robot (Where the flywheel represents the front of the robot)
Motor_Group right_drive({right_front,right_rear});

Motor flywheel1(8,E_MOTOR_GEAR_BLUE,true);
Motor flywheel2(7,E_MOTOR_GEAR_BLUE);
// Both flywheel motors in a motor group
Motor_Group flywheel({flywheel1,flywheel2});

Motor intake(3,E_MOTOR_GEAR_GREEN,true);
Motor roller(4,E_MOTOR_GEAR_GREEN);
// Both intake motors
Motor_Group rollIntChain({intake,roller});

// Optical sensor pointing at the roller on the right side of robot (Where the flywheel represents the front of the robot)
Optical rollerOpR(5);
// Optical sensor pointing at the roller on the left side of robot (Where the flywheel represents the front of the robot)
Optical rollerOpL(6);

// Double acting pneumatic valve for sending discs into the flywheel
ADIPort indexer(1,E_ADI_DIGITAL_OUT);

// Inertial sensor on the center of rotation
Imu inertial(11);

// Expansion release trigger
ADIPort expansion(2,E_ADI_DIGITAL_OUT);