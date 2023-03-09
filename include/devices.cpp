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

Motor right_front(1,E_MOTOR_GEAR_GREEN);
Motor right_rear(2,E_MOTOR_GEAR_GREEN);
// Motor group for the right side of the robot (Where the flywheel represents the front of the robot)
Motor_Group right_drive({right_front,right_rear});

Motor left_front(9,E_MOTOR_GEAR_GREEN,true);
Motor left_rear(10,E_MOTOR_GEAR_GREEN,true);
// Motor group for the left side of the robot (Where the flywheel represents the front of the robot)
Motor_Group left_drive({left_front,left_rear});

Motor flywheel1(7,E_MOTOR_GEAR_BLUE,true);
Motor flywheel2(8,E_MOTOR_GEAR_BLUE);
// Both flywheel motors in a motor group
Motor_Group flywheel({flywheel1,flywheel2});

Motor intake1(4,E_MOTOR_GEAR_GREEN,true);
Motor intake2(5,E_MOTOR_GEAR_GREEN);
// Both intake motors
Motor_Group rollerIntake({intake1,intake2});

// Inertial sensor on the center of rotation
Imu inertial(15);

// Double acting pneumatic valve for sending discs into the flywheel
ADIPort indexer(1,E_ADI_DIGITAL_OUT);

// Expansion release triggers
ADIPort expansion1(2,E_ADI_DIGITAL_OUT);
ADIPort expansion2(3,E_ADI_DIGITAL_OUT);

// Front flap for forcing discs up at an angle after they leave the flywheel
ADIPort flap(8,E_ADI_DIGITAL_OUT);

// This bit of code may look completley useless, but it isn't. If you have morals, please don't remove it.
#define CREDITS "\033[1;5mMade by aHalliday13 of team 64040C. Do not distribute!\033[0m\n\n\033[30;107m█▀▀▀▀▀█ ▀█▀██▀  █ █▀▀▀▀▀█\033[0m\n\033[30;107m█ ███ █ █▄   █▀▀  █ ███ █\033[0m\n\033[30;107m█ ▀▀▀ █ ▀█  ▄▀█▀▀ █ ▀▀▀ █\033[0m\n\033[30;107m▀▀▀▀▀▀▀ ▀▄▀▄▀ ▀▄▀ ▀▀▀▀▀▀▀\033[0m\n\033[30;107m█▀▀▀▄ ▀▄▀▄ █▀ ▀ █▀ ▄▀▀▀▄▀\033[0m\n\033[30;107m▄█ ▄██▀ █  ▀▀▄█▄ █▀▀ ▄▄  \033[0m\n\033[30;107m ▀▄▄▄ ▀ ▀█▄ █▄█▄ ▀█▄█ ▀▀█\033[0m\n\033[30;107m▄▀▄▄▄ ▀ ▄▄█  ▀  ▀▀▄▄█ ▀▀▄\033[0m\n\033[30;107m  ▀▀ ▀▀ ████▄█  █▀▀▀█▀█▀█\033[0m\n\033[30;107m█▀▀▀▀▀█  ▀▄▄   ▄█ ▀ █ ▀██\033[0m\n\033[30;107m█ ███ █ ▄▀▀█ ▄▀ ▀██▀██▄██\033[0m\n\033[30;107m█ ▀▀▀ █ ██▀▀▄▄ █ ▀█ █ █▀ \033[0m\n\033[30;107m▀▀▀▀▀▀▀ ▀ ▀▀▀▀▀   ▀▀▀▀▀▀▀\033[0m\n"