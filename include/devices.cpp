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

// Null Cartridge Flywheel
Motor flywheel(16,E_MOTOR_GEARSET_INVALID,true);

// Roller and Intake
Motor rollerIntake1(18,E_MOTOR_GEAR_BLUE);
Motor rollerIntake2(13,E_MOTOR_GEAR_BLUE,true);
MotorGroup rollerIntake({rollerIntake1,rollerIntake2});

Motor indexer(15,E_MOTOR_GEAR_BLUE);

// Inertial sensor on the center of rotation
//Imu inertial();

// Pneumatic hookups
ADIPort flap(1,E_ADI_DIGITAL_OUT);
ADIPort expansion(2,E_ADI_DIGITAL_OUT);
ADIPort blocker(3,E_ADI_DIGITAL_OUT);

// Attributions for the code. Please don't remove
#define CREDITS "\033[1;5mMade by aHalliday13 of team 64040C. Do not distribute!\033[0m\n\n\033[30;107m█▀▀▀▀▀█ ▀█▀██▀  █ █▀▀▀▀▀█\033[0m\n\033[30;107m█ ███ █ █▄   █▀▀  █ ███ █\033[0m\n\033[30;107m█ ▀▀▀ █ ▀█  ▄▀█▀▀ █ ▀▀▀ █\033[0m\n\033[30;107m▀▀▀▀▀▀▀ ▀▄▀▄▀ ▀▄▀ ▀▀▀▀▀▀▀\033[0m\n\033[30;107m█▀▀▀▄ ▀▄▀▄ █▀ ▀ █▀ ▄▀▀▀▄▀\033[0m\n\033[30;107m▄█ ▄██▀ █  ▀▀▄█▄ █▀▀ ▄▄  \033[0m\n\033[30;107m ▀▄▄▄ ▀ ▀█▄ █▄█▄ ▀█▄█ ▀▀█\033[0m\n\033[30;107m▄▀▄▄▄ ▀ ▄▄█  ▀  ▀▀▄▄█ ▀▀▄\033[0m\n\033[30;107m  ▀▀ ▀▀ ████▄█  █▀▀▀█▀█▀█\033[0m\n\033[30;107m█▀▀▀▀▀█  ▀▄▄   ▄█ ▀ █ ▀██\033[0m\n\033[30;107m█ ███ █ ▄▀▀█ ▄▀ ▀██▀██▄██\033[0m\n\033[30;107m█ ▀▀▀ █ ██▀▀▄▄ █ ▀█ █ █▀ \033[0m\n\033[30;107m▀▀▀▀▀▀▀ ▀ ▀▀▀▀▀   ▀▀▀▀▀▀▀\033[0m\n"