#include "main.h"
using namespace pros;

Controller master(E_CONTROLLER_MASTER);

Motor left_front(1,E_MOTOR_GEAR_GREEN);
Motor left_rear(9,E_MOTOR_GEAR_GREEN);
Motor_Group left_drive({left_front,left_rear});

Motor right_front(2,E_MOTOR_GEAR_GREEN,1);
Motor right_rear(10,E_MOTOR_GEAR_GREEN,1);
Motor_Group right_drive({right_front,right_rear});

Motor intake(3,E_MOTOR_GEAR_GREEN);

Imu inertial(9);

/**
 * Drive Inches Function:
 * Set the distance to a value that is either positive or negative, positive moves forward, negative moves backward
 * Unless otherwise stated, function will wait until it finishes execution to continue on to the next action, so plan accordingly.
 * Velocity should be POSITIVE
 */
void driveIN(float dist, float velocity, bool waitForComplete = true) {
	left_drive.move_relative(dist,velocity);
	right_drive.move_relative(dist,velocity);

	if (waitForComplete) {
		while(abs(left_front.get_target_velocity()) < 5 || abs(right_front.get_target_velocity()) < 5) {}
		while(abs(left_front.get_target_velocity()) > 5 || abs(right_front.get_target_velocity()) > 5) {}
	}
}

/**
 * Odometry turning function:
 * Angles should be bound (-180,180), with clockwise being positive and anti-clockwise being negative.
 * After turning using wheel odometry, the function will check if it is within ±5 degrees of the target, if it is not, it will *attempt* to correct itself.
 * Function will always wait to finish before moving on to next action.
 * Velocity should be POSITIVE!
 */
void odoTurn(float angle, float velocity) {
	inertial.tare_yaw();
	left_drive.move_relative(angle,velocity);
	right_drive.move_relative(-angle,velocity);

	while(abs(left_front.get_target_velocity()) < 5 || abs(right_front.get_target_velocity()) < 5) {}
	while(abs(left_front.get_target_velocity()) > 5 || abs(right_front.get_target_velocity()) > 5) {}

	if (inertial.get_yaw()+5>angle && inertial.get_yaw()-5<angle) {
		return;
	}
	/*
	else {
		if (inertial.get_yaw()+5<angle) {
			left_drive = 10;
			right_drive = -10;
			while(inertial.get_yaw()+5<angle) {}
		}
		else if (inertial.get_yaw()-5>angle) {
			left_drive = -10;
			right_drive = 10;
			while(inertial.get_yaw()-5>angle) {}
		}
		left_drive.brake();
		right_drive.brake();
	}*/
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	left_front.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_front.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	left_rear.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_rear.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	inertial.reset();
	inertial.tare();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		left_drive = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) - master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
		right_drive = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) + master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
	}
}
