#include "main.h"
#include "ARMS/config.h"
#include "devices.cpp"
#include <cmath>

// Left Y-Axis Curve Function
#define CUBERTCTRL_LY (master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)/abs(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y))*(cbrt(abs(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) - 63.5) + 3.989556)*15.9148)
// Left X-Axis Curve Function
#define EXPLOGCTRL_LX (master.get_analog(E_CONTROLLER_ANALOG_LEFT_X)/abs(master.get_analog(E_CONTROLLER_ANALOG_LEFT_X))*((pow(1.0389441558648,abs(master.get_analog(E_CONTROLLER_ANALOG_LEFT_X)))-1)+10*log10(abs(master.get_analog(E_CONTROLLER_ANALOG_LEFT_X))+1)))

inline __attribute__((always_inline)) void rightAuton() {
	flywheel.move_velocity(185);
	arms::chassis::move(-17,arms::REVERSE);

	pros::delay(2500);
	//was n900
	indexer.move_relative(400,600);

	pros::delay(500);
	rollerIntake.move_relative(5000,600);

	pros::delay(1500);
	//was800
	indexer.move_relative(400,600);

	pros::delay(500);

	arms::chassis::turn(55,arms::RELATIVE);
	arms::chassis::move(22);
	arms::chassis::turn(-45,arms::RELATIVE);
	arms::chassis::move(5,arms::THRU);
	rollerIntake.move_relative(-150,600);
}

inline __attribute__((always_inline)) void leftAuton() {
	int idleTime=0;
	flywheel.move_velocity(200);
	arms::chassis::move(-6,arms::REVERSE);

	pros::delay(2500);
	while(flywheel.get_actual_velocity()<200) {
		idleTime++;
		pros::delay(1);
		if (idleTime>2000) {
			goto returntostart;
		}
	}
	indexer.move_relative(170,600);

	pros::delay(1000);
	idleTime=0;
	while(flywheel.get_actual_velocity()<200) {
		idleTime++;
		pros::delay(1);
		if (idleTime>2000) {
			goto returntostart;
		}
	}
	rollerIntake.move_relative(5000,600);

	pros::delay(1500);
	returntostart:
	indexer.move_relative(170,600);
	pros::delay(500);
	arms::chassis::move(9);
	rollerIntake.move_relative(-170,600);	
}

inline __attribute__((always_inline)) void giveUp() {
	flywheel.move_voltage(4000);
	pros::delay(1000);
	indexer.move_voltage(12000);
	pros::delay(1000);
	indexer.brake();
	flywheel.brake();
	arms::chassis::turn(-90,40,arms::RELATIVE);
	rollerIntake.move_voltage(12000);
	arms::chassis::move(40,40);
	rollerIntake.brake();
	flywheel.move_voltage(4000);
	arms::chassis::turn(91,arms::RELATIVE);
	arms::chassis::move(-36,arms::REVERSE);
	arms::chassis::turn(-50,arms::RELATIVE);
	arms::chassis::move(-9,arms::REVERSE | arms::ASYNC);
	indexer.move_voltage(12000);
	pros::delay(1000);
	indexer.brake();
	arms::chassis::move(10);
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	arms::init();
	printf(CREDITS);
	arms::chassis::leftMotors.get()->set_brake_modes(E_MOTOR_BRAKE_COAST);
	arms::chassis::rightMotors.get()->set_brake_modes(E_MOTOR_BRAKE_COAST);
	indexer.set_brake_mode(E_MOTOR_BRAKE_HOLD);
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
void autonomous() {
	switch (arms::selector::auton) {
		case 3:
			giveUp();
			break;
		case 2:
			rightAuton();
			break;
		case 1:
			leftAuton();
			break;
		case 0:
			master.print(0,0,"404: skill issue     ");
			break;
		case -1:
			leftAuton();
			break;
		case -2:
			rightAuton();
			break;
		case -3:
			giveUp();
			break;
	}
}

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
		// Drivetrain control functions
		arms::chassis::arcade(CUBERTCTRL_LY,EXPLOGCTRL_LX);

		// Fire the indexer
		if (master.get_digital(E_CONTROLLER_DIGITAL_X)){
			indexer.move_velocity(600);
		}
		else {
			indexer.brake();
		}

		// Run the intake
		rollerIntake.move(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));

		// Set flywheel velocity
		// Speeds Zach Wants (mV): 9900 (170), 10800 (189) (flaps)
		if(master.get_digital(E_CONTROLLER_DIGITAL_UP)){
			flywheel.move_velocity(flywheel.get_actual_velocity()+1);
			pros::delay(1);
		}
		else if(master.get_digital(E_CONTROLLER_DIGITAL_DOWN)){
			flywheel.move_velocity(flywheel.get_actual_velocity()-1);
			pros::delay(1);
		}

		// 170
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
			if (flywheel.get_target_velocity()==170) {
				flywheel.brake();
			}
			else {
				flywheel.move_velocity(170);
			}
		}
		//184
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) {
			if (flywheel.get_target_velocity()==180) {
				flywheel.brake();
			}
			else {
				flywheel.move_velocity(180);
			}
		}
		//reverse
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
			if (flywheel.get_target_velocity()==-200) {
				flywheel.brake();
			}
			else {
				flywheel.move_velocity(-200);
			}
		}
		//stop
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
			flywheel.brake();
		}

		// Set the flap
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)){
			flap.set_value(!flap.get_value());
			if (flap.get_value()){
				flywheel.move_velocity(170);
			}
			else {
				flywheel.move_velocity(180);
			}
		}

		master.print(0,0,"%d      ",flywheel.get_target_velocity());

		// Deploy Expansion
		expansion.set_value(master.get_digital(E_CONTROLLER_DIGITAL_RIGHT));

		// Deploy expansion blocker
		blocker.set_value(master.get_digital(E_CONTROLLER_DIGITAL_LEFT));
	}
}
