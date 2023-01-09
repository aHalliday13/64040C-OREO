#include "main.h"
#include "math.h"
#include "devices.cpp"
#include <vector>
using namespace pros;

/**
 * THEORY is when you know everything but nothing works.
 * PRACTICE is when everything works but no one knows why.
 * In this lab, theory and practice are combined: nothing works, and no one knows why.
 * - An absolute genius
 *
 * You turned the band saw into a spoon!
 * - What happens when you cut steel on the bandsaw
*/

// Our Left Y-Axis Curve Function. We use a define because it makes the source code more readable, and saves RAM at runtime instead of a variable.
#define CUBERTCTRL_LY (master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)/abs(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y))*(cbrt(abs(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) - 63.5) + 3.989556)*15.9148)
// TeleOp inversion, allows the driver to flip the front and back of the robot to make driving easier.
bool invertDrivetrainTeleOp=true;
// Autonomous selection variable
int routeSelection=0;
// Flywheel velocity variable so that we can return to the same velocity after a shot
float flywheelVelocity=0;

/**
 * Drive a set number of inches forward. Uses the flywheel as the front of the robot.
 * 
 * \param distance
 * 		  The distance in units to drive. This value really isn't acurate or precise,
 * 		  but it's close enough that you should be able to fine tune it with guesswork
 * 		  once you have a rough idea. Set this value positive to go forward, and 
 * 		  negative to go back.
 * \param maxVoltage
 * 		  The maximum voltage to send to the motors. This value should be between
 * 		  0mV and 12000mV.
 * \return Nothing
 * \author aHalliday13
 */
void driveIn(float distance, float maxVoltage=12000) {


	// Define all our constants
	const float kP=30;
	const float kI=.05;
	const float kD=500;
	const float unitsPerInch=57.9184632727;

	// 2500 units = 48.5 inches
	float error,lastError,integral,derivative,power,tweak=0;
	distance=distance*unitsPerInch;
	error=distance;
	std::vector<double> leftEncoderValues,rightEncoderValues;

	// Reset the encoders
	left_drive.tare_position();
	right_drive.tare_position();

	while(abs(error)>15) {
		leftEncoderValues=left_drive.get_positions();
		rightEncoderValues=right_drive.get_positions();
		
		// Calculate the error
		error=distance-(leftEncoderValues[0]+rightEncoderValues[0]+leftEncoderValues[1]+rightEncoderValues[1])/4;
		
		// Update the integral
		integral=integral+error;
		
		// Calculate the derivative
		derivative=error-lastError;
		lastError=error;
		
		// Calculate the power, apply power
		power=error*kP+integral*kI+derivative*kD;

		// Cap the power
		if(abs(power)>maxVoltage) {
			power= power<0 ? -maxVoltage : maxVoltage;
		}

		left_drive.move_voltage(power+tweak);
		right_drive.move_voltage(power-tweak);
		
		// Wait a set ammount of time to ensure that dT remains fairly constant
		pros::delay(15);
	}
	left_drive.brake();
	right_drive.brake();
}



/**
 * Turns the robot chasis a set number of degrees
 * Uses the flywheel as the front of the robot
 * 
 * \param rotation
 * 		  The number of degrees the robot should turn, bound [0,360), positive
 * 		  values turn clockwise, negative values turn anti-clockwise
 * 
 * \param velocity
 * 		  How fast to turn. For best results, go as slow as time will allow.
 * 		  This should always be a positive number. Bound (0,100] for E_MOTOR_GEARSET_36,
 * 		  (0,200] for E_MOTOR_GEARSET_18, and (0,600] for E_MOTOR_GEARSET_6	
 * \return Nothing
 * \author aHalliday13
 */
void driveTurn(float rotation, float velocity) {
	// Tare the inertial sensor
	inertial.tare();

	// Start turning in the appropriate direction
	left_drive.move_velocity(rotation<0 ? -1*velocity : velocity);
	right_drive.move_velocity(rotation>0 ? -1*velocity : velocity);

	// Wait for the inertial sensor to reach the desired rotation
	while (abs(inertial.get_rotation())<abs(rotation)) {}
	
	// Stop the motors
	left_drive.brake();
	right_drive.brake();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	left_front.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right_front.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	left_rear.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right_rear.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	
	intake1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	intake2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	flywheel1.set_brake_mode(E_MOTOR_BRAKE_COAST);
	flywheel2.set_brake_mode(E_MOTOR_BRAKE_COAST);
	
	inertial.reset();
	delay(3000);
	inertial.tare();



	// Autonomous selection menu
	while (true) {
		master.print(0, 0, "Route %d", routeSelection);
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
			routeSelection--;
			master.clear_line(0);
			master.print(0, 0, "Route %d", routeSelection);
		}
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
			routeSelection++;
			master.clear_line(0);
			master.print(0, 0, "Route %d", routeSelection);
		}
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)) {
			master.clear_line(0);
			break;
		}
	}
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
void competition_initialize() {
}

/**
 * Starts sitting on the left roller, shoots preloads into high goal, then
 * backs up, and spins the roller. Robot will then cross field to other roller,
 * picking up the discs in it's path (shooting them into goals) and then spinning the other roller.
 * \author aHalliday13
 */
void leftFullWP() {
	flywheel.move_velocity(560);
	rollerIntake.move_relative(-600,100);
	pros::delay(2000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	pros::delay(1200);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	flywheel.brake();
	driveTurn(-92,40);
	driveIn(-43);
	driveTurn(-27,40);
	driveIn(-40);
}

/**
 * Starts at the left half of the field, spins roller, shoots preloads, goes to
 * center of field, grabs discs on the way, and then fires them into the goal.
 * Confirmed working as of Jan 7, 2023
 * \author aHalliday13
 */
void leftHalfDiscs() {
	flywheel.move_velocity(5750);
	rollerIntake.move_relative(-600,100);
	delay(500);
	driveTurn(-2,30);
	pros::delay(3000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	pros::delay(1000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	driveTurn(-110,40);
	rollerIntake.move_velocity(600);
	flywheel.move_velocity(510);
	driveIn(-52,10000);
	driveTurn(92,30);
	pros::delay(1200);
	indexer.set_value(true);
	pros::delay(250);
	indexer.set_value(false);
	pros::delay(1200);
	indexer.set_value(true);
	pros::delay(250);
	indexer.set_value(false);
	pros::delay(1200);
	indexer.set_value(true);
	pros::delay(250);
	indexer.set_value(false);
}

/**
 * Starts at the right half of the field, shoots preloads, spins roller.
 * Confirmed working as of Jan 7, 2023
 * \author aHalliday13
 */
void rightHalfDiscs() {
	flywheel.move_velocity(600);
	pros::delay(3000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	pros::delay(1000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	flywheel.brake();
	driveTurn(-97,40);
	driveIn(-26);
	driveTurn(90,40);
	left_drive.move_relative(-300,100);
	right_drive.move_relative(-300,100);
	rollerIntake.move_relative(-600,600);
}

/**
 * Shoots discs into high goal (hopefully), spins roller to red, and then 
 * triggers the expansion release.
 * \author aHalliday13
 */
void skillsAuton() {
	flywheel.move_velocity(580);
	right_drive.move_voltage(-12000);
	rollerIntake.move_relative(-1200,100);
	pros::delay(3000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	right_drive.brake();
	pros::delay(2000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	driveTurn(-117,20);
	rollerIntake.move_velocity(600);
	flywheel.move_velocity(490);
	driveIn(-48,10000);
	driveTurn(92,20);
	pros::delay(1200);
	indexer.set_value(true);
	pros::delay(250);
	indexer.set_value(false);
	pros::delay(1200);
	indexer.set_value(true);
	pros::delay(250);
	indexer.set_value(false);
	pros::delay(1200);
	indexer.set_value(true);
	pros::delay(250);
	indexer.set_value(false);
	driveTurn(90,20);
	driveIn(-30);
	driveTurn(45,20);
	driveIn(-10);
	delay(1000);
	expansion1.set_value(true);
	delay(1000);
	driveTurn(-90,20);
	delay(1000);
	expansion2.set_value(true);
}

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
	if (routeSelection==1){
		leftHalfDiscs();
	}
	else if (routeSelection==2){
		rightHalfDiscs();
	}
	else if (routeSelection==3){
		leftFullWP();
	}
	else if (routeSelection==4){
		skillsAuton();
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
		left_drive = (CUBERTCTRL_LY*(invertDrivetrainTeleOp ? -1 : 1) + master.get_analog(E_CONTROLLER_ANALOG_LEFT_X));
		right_drive = (CUBERTCTRL_LY*(invertDrivetrainTeleOp ? -1 : 1) - master.get_analog(E_CONTROLLER_ANALOG_LEFT_X));

		// Invert the drivetrain if the driver requests it
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
			invertDrivetrainTeleOp= !invertDrivetrainTeleOp;
		}

		// Set the intake/ roller velocity
		rollerIntake = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

		// Set flywheel velocity
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
			// %100
			if (flywheel1.get_target_velocity()==600) {
				flywheel.brake();
			}
			else {
				flywheel.move_velocity(600);
			}
		}
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)){
			// %85
			if (flywheel1.get_target_velocity()==510) {
				flywheel.brake();
			}
			else {
				flywheel.move_velocity(510);
			}
		}
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)){
			// %75
			if (flywheel1.get_target_velocity()==450) {
				flywheel.brake();
			}
			else {
				flywheel.move_velocity(450);
			}
		}
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)){
			// %65
			if (flywheel1.get_target_velocity()==390) {
				flywheel.brake();
			}
			else {
				flywheel.move_velocity(390);
			}
		}

		// Fire the indexer
		indexer.set_value(master.get_digital(E_CONTROLLER_DIGITAL_X));

		// Trigger the expansion release
		expansion1.set_value(master.get_digital(E_CONTROLLER_DIGITAL_DOWN)||master.get_digital(E_CONTROLLER_DIGITAL_RIGHT));
		expansion2.set_value(master.get_digital(E_CONTROLLER_DIGITAL_DOWN)||master.get_digital(E_CONTROLLER_DIGITAL_LEFT));
	}
}
