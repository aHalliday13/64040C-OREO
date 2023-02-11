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
 * 
 * The field giveth and the field taketh
 * - Benny Boi finding ramdom parts on the field
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
void driveTurn(float rotation, float maxVoltage=12000) {
	// Define all our constants
	const float kP=175;
	const float kI=.5;
	const float kD=1000;
	const float startRotation=inertial.get_rotation();

	// 2500 units = 48.5 inches
	float error,lastError,integral,derivative,power;
	error=rotation;

	while(abs(error)>1.5) {
		// Calculate the error
		error=rotation-(inertial.get_rotation()-startRotation);
		
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

		left_drive.move_voltage(power);
		right_drive.move_voltage(-power);
		
		// Wait a set ammount of time to ensure that dT remains fairly constant
		pros::delay(15);
		// Print out the error, integral, derivative, and power	
		printf("Error: %f, Integral: %f, Derivative: %f, Power: %f\n",error,integral,derivative,power);
	}
	left_drive.brake();
	right_drive.brake();
}

void flywheelBangBang(float velocity) {
	while (true){
		if((flywheel1.get_actual_velocity()+flywheel2.get_actual_velocity())<(2*velocity)) {
			flywheel.move_voltage(flywheel1.get_voltage()+1000);
		}
		else if ((flywheel1.get_actual_velocity()+flywheel2.get_actual_velocity())>(2*velocity)){
			flywheel.move_voltage(flywheel1.get_voltage()-1000);
		}
	}
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
	while (inertial.is_calibrating());
	pros::delay(500);
	inertial.tare();
	pros::delay(500);

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
 * \author aHalliday13
 */
void leftHalfDiscs() {
	flywheel.move_velocity(580);
	pros::delay(4000);
	indexer.set_value(true);
	pros::delay(1000);
	flywheel.move_velocity(570);
	indexer.set_value(false);
	pros::delay(2000);
	indexer.set_value(true);
	pros::delay(1000);
	indexer.set_value(false);
	flywheel.move_velocity(490);

	left_drive.move_relative(-150,50);
	right_drive.move_relative(-150,50);
	pros::delay(200);
	rollerIntake.move_relative(-400,100);
	pros::delay(2000);
	left_drive.brake();
	right_drive.brake();

	left_drive.move_relative(250,50);
	right_drive.move_relative(250,50);
	pros::delay(700);
	driveTurn(-117);
	rollerIntake.move_velocity(200);
	driveIn(-40);
	driveTurn(100);
	rollerIntake.brake();
	pros::delay(1000);
	indexer.set_value(true);
	pros::delay(1000);
	indexer.set_value(false);
	pros::delay(1000);
	indexer.set_value(true);
	pros::delay(1000);
	indexer.set_value(false);
	pros::delay(1000);
	indexer.set_value(true);
	pros::delay(1000);
	indexer.set_value(false);
}

/**
 * Starts at the right half of the field, shoots preloads, spins roller.
 * \author aHalliday13
 */
void rightHalfDiscs() {
	flywheel.move_velocity(600);
	pros::delay(5000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	pros::delay(1000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	left_drive.move_relative(-420,50);
	right_drive.move_relative(420,50);
	pros::delay(2000);
	driveIn(15);
	driveTurn(-90);
	left_drive.move_velocity(-600);
	right_drive.move_velocity(-600);
	pros::delay(1000);
	rollerIntake.move_relative(-400,100);
	pros::delay(1000);
	left_drive.brake();
	right_drive.brake();
}

/**
 * Shoots discs into high goal (hopefully), spins roller to red, and then 
 * triggers the expansion release.
 * \author aHalliday13
 */
void skillsAuton() {
	left_drive.move_relative(-150,50);
	right_drive.move_relative(-150,50);
	pros::delay(200);
	rollerIntake.move_relative(-400,200);
	pros::delay(300);
	left_drive.move_relative(1500,50);
	right_drive.move_relative(1500,50);
	pros::delay(2000);
	rollerIntake.move_velocity(200);
	driveTurn(90);
	left_drive.move_relative(-1500,50);
	right_drive.move_relative(-1500,50);
	pros::delay(2000);
	left_drive.move_relative(1200,50);
	right_drive.move_relative(1200,50);
	pros::delay(2000);
	left_drive.move_relative(-200,50);
	right_drive.move_relative(200,50);
	pros::delay(2000);
	expansion1.set_value(true);
	left_drive.move_relative(-100,50);
	right_drive.move_relative(100,50);
	pros::delay(2000);
	expansion2.set_value(true);
	driveIn(-20);
	/*
	driveIn(110,9500);
	driveTurn(135);
	left_drive.move_relative(-1200,50);
	right_drive.move_relative(-1200,50);
	pros::delay(2000);
	left_drive.move_relative(1200,50);
	right_drive.move_relative(1200,50);
	pros::delay(2000);*/
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
	else if (routeSelection==-1){
		flywheelBangBang(390);
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
		// Print important stats to the controller screen
		master.print(0,0,"DT:%.0f FT:%.0f FV:%.0f     ",((left_front.get_temperature()+right_front.get_temperature()+left_rear.get_temperature()+right_rear.get_temperature())/4),((flywheel1.get_temperature()+flywheel2.get_temperature())/2),round(flywheel1.get_target_velocity()/6));
		
		// Drivetrain control functions
		left_drive = (CUBERTCTRL_LY*(invertDrivetrainTeleOp ? -1 : 1) + master.get_analog(E_CONTROLLER_ANALOG_LEFT_X));
		right_drive = (CUBERTCTRL_LY*(invertDrivetrainTeleOp ? -1 : 1) - master.get_analog(E_CONTROLLER_ANALOG_LEFT_X));

		// Invert the drivetrain if the driver requests it
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
			invertDrivetrainTeleOp= !invertDrivetrainTeleOp;
		}

		// Toggle the flap if the driver presses Y
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)){
			flap.set_value(!flap.get_value());
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
			// %68
			if (flywheel1.get_target_velocity()==410) {
				flywheel.brake();
			}
			else {
				flywheel.move_velocity(410);
			}
		}
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)){
			// %65
			if (flywheel1.get_target_velocity()==390) {
				flywheel.brake();
			}
			else {
				flywheel.move_velocity(390);
			}
		}
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)){
			// %66
			if (flywheel1.get_target_velocity()==400) {
				flywheel.brake();
			}
			else {
				flywheel.move_velocity(400);
			}
		}

		// Fire the indexer
		indexer.set_value(master.get_digital(E_CONTROLLER_DIGITAL_X));

		// Trigger the expansion release
		expansion1.set_value(master.get_digital(E_CONTROLLER_DIGITAL_DOWN)||master.get_digital(E_CONTROLLER_DIGITAL_LEFT));
		expansion2.set_value(master.get_digital(E_CONTROLLER_DIGITAL_DOWN)||master.get_digital(E_CONTROLLER_DIGITAL_RIGHT));
	}
}