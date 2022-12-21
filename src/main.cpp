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
 * \param kP 
 * 		  The proportional constant for the PID loop. This should be a positive 
 * 		  number. This will be assigned a default value. If you need to change 
 * 		  this at any point, you should change the default, not the value passed
 * 		  to the function.
 * \param kI
 * 		  The integral constant for the PID loop. This should be a positive 
 * 		  number. This will be assigned a default value. If you need to change 
 * 		  this at any point, you should change the default, not the value passed
 * 		  to the function.
 * \param kD
 * 		  The derivative constant for the PID loop. This should be a positive 
 * 		  number. This will be assigned a default value. If you need to change 
 * 		  this at any point, you should change the default, not the value passed
 * 		  to the function.
 * \param unitsPerInch
 * 		  The number of encoder units per inch of travel. This value is specific
 * 		  to each robot, and should be set to a default value. If you need to change
 * 		  this at any point, you should change the default, not the value passed to
 * 		  the function.
 * \return Nothing
 * \author aHalliday13
 */
void driveIn(float distance, float maxVoltage=12000, float kP=30, float kI=.05, float kD=500, float unitsPerInch=51.546392) {
	// 2500 units = 48.5 inches
	float error,lastError,integral,derivative,power,tweak;
	distance=distance*unitsPerInch;
	error=distance;
	std::vector<double> leftEncoderValues,rightEncoderValues;

	// Reset the encoders
	left_drive.tare_position();
	right_drive.tare_position();

	while(error>5) {
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
		if (leftEncoderValues[1]>rightEncoderValues[1]){
			tweak--;
		}
		if(leftEncoderValues[1]<rightEncoderValues[1]){
			tweak++;
		}
		else {
			tweak=0;
		}

		// Cap the power
		if(power>maxVoltage) {
			power=maxVoltage;
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
	
	inertial.reset(true);
	inertial.tare();

	rollerOpL.set_led_pwm(100);
	rollerOpR.set_led_pwm(100);

	/*
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
	}*/
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
 * Starts sitting next to the left roller, shoots preloads into high goal, then
 * backs up, and spins the roller to red.
 * \author aHalliday13
 */
void redLeftHalfWP() {
	flywheel.move_velocity(550);
	pros::delay(3000);
	for (int i=0;i<2;i++) {
		indexer.set_value(true);
		pros::delay(1000);
		indexer.set_value(false);
		pros::delay(500);
	}
	flywheel.brake();

	driveTurn(10,75);
	pros::delay(1000);

	left_drive=-100;
	right_drive=-100;
	pros::delay(1000);

	rollerIntake.move_relative(600,100);
}

/**
 * Starts sitting next to the left roller, shoots preloads into high goal, then
 * backs up, and spins the roller to blue.
 * \author aHalliday13
 */
void blueLeftHalfWP() {
	flywheel.move_velocity(550);
	pros::delay(3000);
	for (int i=0;i<2;i++) {
		indexer.set_value(true);
		pros::delay(1000);
		indexer.set_value(false);
		pros::delay(500);
	}
	flywheel.brake();

	driveTurn(10,75);
	pros::delay(1000);

	left_drive=-100;
	right_drive=-100;
	pros::delay(1000);
	left_drive.brake();
	right_drive.brake();

	rollerIntake=70;
	
	while (rollerOpR.get_hue() > 100){} // Rotate until red
	while (rollerOpR.get_hue() < 100){} // Rotate until blue
	
	rollerIntake.brake();
}

/**
 * Starts sitting on the left roller, shoots preloads into high goal, then
 * backs up, and spins the roller to red. Robot will then cross field to other roller,
 * picking up the discs in it's path (shooting them into goals) and then spinning the other roller to red.
 * \author aHalliday13
 */
void redLeftFullWP() {
	flywheel.move_velocity(555);
	left_drive=-100;
	right_drive=-100;
	pros::delay(500);
	rollerIntake.move_relative(600,100);
	left_drive.brake();
	right_drive.brake();

	driveIn(3);
	driveTurn(-2,50);
	driveIn(3);

	while(abs(flywheel1.get_actual_velocity())<550){}
	indexer.set_value(true);
	pros::delay(1000);
	indexer.set_value(false);
	pros::delay(500);

	while(abs(flywheel1.get_actual_velocity())<550){}
	indexer.set_value(true);
	pros::delay(1000);
	indexer.set_value(false);
	pros::delay(500);

	flywheel.brake();
}

/**
 * Starts sitting on the left roller, shoots preloads into high goal, then
 * backs up, and spins the roller to blue. Robot will then cross field to other roller,
 * picking up the three discs in it's path and then spinning the other roller to blue.
 * \author aHalliday13
 */
void blueLeftFullWP() {
	
}

/**
 * Starts at the left half of the field, spins roller to red, shoots preloads, goes to
 * center of field, grabs discs on the way, and then fires them into the goal.
 * \author aHalliday13
 */
void redLeftHalfDiscs() {
	driveIn(-2,100);
	rollerIntake=70;
	flywheel.move_velocity(550);
	while (rollerOpL.get_hue() < 100){} // Rotate until blue
	while (rollerOpL.get_hue() > 100){} // Rotate until red
	rollerIntake.brake();

	driveIn(3,50);
	driveTurn(-5,10);
    driveIn(3,50);
	pros::delay(500);

	while (flywheel1.get_actual_velocity()<550) {}
	indexer.set_value(true);
	pros::delay(1000);
	flywheel.move_velocity(550);
	indexer.set_value(false);
	pros::delay(500);

	while (flywheel1.get_actual_velocity()<550) {}
	indexer.set_value(true);
	pros::delay(1000);
	indexer.set_value(false);
	pros::delay(500);

	flywheel.brake();

	driveTurn(-100,50);
	rollerIntake=127;
	driveIn(-36,100);
	driveIn(-14,50);
	
	driveTurn(60,90);
	flywheel.move_velocity(500);
	while (flywheel1.get_actual_velocity()<500) {}
	for(int i=0;i<3;i++) {
		indexer.set_value(true);
		pros::delay(1000);
		indexer.set_value(false);
		pros::delay(500);
	}
}

/**
 * Starts at the left half of the field, spins roller to blue, shoots preloads, goes to
 * center of field, grabs discs on the way, and then fires them into the goal.
 * \author aHalliday13
 */
void blueLeftHalfDiscs() {
	driveIn(-2,100);
	rollerIntake=70;
	flywheel.move_velocity(550);
	while (rollerOpL.get_hue() > 100){} // Rotate until red
	while (rollerOpL.get_hue() < 100){} // Rotate until blue
	rollerIntake.brake();

	driveIn(3,50);
	driveTurn(-5,10);
    driveIn(3,50);
	pros::delay(500);

	while (flywheel1.get_actual_velocity()<550) {}
	indexer.set_value(true);
	pros::delay(1000);
	flywheel.move_velocity(550);
	indexer.set_value(false);
	pros::delay(500);

	while (flywheel1.get_actual_velocity()<550) {}
	indexer.set_value(true);
	pros::delay(1000);
	indexer.set_value(false);
	pros::delay(500);

	flywheel.brake();

	driveTurn(-100,50);
	rollerIntake=127;
	driveIn(-36,100);
	driveIn(-14,50);
	
	driveTurn(60,90);
	flywheel.move_velocity(500);
	while (flywheel1.get_actual_velocity()<500) {}
	for(int i=0;i<3;i++) {
		indexer.set_value(true);
		pros::delay(1000);
		indexer.set_value(false);
		pros::delay(500);
	}
}

/**
 * Starts at the right half of the field, shoots preloads, spins roller to red.
 * \author aHalliday13
 */
void redRightHalfDiscs() {
	flywheel.move_velocity(550);
	driveIn(14,30);
	while (flywheel1.get_actual_velocity()<550) {}
	for(int i=0;i<2;i++) {
		indexer.set_value(true);
		pros::delay(1000);
		indexer.set_value(false);
		pros::delay(500);
	}
	flywheel.brake();
	driveTurn(110,30);
	rollerIntake=127;
	driveIn(-40,100);
	driveTurn(-85,30);
	rollerIntake.brake();
	flywheel.move_velocity(500);
	while (flywheel1.get_actual_velocity()<500) {}
	for(int i=0;i<2;i++) {
		indexer.set_value(true);
		pros::delay(1000);
		indexer.set_value(false);
		pros::delay(500);
	}
	flywheel.brake();
	rollerIntake=127;
	driveTurn(-77,30);
	driveIn(-60,150);
	rollerIntake.brake();
	right_drive=-127;
	pros::delay(500);
	left_drive=-127;
	rollerIntake.move_voltage(7000);
	while (rollerOpR.get_hue() < 100){} // Rotate until blue
	while (rollerOpR.get_hue() > 100){} // Rotate until red
	rollerIntake.brake();
}

/**
 * Starts at the right half of the field, shoots preloads, spins roller to blue.
 * \author aHalliday13
 */
void blueRightHalfDiscs() {
	flywheel.move_velocity(550);
	driveIn(14,30);
	while (flywheel1.get_actual_velocity()<550) {}
	for(int i=0;i<2;i++) {
		indexer.set_value(true);
		pros::delay(1000);
		indexer.set_value(false);
		pros::delay(500);
	}
	flywheel.brake();
	driveTurn(110,30);
	rollerIntake=127;
	driveIn(-40,100);
	driveTurn(-90,30);
	rollerIntake.brake();
	flywheel.move_velocity(500);
	while (flywheel1.get_actual_velocity()<500) {}
	for(int i=0;i<2;i++) {
		indexer.set_value(true);
		pros::delay(1000);
		indexer.set_value(false);
		pros::delay(500);
	}
	flywheel.brake();
	rollerIntake=127;
	driveTurn(-77,30);
	driveIn(-60,150);
	right_drive=-127;
	pros::delay(500);
	left_drive=-127;
	while (rollerOpR.get_hue() > 100){} // Rotate until red
	while (rollerOpR.get_hue() < 100){} // Rotate until blue
	rollerIntake.brake();
}

/**
 * Shoots discs into high goal (hopefully), spins roller to red, and then 
 * triggers the expansion release.
 * \author aHalliday13
 */
void skillsAuton() {
	flywheel.move_velocity(550);
	pros::delay(3000);
	for (int i=0;i<2;i++) {
		indexer.set_value(true);
		pros::delay(1000);
		indexer.set_value(false);
		pros::delay(500);
	}
	flywheel.brake();

	driveTurn(10,75);
	pros::delay(1000);

	left_drive=-100;
	right_drive=-100;
	pros::delay(1000);
	left_drive.brake();
	right_drive.brake();

	driveIn(5,50);
	driveTurn(45,90);
	expansion1.set_value(true);
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
	return;
	// Yes, I know it's bad practice to call functions from within a function, but it works, and this is unfortunatley the best way to organize it.
	if (routeSelection==-1){
		skillsAuton();
	}
	else if (routeSelection==0) {
		redLeftHalfDiscs();
	} 
	else if (routeSelection==1) {
		blueLeftHalfDiscs();
	} 
	else if (routeSelection==2) {
		redRightHalfDiscs();
	} 
	else if (routeSelection==3) {
		blueRightHalfDiscs();
	} 
	else if (routeSelection==4) {
		redLeftHalfWP();
	} 
	else if (routeSelection==5) {
		blueLeftHalfWP();
	} 
	else if (routeSelection==6) {
		redLeftFullWP();
	} 
	else if (routeSelection==7) {
		blueLeftFullWP();
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
