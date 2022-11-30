#include "main.h"
#include "math.h"
#include "devices.cpp"
using namespace pros;

/**
 * THEORY is when you know everything but nothing works.
 * PRACTICE is when everything works but no one knows why.
 * In this lab, theory and practice are combined: nothing works, and no one knows why.
 * - An absolute genius
 */
/**
 * You turned the band saw into a spoon!
 * - What happens when you cut steel on the bandsaw
*/

// Our Left Y-Axis Curve Function. We use a define because it makes the source code more readable, and saves RAM at runtime (over a variable).
#define CUBERTCTRL_LY (master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)/abs(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y))*(cbrt(abs(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) - 63.5) + 3.989556)*15.9148)
// Driver control inversion, allows the driver to flip the front and back of the robot to make driving easier.
bool driveInv=true;

/**
 * Drive a set number of inches forward. Uses the flywheel as the front of the robot.
 * 
 * \param dist
 * 		  The distance in inches to drive. This value really isn't acurate or precise,
 * 		  but it's close enough that you should be able to fine tune it with guesswork
 * 		  once you have a rough idea. Set this value positive to go forward, and 
 * 		  negative to go back.
 * \param velocity
 * 		  How fast to drive. For best results, go as slow as time will allow.
 * 		  This should always be a positive number. Bound (0,100] for E_MOTOR_GEARSET_36,
 * 		  (0,200] for E_MOTOR_GEARSET_18, and (0,600] for E_MOTOR_GEARSET_6	
 * \param waitForComplete
 * 		  Should this function wait until the movement is complete to exit. Defaults
 * 		  to true.
 * \return Nothing
 * \author aHalliday13
 */
void driveIn(float dist, float velocity, bool waitForComplete = true) {
	// 1000 units is 19 3/16" 
	// 57.1172 units per inch
	dist=dist*57.1172;
	left_drive.tare_position();
	right_drive.tare_position();
	left_drive.move_relative(dist,velocity);
	right_drive.move_relative(dist,velocity);

	if (waitForComplete) {
		while ((abs(left_front.get_position())<(abs(dist)-10)) && (abs(right_front.get_position())<(abs(dist)-10))) {}
		delay(100);
	}
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
	inertial.tare();

	left_drive.move_velocity(rotation<0 ? -1*velocity : velocity);
	right_drive.move_velocity(rotation>0 ? -1*velocity : velocity);

	if (rotation>0){
		while (inertial.get_rotation()<rotation) {}
	}
	else if (rotation<0) {
		while (inertial.get_rotation()>rotation) {}
	}
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
	left_front.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_front.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	left_rear.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_rear.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	
	roller.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	intake.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	flywheel1.set_brake_mode(E_MOTOR_BRAKE_COAST);
	flywheel2.set_brake_mode(E_MOTOR_BRAKE_COAST);
	
	inertial.reset(true);
	inertial.tare();

	rollerOpL.set_led_pwm(100);
	rollerOpR.set_led_pwm(100);


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
	delay(3000);
	for (int i=0;i<2;i++) {
		indexer.set_value(true);
		delay(1000);
		indexer.set_value(false);
		delay(500);
	}
	flywheel.brake();

	driveTurn(10,75);
	delay(1000);

	left_drive=-100;
	right_drive=-100;
	delay(1000);
	left_drive.brake();
	right_drive.brake();

	rollIntChain=70;
	
	while (rollerOpR.get_hue() < 100){} // Rotate until blue
	while (rollerOpR.get_hue() > 100){} // Rotate until red
	
	rollIntChain.brake();
}

/**
 * Starts sitting next to the left roller, shoots preloads into high goal, then
 * backs up, and spins the roller to blue.
 * \author aHalliday13
 */
void blueLeftHalfWP() {
	flywheel.move_velocity(550);
	delay(3000);
	for (int i=0;i<2;i++) {
		indexer.set_value(true);
		delay(1000);
		indexer.set_value(false);
		delay(500);
	}
	flywheel.brake();

	driveTurn(10,75);
	delay(1000);

	left_drive=-100;
	right_drive=-100;
	delay(1000);
	left_drive.brake();
	right_drive.brake();

	rollIntChain=70;
	
	while (rollerOpR.get_hue() > 100){} // Rotate until red
	while (rollerOpR.get_hue() < 100){} // Rotate until blue
	
	rollIntChain.brake();
}

/**
 * Starts sitting on the left roller, shoots preloads into high goal, then
 * backs up, and spins the roller to red. Robot will then cross field to other roller,
 * picking up the discs in it's path (shooting them into goals) and then spinning the other roller to red.
 * \author aHalliday13
 */
void redLeftFullWP() {
	flywheel.move_voltage(12000);
	left_drive=-100;
	right_drive=-100;
	delay(500);
	left_drive.brake();
	right_drive.brake();
	rollIntChain=70;
	while (rollerOpL.get_hue() < 100){} // Rotate until blue
	while (rollerOpL.get_hue() > 100){} // Rotate until red
	rollIntChain.brake();

	right_drive.move_relative(200,200);

	while (flywheel1.get_actual_velocity()<590) {}
	indexer.set_value(true);
	delay(1000);
	indexer.set_value(false);
	delay(500);

	while (flywheel1.get_actual_velocity()<590) {}
	indexer.set_value(true);
	delay(1000);
	indexer.set_value(false);
	delay(500);

	flywheel.brake();

	driveTurn(50,50);
	driveIn(70,200);

	left_drive=127;
	right_drive=127;
	delay(2000);
	left_drive.brake();
	right_drive.brake();

	driveIn(-5,70);
	driveTurn(-90,100);
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
	rollIntChain=70;
	flywheel.move_velocity(550);
	while (rollerOpL.get_hue() < 100){} // Rotate until blue
	while (rollerOpL.get_hue() > 100){} // Rotate until red
	rollIntChain.brake();

	driveIn(3,25);
	driveTurn(-7,10);
    driveIn(3,10);
	delay(500);

	while (flywheel1.get_actual_velocity()<550) {}
	indexer.set_value(true);
	delay(1000);
	flywheel.move_velocity(550);
	indexer.set_value(false);
	delay(500);

	while (flywheel1.get_actual_velocity()<550) {}
	indexer.set_value(true);
	delay(1000);
	indexer.set_value(false);
	delay(500);

	flywheel.brake();

	driveTurn(-115,20);
	rollIntChain=127;
	driveIn(-36,100);
	driveIn(-24,25);
	
	driveTurn(80,50);
	flywheel.move_velocity(550);
	while (flywheel1.get_actual_velocity()<550) {}
	for(int i=0;i<2;i++) {
		indexer.set_value(true);
		delay(1000);
		indexer.set_value(false);
		delay(500);
	}

	indexer.set_value(true);
	delay(1000);
	flywheel.move_velocity(550);
	indexer.set_value(false);
	delay(500);
}

/**
 * Starts at the left half of the field, spins roller to blue, shoots preloads, goes to
 * center of field, grabs discs on the way, and then fires them into the goal.
 * \author aHalliday13
 */
void blueLeftHalfDiscs() {
	driveIn(-2,100);
	rollIntChain=70;
	flywheel.move_velocity(550);
	while (rollerOpL.get_hue() < 100){} // Rotate until blue
	while (rollerOpL.get_hue() > 100){} // Rotate until red
	rollIntChain.brake();

	driveTurn(5,50);

	while (flywheel1.get_actual_velocity()<550) {}
	indexer.set_value(true);
	delay(1000);
	flywheel.move_velocity(550);
	indexer.set_value(false);
	delay(500);

	while (flywheel1.get_actual_velocity()<550) {}
	indexer.set_value(true);
	delay(1000);
	indexer.set_value(false);
	delay(500);

	flywheel.brake();

	driveTurn(-115,20);
	rollIntChain=127;
	driveIn(-36,100);
	driveIn(-24,25);
	
	driveTurn(80,50);
	flywheel.move_velocity(550);
	while (flywheel1.get_actual_velocity()<550) {}
	for(int i=0;i<2;i++) {
		indexer.set_value(true);
		delay(1000);
		indexer.set_value(false);
		delay(500);
	}

	indexer.set_value(true);
	delay(1000);
	flywheel.move_velocity(550);
	indexer.set_value(false);
	delay(500);
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
	redLeftFullWP();
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
		left_drive = (CUBERTCTRL_LY*(driveInv ? -1 : 1) + master.get_analog(E_CONTROLLER_ANALOG_LEFT_X));
		right_drive = (CUBERTCTRL_LY*(driveInv ? -1 : 1) - master.get_analog(E_CONTROLLER_ANALOG_LEFT_X));

		// Invert the drivetrain if the driver requests it
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
			driveInv= !driveInv;
		}

		// Set the intake/ roller velocity
		rollIntChain = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

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
	}
}
