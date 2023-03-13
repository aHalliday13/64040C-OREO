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
 * 
 * THE BEES!
 * - Zach Bruno while having a complete exestential crisis about bees in scuba gear
 * 
 * All I can taste is the bittering agent
 * - Benny Boi after licking a soda can that was sprayed with canned air
 * 
 * Have you ever licked nintendo switch cartridges? I've licked all of mine.
 * - Maximo
*/

// Our Left Y-Axis Curve Function. We use a define because it makes the source code more readable, and saves RAM at runtime instead of a variable.
#define CUBERTCTRL_LY (master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)/abs(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y))*(cbrt(abs(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) - 63.5) + 3.989556)*15.9148)
// Autonomous selection variable
int routeSelection=0;

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
	printf("power,error*kP,integral*kI,derivative*kD\n");
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
		printf("%f,%f,%f,%f\n",power,error*kP,integral*kI,derivative*kD);
	}
	left_drive.brake();
	right_drive.brake();
}

/**
 * Turns the robot chasis a set number of degrees.
 * Uses the flywheel as the front of the robot.
 * Utilizes the motor encoders instead of the inertial sensor for more precision.
 * 
 * \param rotation
 * The number of degrees the robot should turn.
 * Positive to turn clockwise, negative for anti-clockwise.
 * 
 * \param velocity
 * How fast to spin the motors. This should always be a positive number.
 */
void driveETurn(float rotation, float velocity) {
	const float unitsPerDegree=6.67;
	left_drive.tare_position();
	right_drive.tare_position();
	left_drive.move_relative(rotation*unitsPerDegree,velocity);
	right_drive.move_relative(-rotation*unitsPerDegree,velocity);
	while (abs(left_drive.get_positions()[0])<abs(rotation*unitsPerDegree)-5 || abs(right_drive.get_positions()[0])<abs(rotation*unitsPerDegree)-5);
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
	printf(CREDITS);
	left_front.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right_front.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	left_rear.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right_rear.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	
	intake1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	intake2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	flywheel1.set_brake_mode(E_MOTOR_BRAKE_COAST);
	flywheel2.set_brake_mode(E_MOTOR_BRAKE_COAST);
	
	pros::delay(500);
	pros::delay(500);

	// Autonomous selection menu
	while (false) {
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
}

/**
 * Starts at the left half of the field, spins roller, shoots preloads, goes to
 * center of field, grabs discs on the way, and then fires them into the goal.
 * \author aHalliday13
 */
void leftHalfDiscs() {
	flywheel.move_velocity(550);

	left_drive.move_relative(-150,50);
	right_drive.move_relative(-150,50);
	pros::delay(200);
	rollerIntake.move_relative(-300,100);
	pros::delay(1000);
	left_drive.brake();
	right_drive.brake();

	left_drive.move_relative(250,50);
	right_drive.move_relative(250,50);
	pros::delay(700);
	driveETurn(-115,50);
	rollerIntake.move_velocity(200);
	driveIn(-46,8000);
	driveETurn(90,50);
	rollerIntake.brake();
	pros::delay(2000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	pros::delay(3000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	pros::delay(3000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
}

/**
 * Starts at the right half of the field, shoots preloads, spins roller.
 * \author aHalliday13
 */
void rightHalfDiscs() {
	flywheel.move_velocity(540);
	driveIn(20);
	pros::delay(4000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	pros::delay(3000);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	driveIn(-20);
	left_drive.move_relative(450,50);
	right_drive.move_relative(-450,50);
	pros::delay(2000);
	driveIn(18);
	driveETurn(-80,70);
	rollerIntake.move_relative(-800,200);
	left_drive.move_velocity(-600);
	right_drive.move_velocity(-600);
	pros::delay(1000);
	left_drive.brake();
	right_drive.brake();
	rollerIntake.brake();
}

/**
 * Super special secret skills routine that I'm not going to doccument for *reasons*
 * Welcome to recursion hell
 * \author aHalliday13
 */
void skillsAuton() {
	flywheel.move_velocity(400);
	pros::delay(500);
	rollerIntake.move_velocity(-100);
	pros::delay(500);
	for (int i=0;i<3;i++) {
		right_drive.move_relative(200,80);
		left_drive.move_relative(200,80);
		pros::delay(1000);
		left_drive.brake();
		right_drive.brake();
		rollerIntake.brake();
		driveETurn(85,40);
		rollerIntake.move_velocity(-100);
		indexer.set_value(true);
		pros::delay(500);
		indexer.set_value(false);
		pros::delay(300);
		indexer.set_value(true);
		pros::delay(500);
		indexer.set_value(false);
		pros::delay(300);
		indexer.set_value(true);
		pros::delay(500);
		indexer.set_value(false);
		if (i!=2){
			driveETurn(-85,40);
			right_drive.move_relative(-250,40);
			left_drive.move_relative(-250,40);
			pros::delay(2000);
		}
	}
	rollerIntake.move_velocity(200);
	driveETurn(60,40);
	driveIn(-30,7000);
	pros::delay(500);
	driveIn(22);
	driveETurn(-63,40);
	pros::delay(700);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	pros::delay(300);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	pros::delay(300);
	indexer.set_value(true);
	pros::delay(500);
	indexer.set_value(false);
	rollerIntake.move(200);
	driveETurn(-89,40);
	driveIn(-4);
	driveETurn(90,40);
	driveIn(-50);
	left_drive.move_velocity(-100);
	right_drive.move_velocity(-100);
	pros::delay(500);
	left_drive.brake();
	right_drive.brake();
	driveIn(30);
	driveETurn(-90,40);
	left_drive.move_velocity(-100);
	right_drive.move_velocity(-100);
	pros::delay(500);
	left_drive.brake();
	right_drive.brake();
	driveIn(10);
	driveETurn(90,40);
	driveIn(-10);
	
	driveETurn(-60,40);
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
	leftHalfDiscs();
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
		master.print(0,0,"DT:%.0f FT:%.0f FV:%.0f     ",((left_front.get_temperature()+right_front.get_temperature()+left_rear.get_temperature()+right_rear.get_temperature())/4),((flywheel1.get_temperature()+flywheel2.get_temperature())/2),round(flywheel1.get_target_velocity()));
		
		// Drivetrain control functions
		left_drive.move((-CUBERTCTRL_LY + master.get_analog(E_CONTROLLER_ANALOG_LEFT_X)));
		right_drive.move((-CUBERTCTRL_LY - master.get_analog(E_CONTROLLER_ANALOG_LEFT_X)));
		if (left_drive.get_target_velocities()[0]==0) {
			left_drive.brake();
		}
		if (right_drive.get_target_velocities()[0]==0) {
			right_drive.brake();
		}

		// Set the velocity of the intake
		if (master.get_digital(E_CONTROLLER_DIGITAL_A)){
			rollerIntake.move_velocity(-100);
		}
		else {
			rollerIntake.move(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
		}

		// Toggle the flap if the driver presses Y
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)){
			flap.set_value(!flap.get_value());
		}	

		// Set flywheel velocity
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
			// %100
			flywheel.move_velocity(flywheel.get_target_velocities()[0]==600 ? 0 : 600);
		}
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)){
			// %68
			flywheel.move_velocity(flywheel.get_target_velocities()[0]==410 ? 0 : 410);
		}
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)){
			// %65
			flywheel.move_velocity(flywheel.get_target_velocities()[0]==390 ? 0 : 390);
		}
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)){
			// %66
			flywheel.move_velocity(flywheel.get_target_velocities()[0]==400 ? 0 : 400);
		}

		// Fire the indexer
		indexer.set_value(master.get_digital(E_CONTROLLER_DIGITAL_X));

		// Trigger the expansion release
		expansion1.set_value(master.get_digital(E_CONTROLLER_DIGITAL_DOWN)||master.get_digital(E_CONTROLLER_DIGITAL_LEFT));
		expansion2.set_value(master.get_digital(E_CONTROLLER_DIGITAL_DOWN)||master.get_digital(E_CONTROLLER_DIGITAL_RIGHT));

		blocker.set_value(master.get_digital(E_CONTROLLER_DIGITAL_UP));
	}
}
