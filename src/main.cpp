#include "main.h"
#define LEFT_FRONT_PORT 1
#define LEFT_BACK_PORT 2
#define RIGHT_FRONT_PORT 3
#define RIGHT_BACK_PORT 4
#define CATA_PORT_ONE 7
#define CATA_PORT_TWO 8
#define INTAKE_PORT 10
#define LEFT_PISTON_PORT 'B'
#define RIGHT_PISTON_PORT 'A'

pros::Controller controller1(CONTROLLER_MASTER);

pros::Motor leftFrontDrive(LEFT_FRONT_PORT, true);
pros::Motor leftBackDrive(LEFT_BACK_PORT, true);
pros::Motor rightFrontDrive(RIGHT_FRONT_PORT, false);
pros::Motor rightBackDrive(RIGHT_BACK_PORT, false);
pros::Motor_Group leftDrive({leftFrontDrive, leftBackDrive});
pros::Motor_Group rightDrive({rightFrontDrive, rightBackDrive});

pros::Motor intake(INTAKE_PORT);

pros::Motor cata1(CATA_PORT_ONE, true);
pros::Motor cata2(CATA_PORT_TWO, false);
pros::Motor_Group cata({cata1, cata2});

pros::ADIDigitalOut pistonL(LEFT_PISTON_PORT);
pros::ADIDigitalOut pistonR(RIGHT_PISTON_PORT);

bool catapultToggle = false;
bool pneumaticRightToggle = true;       
bool pneumaticRightToggle2 = false;
bool pneumaticLeftToggle = true;       
bool pneumaticLeftToggle2 = false;

bool isAuton = false;

int velCap; //velCap limits the change in velocity and must be global
int targetLeft;
int targetRight;

void drivePIDFn(){
  leftDrive.tare_position(); //reset base encoders
  rightDrive.tare_position();
  int errorLeft;
  int errorRight;
  float kp = 0.075;
  float kpTurn = 0.2;
  int acc = 5000;
  int voltageLeft = 0;
  int voltageRight = 0;
  int signLeft;
  int signRight;
  // pros::delay(5);
  while(true){
    errorLeft = targetLeft - leftFrontDrive.get_position(); //error is target minus actual value
    errorRight = targetRight - rightFrontDrive.get_position();

    signLeft = (errorLeft > 0) - (errorLeft < 0);
    signRight = (errorRight > 0) - (errorRight < 0);

    if(signLeft == signRight){
      voltageLeft = errorLeft * kp; //intended voltage is error times constant
      voltageRight = errorRight * kp;
    }
    else{
      voltageLeft = errorLeft * kpTurn; //same logic with different turn value
      voltageRight = errorRight * kpTurn;
    }

    velCap = velCap + acc;  //slew rate
    if(velCap > 115){
      velCap = 115; //velCap cannot exceed 115
    }

    if(abs(voltageLeft) > velCap){ //limit the voltage
      voltageLeft = velCap * signLeft;
    }

    if(abs(voltageRight) > velCap){ //ditto
      voltageRight = velCap * signRight;
    }

    leftBackDrive.move(voltageLeft); //set the motors to the intended speed
    leftFrontDrive.move(voltageLeft);
    rightBackDrive.move(voltageRight);
    rightFrontDrive.move(voltageRight);

    // pros::delay(25);
  }
}

void drive(int left, int right){
  targetLeft = targetLeft + left;
  targetRight = targetRight + right;
  velCap = 0; //reset velocity cap for slew rate
}

void cataControl(){
    // Cata 

    while(true){
        if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_B)) { // Button B
            catapultToggle = true;
            cata.move_velocity(100);
        } else if (catapultToggle) {
            catapultToggle = false;
            cata.move_velocity(0);
        }

        pros::Task::delay(25);
    }

}

void wingControl(){
    // Left wing 
    while(true){
        if(controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // L2
            pneumaticRightToggle2 = true;
            if (pneumaticRightToggle){
                pistonL.set_value(true);
            } else {
                pistonL.set_value(false);
            }
        } else if(pneumaticRightToggle2){
            pneumaticRightToggle2 = false;
            pneumaticRightToggle = !pneumaticRightToggle;
        }

        // Right Wing

        if(controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            pneumaticLeftToggle2 = true;
            if (pneumaticLeftToggle){
                pistonR.set_value(true);
            } else {
                pistonR.set_value(false);
            }
        } else if(pneumaticLeftToggle2){
            pneumaticLeftToggle2 = false;
            pneumaticLeftToggle = !pneumaticLeftToggle;
        }

        pros::Task::delay(25);
    }
    
}

void intakeControl(){
    while(true){
        if(controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&  !controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            intake.move_velocity(100);

        } else if(controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intake.move_velocity(-100);

        } else{
            intake.move_velocity(0);
        }

        pros::Task::delay(25);
    }
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	pros::Task cataTask(cataControl);
	pros::Task wingTask(wingControl);
	pros::Task intakeTask(intakeControl);
    autonomous();
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
	isAuton = true;

	// How to use auton:
	// drive(left motor group distance, right motor group distance)
	// ie. drive(100, 100) = 100 ticks forward
	// drive(100, -100) = right turn for 100 ticks

    drive(100000, 100000);
    pros::Task autonTask(drivePIDFn);



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

	// isAuton = false;

	while(true){
		int power = controller1.get_analog(ANALOG_LEFT_Y);
		int turn = controller1.get_analog(ANALOG_RIGHT_X);
		int left = power + turn;
		int right = power - turn;

		leftDrive.move(left);
		rightDrive.move(right);
	}
}
