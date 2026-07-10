//INCLUDE FILES - ANY NECESSARY FILES FOR THE PROGRAM TO RUN
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

//VARIABLES 
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({11, 12, 13}); //signs
pros::MotorGroup right_mg({16, 17, 18}); //signs
pros::adi::Pneumatics piston('A', false); //place holder
pros::Imu inertial(7); //port
pros::Rotation vertodom(8);	//port
lemlib::TrackingWheel vert_tracking_wheel(&vertodom, 2, 2);  //place holder
lemlib::Drivetrain drivetrain (&left_mg, &right_mg, 11.25, lemlib::Omniwheel::NEW_275, 450, 2);
lemlib::OdomSensors odometry(&vert_tracking_wheel, nullptr, nullptr, nullptr, &inertial);
lemlib::ControllerSettings straight_settings//tune
(
	5, //kP
	0, //kI
	1, //kD
	3, //windup range
	1, //small error range, inches
	100, //small error range timeout, milliseconds
	3, //large error range, inches
	500, //large error range timeout, milliseconds
	3 //max acceleration (slew)
); 
lemlib::ControllerSettings turn_settings //tune
(
	5, //kP
	0, //kI
	1, //kD
	3, //windup range
	1, //small error range, inches
	100, //small error range timeout, milliseconds
	3, //large error range, inches
	500, //large error range timeout, milliseconds
	3 //max acceleration (slew)
); 
lemlib::ExpoDriveCurve throttle_curve //tune
(
	5, //joystick deadband out of 127
	12, //min output out of 127
	1 //amount of curve
); 
lemlib::ExpoDriveCurve turn_curve //tune
(
	5, //joystick deadband out of 127
	12, //min output out of 127
	1 //amount of curve
); 
lemlib::Chassis chassis(drivetrain, straight_settings, turn_settings, odometry, &throttle_curve, &turn_curve); 

//AUTON FUNCTIONS
void turnPIDtest()
{
	chassis.turnToHeading(90, false);
}

//FUNCTIONS FOR AUTON AND DRIVER 
void initialize() {}
void disabled() {} //runs while robot is disabled after auton or opcontrol. ends when robot is enabled
void competition_initialize() {} //runs after initialize, b4 auton. ends when robot is enabled. 
//intended for comp-specific initialization routines, like an auton selector.
void autonomous() //runs auton code. if disabled and restarted, the task will restart from the beginning, not resume.
{
	turnPIDtest();
}
void opcontrol() ////runs driver code. if disabled and restarted, the task will restart from the beginning, not resume.
//if no comp control: will run immediately after initialize. 
{
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
    while (true) 
	{
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        // move the robot
        chassis.arcade(leftY, leftX, false, 0.5);

        // delay to save resources
        pros::delay(25);

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			piston.toggle();
		}
    }

}
