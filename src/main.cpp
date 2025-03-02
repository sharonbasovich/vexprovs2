#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include <iostream>
// #include <sys/_intsup.h>
#include "config.h"
#include <deque>

#pragma once
#include "pros/imu.hpp"
#include <cmath>

class MockIMU : public pros::Imu
{
public:
	MockIMU(int port, double gain)
		: pros::Imu(port), imu_gain(gain) {}

	double get_rotation() const override
	{
		double raw = pros::Imu::get_rotation();
		if (raw == PROS_ERR_F)
			return NAN;
		return raw * imu_gain;
	}

private:
	double imu_gain;
};

MockIMU imu(IMU, 360.0 / 363.0);

int loadToggle = 0;
double target = 0;
double toutput = 0;
bool shouldGo = false;
int state = 0;

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg,					  // left motor group
							  &right_mg,				  // right motor group
							  10.4,						  // 12 inch track width
							  lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
							  450,						  // drivetrain rpm is 450
							  2							  // horizontal drift is 2 (for now)
);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_odom, lemlib::Omniwheel::NEW_2, -5.25); //-3.25

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_odom, lemlib::Omniwheel::NEW_2, 0.5);

lemlib::OdomSensors sensors(
	&vertical_tracking_wheel,	// vertical tracking wheel 1, set to null
	nullptr,					// vertical tracking wheel 2, set to nullptr as we have none
	&horizontal_tracking_wheel, // horizontal tracking wheel 1
	nullptr,					// horizontal tracking wheel 2, set to nullptr as we don't have a
								// second one
	&imu						// inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
											  0,  // integral gain (kI)
											  30, // derivative gain (kD)
											  0,  // anti windup
											  0,  // small error range, in inches
											  0,  // small error range timeout, in milliseconds
											  0,  // large error range, in inches
											  0,  // large error range timeout, in milliseconds
											  5	  // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3.3, // proportional gain (kP)
											  0,   // integral gain (kI)
											  33,  // derivative gain (kD)
											  0,   // anti windup
											  0,   // small error range, in inches
											  0,   // small error range timeout, in milliseconds
											  0,   // large error range, in inches
											  0,   // large error range timeout, in milliseconds
											  0	   // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve
	steer_curve(3,	  // joystick deadband out of 127
				10,	  // minimum output where drivetrain will move out of 127
				1.019 // expo curve gain0
	);

lemlib::ExpoDriveCurve
	throttle_curve(3,	 // joystick deadband out of 127
				   10,	 // minimum output where drivetrain will move out of 127
				   1.019 // expo curve gain
	);

lemlib::Chassis chassis(drivetrain,			// drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors				// odometry sensors
											// &throttle_curve, &steer_curve
);

void intakeForward()
{
	intake_preroller.move(127);
	pros::delay(10);
	intake_hooks.move(127);
	pros::delay(10);
}

void intakeBackward()
{
	intake_preroller.move(-127);
	pros::delay(10);
	intake_hooks.move(-127);
	pros::delay(10);
}

void intakeStop()
{
	intake_preroller.move(0);
	pros::delay(10);
	intake_hooks.move(0);
	pros::delay(10);
}

bool hold = false;
bool hasRing = false;

void holdRing()
{
	// detects if ring is held or not
	// hasRing is constantly updated for if a ring is detected or not
	// set hold to true if intake should stop to hold ring when it is detected or false otherwise
	while (true)
	{
		if (ring_color.get_proximity() > 100)
		{
			if (hold)
			{
				// pros::delay(100);
				intake_hooks.move(0);
			}
			hasRing = true;
		}
		else
		{
			hasRing = false;
		}
		pros::delay(10);
	}
}

bool isRed = IS_RED;
float hue = -1;
bool sort = true;
int currentRed = 0;
bool hasCurrent = false;
bool hasPrevious = false;

int previousColour = 0;
int previousDist = 0;
int ringColour = 0;
// 0 is none, 1 is red, 2 is blue

int proximity = 0;
int distance = 0;
std::deque<int> intakeQ;
// 1 is a red ring, 2 is a blue ring
void detectChange()
{
	ring_color.set_led_pwm(100);
	intakeQ.clear();
	pros::delay(10);
	while (true)
	{
		hue = ring_color.get_hue();
		proximity = ring_color.get_proximity();
		distance = ring_distance.get_distance();

		if (sort)
		{
			if ((hue < 30) && (proximity > RING_PROXIMITY))
			{
				ringColour = 1;
			}
			else if ((hue > 100) && (proximity > RING_PROXIMITY))
			{
				ringColour = 2;
			}
			else
			{
				ringColour = 0;
			}

			if (ringColour != previousColour && ringColour != 0)
			{
				intakeQ.push_back(ringColour);
			}

			previousColour = ringColour;

			if (distance < RING_DISTANCE_THRESHOLD && previousDist >= RING_DISTANCE_THRESHOLD && !intakeQ.empty())
			{
				if ((IS_RED && intakeQ.front() == 2) || (!IS_RED && intakeQ.front() == 1))
				{
					pros::delay(40);
					intakeStop();
					pros::delay(400);
					intakeForward();
				}

				intakeQ.pop_front();
			}

			previousDist = distance;

			pros::delay(10);
		}
	}
}

// void updateController()
// {
//     while (true)
//     {
//         // prints to controller up to 3 rows (this is the max possible)
//         master.clear();

//         // up to 3 lines (0-2)
//         // use %d for integer and boolean
//         // use %f for floating point
//         // if the wrong one is used could have 0 or huge random output
//         master.print(0, 0, "target: %d", loadToggle);
//         master.print(1, 0, "angle: %f", wall_rotation.get_position());
//         master.print(2, 0, "toutput: %f", wallAngle);

//         pros::delay(50);
//     }
// }

double wallAngle;

void wallPID()
{
	double bottom = 20;
	double load = 53;
	double score = 160;

	const double tkP = 2.5; //
	const double tkI = 0;	// 00004;//lower the more perscise
	const double tkD = 5.7; // 4larger the stronger the the kD is so response is quicker
	const double kCos = 8.5;
	double terror = 0;
	double tprevious_error = 0;
	double tintegral = 0;
	double tderivative = 0;

	while (true)
	{
		switch (state)
		{
		case 0:
			target = bottom;
			break;
		case 1:
			target = load;
			break;
		case 2:
			target = score;
			break;

		default:
			target = bottom;
			break;
		}
		wallAngle = wall_rotation.get_position() / 100;
		terror = target - wallAngle;
		tintegral += terror;
		tderivative = terror - tprevious_error;
		toutput = tkP * terror + tkI * tintegral + tkD * tderivative;
		wall_motor.move(toutput + cos(((wall_rotation.get_position() / 100) - 40) * 0.017453) * kCos);
		tprevious_error = terror;
		pros::delay(20);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize()
{
	// wall_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// pros::delay(10);
	// wall_rotation.reset();
	// pros::delay(10);
	wall_rotation.set_position(2000);
	pros::delay(10);
	ring_color.disable_gesture();
	pros::delay(10);
	ring_color.set_integration_time(20);

	lift.extend();
	pros::delay(10);
	// doinker.extend();
	// pros::delay(10);
	chassis.calibrate(); // calibrate sensors
	pros::delay(10);
	// pros::Task controller_task(updateController); // prints to controller, comment out to get back default ui

	// cannot use if using auton selector

	pros::lcd::initialize(); // initialize brain screen
	pros::delay(10);
	pros::Task screen_task([&]()
						   {
        while (true) {
            // print robot location to the brain screen
            // pros::lcd::print(0, "vertical: %f", ((float)wall_rotation.get_position())/100.0);
            // pros::lcd::print(1, "target: %f", target);
            // pros::lcd::print(2, "toutput: %f", toutput);

            // pros::lcd::print(0, "currentRed: %i", currentRed);
            // pros::lcd::print(1, "previousRed: %i", previousRed);
            // pros::lcd::print(2,"hasCurrent: %i", hasCurrent);

            pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
            //pros::lcd::print(2, "Theta: %f", ((chassis.getPose().theta) )); // heading
			//pros::lcd::print(3, "queue front %i", intakeQ[0]);
			//			pros::lcd::print(4, "current color %i", ringColour);

//						pros::lcd::print(5, "previous color %i", previousColour);

						// pros::lcd::print(3, "IMU HEADING: %f", imu.get_heading());

						// delay to save resources
						pros::delay(20);
        } });
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

void leftNoCross()
{
	sort = false;
	chassis.setPose(-63, 8, 180);
	// pros::Task wallstake_task(wallPID);
	// pros::delay(10);

	// pros::Task holdring_task(holdRing);
	// pros::delay(10);

	lift.retract();
	// state += 2;
	pros::delay(600);
	chassis.moveToPoint(-40, 15, 500, {.forwards = false});
	chassis.moveToPoint(-16, 24, 1000, {.forwards = false, .maxSpeed = 60});
	// state -= 2;
	pros::delay(1400);
	clamp.extend();
	intakeForward();
	pros::delay(200);
	chassis.turnToPoint(-20, 47, 600);
	chassis.moveToPoint(-20, 47, 1500);

	chassis.turnToPoint(-66, -18, 800);
	// lift.extend();
	// chassis.moveToPoint(-40, 9, 700); // intake ring for alliance

	chassis.moveToPoint(-58, -16, 3600, {.maxSpeed = 60}); // was -62
	// pros::delay(2000);
	// lift.retract();
	// pros::delay(00);
	// clamp.extend();
	// // pros::delay(10);
	// // lift.extend();

	// // hold = true;

	// chassis.turnToHeading(290, 2000);
	pros::delay(3400);
	chassis.turnToPoint(-17, 0, 1500, {.maxSpeed = 70});
	chassis.moveToPoint(-17, 0, 5000, {.maxSpeed = 70});
	pros::delay(10000);
}

void sawpLeft()
{
	sort = false;
	chassis.setPose(-63, 8, 180);
	// pros::Task wallstake_task(wallPID);
	// pros::delay(10);

	// pros::Task holdring_task(holdRing);
	// pros::delay(10);

	lift.retract();
	state += 2;
	pros::delay(600);
	chassis.moveToPoint(-40, 15, 500, {.forwards = false});
	chassis.moveToPoint(-16, 24, 1000, {.forwards = false, .maxSpeed = 60});
	state -= 2;
	pros::delay(1400);
	clamp.extend();
	intakeForward();
	pros::delay(200);
	chassis.turnToPoint(-20, 47, 600);
	chassis.moveToPoint(-20, 47, 1500);

	chassis.turnToPoint(-66, -18, 800);
	// lift.extend();
	// chassis.moveToPoint(-40, 9, 700); // intake ring for alliance

	chassis.moveToPoint(-58, -16, 3600, {.maxSpeed = 60}); // was -62
	// pros::delay(2000);
	// lift.retract();
	// pros::delay(00);
	// clamp.extend();
	// // pros::delay(10);
	// // lift.extend();

	// // hold = true;

	// chassis.turnToHeading(290, 2000);
	pros::delay(3400);
	clamp.retract();
	chassis.turnToHeading(320, 800, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
	// pros::delay(1200);
	chassis.moveToPoint(-44, -18, 500, {.forwards = false});
	chassis.moveToPoint(-34, -22, 1000, {.forwards = false, .maxSpeed = 40});
	pros::delay(1400);
	clamp.extend();
	chassis.turnToPoint(-23, -56, 800);
	chassis.moveToPoint(-23, -56, 1000);
	chassis.turnToHeading(0, 1000);
	chassis.moveToPoint(-23, 0, 10000);
	pros::delay(10000);
	// chassis.moveToPose(-2, -23, 40, 4000, {.maxSpeed = 60});
}

void rightNoCrossWithDelay()
{
	sort = false;
	chassis.setPose(-63, -8, 0);
	// pros::Task wallstake_task(wallPID);
	// pros::delay(10);

	// pros::Task holdring_task(holdRing);
	// pros::delay(10);

	lift.retract();
	// state += 2;
	pros::delay(600);
	chassis.moveToPoint(-40, -15, 500, {.forwards = false});
	chassis.moveToPoint(-16, -24, 1000, {.forwards = false, .maxSpeed = 60});
	// state -= 2;
	pros::delay(1400);
	clamp.extend();
	intakeForward();
	pros::delay(200);
	chassis.turnToPoint(-20, -47, 600);
	chassis.moveToPoint(-20, -47, 1500);

	chassis.turnToPoint(-66, 18, 800);
	// lift.extend();
	// chassis.moveToPoint(-40, 9, 700); // intake ring for alliance

	chassis.moveToPoint(-58, 16, 5000, {.maxSpeed = 60}); // was -62
	pros::delay(4800);
	chassis.turnToPoint(-17, 0, 1500, {.maxSpeed = 70});
	chassis.moveToPoint(-17, 0, 5000, {.maxSpeed = 70});
	pros::delay(10000);
}

void rightNoCross()
{
	sort = false;
	chassis.setPose(-63, -8, 0);
	// pros::Task wallstake_task(wallPID);
	// pros::delay(10);

	// pros::Task holdring_task(holdRing);
	// pros::delay(10);

	lift.retract();
	state += 2;
	pros::delay(600);
	chassis.moveToPoint(-40, -15, 500, {.forwards = false});
	chassis.moveToPoint(-16, -24, 1000, {.forwards = false, .maxSpeed = 60});
	state -= 2;
	pros::delay(1400);
	clamp.extend();
	intakeForward();
	pros::delay(200);
	chassis.turnToPoint(-20, -47, 600);
	chassis.moveToPoint(-20, -47, 1500);

	chassis.turnToPoint(-66, 18, 800);
	// lift.extend();
	// chassis.moveToPoint(-40, 9, 700); // intake ring for alliance

	chassis.moveToPoint(-58, 16, 5000, {.maxSpeed = 60}); // was -62
	pros::delay(4800);
	chassis.turnToPoint(-17, 0, 1500, {.maxSpeed = 70});
	chassis.moveToPoint(-17, 0, 5000, {.maxSpeed = 70});
	pros::delay(10000);
}

void sawpRight()
{
	sort = false;
	chassis.setPose(-63, -8, 0);
	// pros::Task wallstake_task(wallPID);
	// pros::delay(10);

	// pros::Task holdring_task(holdRing);
	// pros::delay(10);

	lift.retract();
	state += 2;
	pros::delay(600);
	chassis.moveToPoint(-40, -15, 500, {.forwards = false});
	chassis.moveToPoint(-16, -24, 1000, {.forwards = false, .maxSpeed = 60});
	state -= 2;
	pros::delay(1400);
	clamp.extend();
	intakeForward();
	pros::delay(200);
	chassis.turnToPoint(-20, -47, 600);
	chassis.moveToPoint(-20, -47, 1500);

	chassis.turnToPoint(-66, 18, 800);
	// lift.extend();
	// chassis.moveToPoint(-40, 9, 700); // intake ring for alliance

	chassis.moveToPoint(-58, 16, 3600, {.maxSpeed = 60}); // was -62
	// pros::delay(2000);
	// lift.retract();
	// pros::delay(00);
	// clamp.extend();
	// // pros::delay(10);
	// // lift.extend();

	// // hold = true;

	// chassis.turnToHeading(290, 2000);
	pros::delay(3400);
	clamp.retract();
	chassis.turnToHeading(220, 800, {.direction = AngularDirection::CW_CLOCKWISE});
	// pros::delay(1200);
	chassis.moveToPoint(-44, 18, 500, {.forwards = false});
	chassis.moveToPoint(-34, 22, 1000, {.forwards = false, .maxSpeed = 40});
	pros::delay(1400);
	clamp.extend();
	chassis.turnToPoint(-23, 54, 800);
	chassis.moveToPoint(-23, 54, 1000);
	chassis.turnToHeading(180, 1000);
	chassis.moveToPoint(-23, 0, 10000);
	pros::delay(10000);
	// chassis.moveToPose(-2, -23, 40, 4000, {.maxSpeed = 60});
}

void progSkills()
{
	// ---------- INITIALIZATION (â‰ˆ1 s delay) ----------
	// Set initial robot position and retract mechanisms
	chassis.setPose(-58.263, -0.459, 90);
	sort = false;
	lift.retract();
	// doinker.extend();
	pros::delay(10);
	intakeForward();
	pros::delay(200); // Wait 1 sec

	// Start wall PID correction asynchronously
	pros::Task wallstake_task(wallPID);
	pros::delay(10);

	// start ring hold task
	pros::Task holdring_task(holdRing);
	pros::delay(10);

	// ---------- PART 1: MOGO 1 & LADY BROWN (â‰ˆ26 s worst-case) ----------
	// Move forward along x-axis to (-47, -0.459)
	chassis.moveToPoint(-47, -0.459, 700);
	pros::delay(500);
	chassis.turnToHeading(0, 700);
	pros::delay(100);

	// MOGO 1 Sequence:
	// Approach and clamp MOGO 1

	// GET MOGO 1
	chassis.moveToPoint(-50, -20, 1800, {.forwards = false, .maxSpeed = 40});
	pros::delay(1500);
	clamp.extend(); // Clamp MOGO 1
	pros::delay(100);

	// Intake first ring on MOGO 1
	chassis.turnToHeading(70, 400);
	pros::delay(100);
	chassis.moveToPoint(-22, -22, 1000);
	pros::delay(500);

	// Move towards Lady Brown goal and stop intake
	// Turn to face (0, -42) and move there normally (no early exit needed)
	chassis.turnToPoint(0, -46, 500);
	chassis.moveToPoint(0, -46, 400, {.earlyExitRange = 3});
	// Turn to face (28, -51) but exit early to begin moving sooner
	chassis.turnToPoint(33, -48, 250, {.earlyExitRange = 5});

	// Move to (28, -51) but exit early to avoid stopping completely

	// grab lb ring
	chassis.moveToPoint(33, -48, 2000);

	pros::delay(550);

	state += 1; // Raise Lady Brown mechanism
	pros::delay(10);
	// intakeStop();

	// Move to second ring on MOGO 1
	// pros::delay(2500);
	// intake_preroller.move(127);

	// drive back
	chassis.moveToPoint(8, -42, 900, {.forwards = false});
	pros::delay(100);
	chassis.turnToHeading(180, 700);

	// Move to Lady Brown scoring position and release
	chassis.moveToPoint(2, -68, 2400, {.maxSpeed = 40});

	pros::delay(1500);
	intake_hooks.move(-100);
	pros::delay(150);
	intake_hooks.move(0);
	pros::delay(10);
	state += 1; // Score Lady Brown

	pros::delay(200);

	// Reset Lady Brown mechanism and move to next position

	intakeForward();
	pros::delay(10);

	chassis.moveToPoint(7, -47, 900, {.forwards = false});
	pros::delay(700);

	// Move to (-58, -47) facing 270Â° smoothly
	state -= 1;
	pros::delay(200);

	// chassis.turnToHeading(270, 500);
	pros::delay(500);
	chassis.moveToPose(-45, -47, 270, 700);
	pros::delay(10);
	chassis.moveToPose(-64, -47, 270, 2000, {.maxSpeed = 100});
	pros::delay(500);

	// Score Ring 6

	chassis.turnToPoint(-47, -64, 1100, {.maxSpeed = 80});

	chassis.moveToPoint(-47, -64, 1300);
	pros::delay(1300);

	// Score mogo in the corner
	chassis.turnToHeading(70, 700);

	chassis.moveToPoint(-77, -70, 500, {.forwards = false});
	pros::delay(10);
	clamp.retract();
	intakeBackward();
	// ---------- PART 2: SECOND MOGO (=25 s worst-case) ----------
	// Move towards second mogo
	pros::delay(10);

	chassis.moveToPoint(-53, -36, 700);

	// Turn to align with MOGO 2 using the back clamp
	chassis.turnToPoint(-58, 19, 600, {.forwards = false, .earlyExitRange = 5});
	chassis.moveToPoint(-58, 19, 800, {.forwards = false, .earlyExitRange = 5});
	pros::delay(100);

	// SECOND HALF++++++++++++++++++++++
	//  Approach and clamp MOGO 2
	chassis.moveToPoint(-58, 19, 1700, {.forwards = false, .maxSpeed = 40});
	pros::delay(1300);
	clamp.extend();
	intakeForward();
	pros::delay(700);

	// Move and intake rings on MOGO 2
	chassis.moveToPose(-22, 18, 120, 1600);
	pros::delay(1000);
	// chassis.turnToHeading(340, 500);
	// pros::delay(500);
	chassis.moveToPose(-31, 54, 300, 1600);
	pros::delay(1000);

	chassis.moveToPose(-80, 43, 270, 2500, {.maxSpeed = 60});
	pros::delay(500);

	chassis.moveToPose(-58, 30, 350, 800, {.forwards = false});

	chassis.moveToPoint(-58, 66, 1500, {.maxSpeed = 80});
	pros::delay(500);

	// Score MOGO 2 in the corner
	chassis.turnToPoint(-75, 75, 700, {.forwards = false});

	chassis.moveToPoint(-75, 75, 700, {.forwards = false});
	pros::delay(100);
	clamp.retract();
	pros::delay(10);

	// ---------- PART 3: MOGO 3 & LADY BROWN (â‰ˆ27 s worst-case) ----------
	// Move to MOGO 3 and prepare for Lady Brown
	// Line up for Lady Brown and score

	// LB 2++++++++++++
	chassis.moveToPoint(-17, 50, 500, {.earlyExitRange = 3});

	chassis.moveToPoint(-12, 40, 1600, {.maxSpeed = 70});
	state += 1;
	pros::delay(500);
	chassis.turnToHeading(355, 700);
	chassis.moveToPoint(-17, 70, 1800, {.maxSpeed = 60});
	pros::delay(1800);
	intake_hooks.move(-100);
	pros::delay(150);
	intake_hooks.move(0);
	pros::delay(10);
	state += 1;
	pros::delay(900);

	chassis.moveToPoint(-1, 46, 1000, {.forwards = false});
	state -= 2;
	pros::delay(10);
	hold = true;
	pros::delay(300);
	intakeForward();
	// pros::delay(500);

	// FIRST AND SECOND RING MOGO 3
	chassis.moveToPoint(21, 52, 1500); //(Ring 1 mogo 3)
	pros::delay(250);
	chassis.moveToPoint(9, 18, 1500);
	// intake 2 rings and move to Mogo 3

	// chassis.moveToPoint(24, 47, 1500); //(Ring 1 mogo 3)
	// pros::delay(250);

	// chassis.moveToPose(24, 47, 130, 2000); // ring 1

	// pros::delay(500);

	// chassis.moveToPose(24, 23, 180, 2000);
	// pros::delay(500);

	// chassis.turnToHeading(320, 2000);

	// pros::delay(50);

	chassis.moveToPoint(33, 0, 2000, {.forwards = false, .maxSpeed = 60});
	pros::delay(1800);
	clamp.extend();

	// Intake rings on MOGO 3
	chassis.moveToPoint(32, 72, 1500);
	pros::delay(200);
	hold = false;
	intakeForward();

	sort = true;
	chassis.moveToPoint(34, 59, 5000, {.maxSpeed = 35});

	pros::delay(4500);
	chassis.moveToPoint(32, 41, 800, {.forwards = false});
	chassis.moveToPoint(52, 55, 1500); // intake ring for alliance
	pros::delay(1500);
	// Score MOGO 3 in the corner
	chassis.turnToHeading(240, 1000, {.direction = AngularDirection::CW_CLOCKWISE});
	pros::delay(800);
	clamp.retract();
	chassis.moveToPoint(56, 68, 1000, {.forwards = false});
	pros::delay(250);

	// ---------- ALLIANCE STAKE & HANG (â‰ˆ5 s worst-case) ----------
	// Move to intake rings for alliance stake
	// hold = true;
	// pros::delay(10);

	// chassis.moveToPoint(73, 51, 1000); // intake ring for alliance
	// pros::delay(500);
	intakeBackward();
	chassis.moveToPoint(44, 31, 700); // move to position for alliance

	// LINE UP ALLIANCE
	chassis.moveToPoint(47, 0, 1000); // move to position for alliance
	pros::delay(500);
	// chassis.moveToPoint(62, 0, 2000); // turn to alliance
	// pros::delay(500);
	// chassis.turnToHeading(270, 700);
	// chassis.moveToPoint(62, 5, 1000, {.forwards = false}); // go to alliance
	// pros::delay(900);

	// hold = false;
	// intakeForward();
	// pros::delay(500);
	// intakeStop();
	// chassis.moveToPoint(51, 0, 1000, {.forwards = false}); // move to position for alliance

	chassis.turnToHeading(330, 900);
	chassis.moveToPoint(63, -60, 1300, {.forwards = false}); // move to position for alliance
	chassis.moveToPoint(12, -14, 10000);
	// chassis.moveToPoint(56, -19, 1000, {.forwards = false}); // aligh wiht mogo
	// pros::delay(500);
	// chassis.moveToPoint(65, -65, 2000, {.forwards = false}); // push mogo in cornor
	// pros::delay(500);
	// state += 2; // initiate hang
	// chassis.moveToPoint(12, -14, 2000);
	pros::delay(10000);
}

void autonomous()
{
	pros::Task wallstake_task(wallPID);
	pros::delay(10);
	pros::Task hold_task(holdRing);
	pros::delay(10);
	pros::Task detect_task(detectChange);
	pros::delay(10);
	// progSkills();
	progSkills();
	pros::delay(100000);
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

bool intake = false;
bool outake = false;
bool doinkerToggle = false;
bool rushToggle = false;
bool prerollerToggle = false;
int cooldown = 0;
bool check = false;
int cooldown2 = 0;
bool check2 = false;

void opcontrol()
{
	sort = true;
	pros::Task wallstake_task(wallPID);
	pros::delay(10);
	pros::Task detect_task(detectChange);
	pros::delay(10);
	pros::Task hold_task(holdRing);
	pros::delay(10);
	// pros::Task sort_task(colorSort);
	// pros::delay(10);
	lift.retract();

	state += 2;
	pros::delay(600);
	state -= 2;

	// // pros::Task controller_task(updateController); // prints to controller, comment out to get back default ui
	// pros::delay(10);

	while (true)
	{
		// get left y and right x positions of joystick
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		// move the robot
		chassis.arcade(leftY, rightX);

		// buttons

		// when a is pressed, toggle between intaking and stopping the intake
		// overrides outtaking when pressed
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
		{
			intake = !intake;
			outake = 0;
			if (intake)
			{
				intakeForward();
			}
			else
			{
				intakeStop();
			}
		} // activate intake

		// when b is pressed, toggle between outtaking and stopping the intake
		// overrides intaking when pressed
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			outake = !outake;
			intake = 0;
			if (outake)
			{
				intakeBackward();
			}
			else
			{
				intakeStop();
			}
		}

		// when r1 is pressed, moves wall stake mechanism forward by one state
		// unless wall stake mechanism is already fully extended
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
		{

			if (state == 1)
			{
				intake_hooks.move(-100);
				pros::delay(150);
				intake_hooks.move(0);
				pros::delay(10);
				if (intake)
				{
					intake = false;
				}
			}
			if (state != 2)
			{
				state++;
			}
		}

		// when r2 is pressed, moves wall stake mechanism backward by one state
		// unless wall stake mechanism is already fully retracted
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
		{
			if (state != 0)
			{
				state--;
			}
			if (!intake)
			{
				cooldown = 20;
				check = true;
			}
		}

		if (check)
		{
			if (cooldown == 0)
			{
				check = false;
				intakeForward();
				intake = true;
			}
			cooldown--;
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
			lift.extend();
		}

		// extend clamp on press
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
		{
			clamp.extend();
		}

		// retract clamp on press
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
		{
			clamp.retract();
			check2 = true;
			cooldown2 = 30;
			pros::delay(10);
			intake_hooks.move(-127);
		}

		if (check2)
		{
			if (cooldown2 == 0)
			{
				check2 = false;
				if (intake)
				{
					intakeForward();
				}
				else if (outake)
				{
					intakeBackward();
				}
				else
				{
					intakeStop();
				}
			}
			cooldown2--;
		}

		// disable color sort, for matches where colored lighting may mess up detection
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			sort = false;
		}

		// toggle the clamp on the rush arm to grab or release a mobile goal
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{

			rushToggle = !rushToggle;
			if (rushToggle)
			{
				rush.extend();
			}
			else
			{
				rush.retract();
			}
		}

		// quickly pulse the conveyor hooks backward, moving them out of the way
		// this is necessary for the hooks not to hit the ring when attempting to extend wall stake mechanism
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			hold = true;
		}

		if (hold)
		{
			if (hasRing)
			{

				intake = false;
				hold = false;
			}
		}

		// only toggles the preroller stage of intake
		// allows for holding one ring while intaking another
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			prerollerToggle = !prerollerToggle;

			if (prerollerToggle)
			{
				intake_preroller.move(127);
			}
			else
			{
				intake_preroller.move(0);
			}
		}

		// toggle the goal arm
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
		{
			doinkerToggle = !doinkerToggle;
			if (doinkerToggle)
			{
				doinker.extend();
			}
			else
			{
				doinker.retract();
			}
		}

		pros::delay(20);
	}
}