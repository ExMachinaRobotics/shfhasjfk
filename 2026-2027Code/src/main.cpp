#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({4, 3, 5},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-1, -2, -6}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// ARM - 2 motors total
pros::MotorGroup ArmLeft({11}, pros::MotorGearset::red); // left side of arm
pros::MotorGroup ArmRight({-12}, pros::MotorGearset::red); // right side of arm

// Inertial Sensor on port 7
pros::Imu imu(7);
//hi

// Pneumatic claw on ADI Port A
pros::adi::DigitalOut claw('A');

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);

// vertical tracking wheel encoder, port 17, reversed
pros::Rotation verticalEnc(-17);

/* horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);

// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);
*/


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for tis mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    int stacklevel = 0; // sets the initial value for the stack level

    // Hold is okay at the final position, but for step movements we want
    // the motor to stop cleanly after a move_relative() command.
    ArmLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ArmRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

            // delay to save resources
            pros::delay(50);
        }
    });
}

void clawOpen() {
    claw.set_value(true);
}

void clawClose() {
    claw.set_value(false);
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void example_autonomous() {
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);

    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});

    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();

    // Turn to face the point x:45, y:-45. Timeout set to 1000
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});

    // Turn to face a direction of 90º. Timeout set to 1000
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});

    // Follow the path in path.txt
    chassis.follow(example_txt, 15, 4000, false);

    // wait until the movement is done
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");

    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
}

// Fixed-position up/down steps for the lift.
// Tune these numbers after testing on the real robot.
const double ARM_STEP_DEGREES = 1000.0;
const int ARM_STEP_VELOCITY = 100;
uint32_t armTimeout = 2000; // Timeout in milliseconds

void opcontrol() {
    pros::adi::Pneumatics left_piston('a', false);
    pros::adi::Pneumatics right_piston('b', false, true);

    bool clawOpenState = false;
    bool armStepMoving = false;
    uint32_t armStepStartTime = 0;

    while (true) {
        // =========================
        // Drive
        // =========================
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, -rightX * 0.75);

        // =========================
        // Arm
        // L1/L2 = manual hold control
        // Up/Down = one fixed step each press
        // =========================
        bool manualUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool manualDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        if (manualUp) {
            ArmLeft.move_velocity(100);
            ArmRight.move_velocity(100);
            armStepMoving = false;
        }
        else if (manualDown) {
            ArmLeft.move_velocity(-75);
            ArmRight.move_velocity(-75);
            armStepMoving = false;
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            ArmLeft.move_relative(ARM_STEP_DEGREES, ARM_STEP_VELOCITY);
            ArmRight.move_relative(ARM_STEP_DEGREES, ARM_STEP_VELOCITY);
            armStepMoving = true;
            armStepStartTime = pros::millis();
            armTimeout = 2000; // Timeout in milliseconds
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            ArmLeft.move_absolute(0, ARM_STEP_VELOCITY);
            ArmRight.move_absolute(0, ARM_STEP_VELOCITY);
            armStepMoving = true;
            armStepStartTime = pros::millis();
            armTimeout = 2000; // Timeout in milliseconds
        }
        else if (!armStepMoving) {
            ArmLeft.brake();
            ArmRight.brake();
        }


        // =========================
        // Claw
        // X = Toggle
        // =========================
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            clawOpenState = !clawOpenState;
            if (clawOpenState) {
                clawOpen();
            }
            else {
                clawClose();
                ArmLeft.move_relative(90.0, ARM_STEP_VELOCITY);
                ArmRight.move_relative(90.0, ARM_STEP_VELOCITY);
                armStepMoving = true;
                armStepStartTime = pros::millis();
                armTimeout = 300; // Timeout in milliseconds
            }
        }

        
        //arm times out after armTimeout milliseconds
        if (armStepMoving && (pros::millis() - armStepStartTime) > armTimeout) {
            armStepMoving = false;
        }

        pros::delay(10);
    }
}