#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

int goal = 0;
int stack = 1;
int reallevel = 0;
FILE* logfile = nullptr;
int flushCounter = 0;

void updateRealLevel() {
    reallevel = 800 * stack + (goal * 260);
}

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::Rotation rotation(19);
pros::MotorGroup leftMotors({-1, -2, -6},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({3, 4, 5}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// ARM - 2 motors total
pros::MotorGroup Arm({11, -12}, pros::MotorGearset::red); // left side of arm
// pros::MotorGroup ArmRight({-12}, pros::MotorGearset::red); // right side of arm

// Inertial Sensor on port 10
pros::Imu imu(10);
//hi

// Pneumatic claw on ADI Port A
pros::adi::DigitalOut claw('A');

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(9);

// vertical tracking wheel encoder, port 13, reversed
pros::Rotation verticalEnc(-13);

// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 3.5);

// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0);



// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10.25, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            5, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             15, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
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
    if (pros::usd::is_installed()) {
        pros::lcd::print(6, "USB drive installed");
    } else {
        pros::lcd::print(6, "USB drive not installed");
    }
    pros::lcd::initialize(); // initialize brain screen
   
    chassis.calibrate(); // calibrate the chassis
    
    while (imu.is_calibrating()) { // wait for the IMU to finish calibrating
        pros::delay(1000);
    }
   
    updateRealLevel();
    
    int stacklevel = 0; // sets the initial value for the stack level
    rotation.reset_position(); // reset the rotation sensor to 0
    rotation.reset(); // reset the rotation sensor to 0
    Arm.set_zero_position(80); // sets the lowest position of the arm to __ degrees.

    // Hold is okay at the final position, but for step movements we want
    // the motor to stop cleanly after a move_relative() command.
    Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  //  ArmRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    
    if (pros::usd::is_installed()) {
      logfile = fopen("/usd/robot_log.csv", "w");  
      if (!logfile) {
        pros::lcd::print(7, "Failed to open logfile");
      } else {
        pros::lcd::print(7, "Logfile opened");
      }
    }  

    pros::Task screenTask([&]() {
        flushCounter = 0;
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(4, "IMU Heading: %.2f", imu.get_heading());
            pros::lcd::print(5, "Hold: %d \n", Arm.get_brake_mode());
            printf("X: %f\n", chassis.getPose().x);
            printf("Y: %f\n", chassis.getPose().y);
            printf("Theta: %f\n", chassis.getPose().theta);

            if (logfile != nullptr) {
                fprintf(logfile, "Time: %f\n", pros::millis() / 1000.0);
                fprintf(logfile, "X: %f\n", chassis.getPose().x);
                fprintf(logfile, "Y: %f\n", chassis.getPose().y);
                fprintf(logfile, "Theta: %f\n", chassis.getPose().theta);
                fprintf(logfile, "IMU Heading: %.2f\n", imu.get_heading());
                fprintf(logfile, "Hold: %d \n", imu.get_rotation());
                fprintf(logfile, "Rotation: %.2f \n", imu.get_gyro_rate().z);
            }
            flushCounter++;
            if (flushCounter >= 50) {
                fflush(logfile);
                flushCounter = 0;
            }
            // pros::lcd::print(6, "Goal: %d", goal);
            // pros::lcd::print(7, "Stack: %d", stack);
            // pros::lcd::print(3, "Real Level: %d", reallevel);
            //Highest tick position: 25277
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

            // delay to save resources
            pros::delay(50);
        }
    });
}

void clawClose() {
    claw.set_value(true);
}

void clawOpen() {
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
    chassis.turnToPoint(90, 1000, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});

    // Follow the path in path.txt
    chassis.follow(example_txt, 15, 4000, false);

    // wait until the movement is done
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");

    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
}
void HALF_AWP() {
    chassis.moveToPoint(-17, 9.6, 1000, {.forwards = true}); 
    chassis.turnToPoint(-17, 9.6, 1000);    
    chassis.moveToPoint(-17, 9.6, 1000, {.forwards = true});    
    clawOpen();
    chassis.moveToPoint(-17, 9.6, 1000, {.forwards = true});
    clawOpen();
    chassis.moveToPoint(-14, 6.6, 1000, {.forwards = false});
    chassis.turnToPoint(-29.25, -12.9, 1000);
    chassis.moveToPoint(-29.25, -12.9, 1000, {.forwards = true});
    clawClose();
    chassis.moveToPoint(-15.28, 1.56, 1000, {.forwards = false});
    chassis.turnToPoint(-24.24, 3.19, 1000);
    Arm.move_absolute(800,100);
    chassis.moveToPoint(-24.24, 3.19, 1000);
    Arm.move_absolute(800,100);
    pros::delay(250);
    Arm.move_absolute(700,100);
    clawOpen();
    Arm.move_absolute(0,100);
    chassis.moveToPoint(-17.47, 3.63, 1000, {.forwards = false});
    chassis.turnToPoint(-21.64, 21.34, 1000);
    chassis.moveToPoint(-21.64, 21.34, 1000, {.forwards = true});
    clawClose();
    chassis.turnToPoint(-23.13, 7.43 , 1000);
    Arm.move_absolute(1600,100);
    chassis.moveToPoint(-23.13, 7.43 , 1000);
    Arm.move_absolute(1500,100);
    clawOpen();
    chassis.moveToPoint(-21.21, 19.37, 1000, {.forwards = false});
    Arm.move_absolute(0,100);
}
void ELIMS_NEITHER_PIN() {
    chassis.turnToPoint(-17, 9.6, 1000);    
    chassis.moveToPoint(-17, 9.6, 1000, {.forwards = true});    
    clawOpen();
    chassis.moveToPoint(-17.47, 3.63, 1000, {.forwards = false});
    chassis.turnToPoint(-21.64, 21.34, 1000);
    chassis.moveToPoint(-21.64, 21.34, 1000, {.forwards = true});
    clawClose();
    chassis.moveToPoint(-21.64, 21.34, 1000, {.forwards = false});
    chassis.turnToPoint(-23.13, 7.43 , 1000);
    Arm.move_absolute(800,100);
    chassis.moveToPoint(-23.13, 7.43 , 1000);
    Arm.move_absolute(100,100);
    clawOpen();
    chassis.moveToPoint(-21.21, 19.37, 1000, {.forwards = false});
    Arm.move_absolute(0,100);
    chassis.turnToPoint(-21.21, 0, 1000);
    chassis.moveToPoint(-21.21, 0, 1000);
    clawClose();
    chassis.turnToPoint(-21.21, 19.37, 1000);
    Arm.move_absolute(800,100);
    chassis.moveToPoint(-21.21, 19.37, 1000);
    Arm.move_absolute(100,100);
    clawOpen();
    Arm.move_absolute(0,100);
    chassis.moveToPoint(-21.21, 0, 1000, {.forwards = false});
    chassis.turnToPoint(-21.21, 19.37, 1000);
    chassis.moveToPoint(-21.21, 19.37, 1000);
}

void autonomous() {
     chassis.setPose(0, 70, 0);
    Arm.set_zero_position(Arm.get_position());
    Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    Arm.move_absolute(1600,1000);
    pros::delay(850);
    Arm.move_absolute(800,1000);
    pros::delay(850);
    chassis.moveToPoint(0, 92, 1000, {.maxSpeed = 80});
    chassis.turnToPoint(23.5, 92, 1000);
    chassis.moveToPoint(23.5, 92, 1000, {.maxSpeed = 50});
    pros::delay(1000);
    Arm.move_absolute(0,1000);
    Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(500);
    clawClose();
    Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPoint(12, 92, 1000, {.forwards = false});
    chassis.turnToPoint(23, 115, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(23, 115, 1000, {.maxSpeed = 50});
    //chassis.moveToPoint(29.5, 120, 1000, {.maxSpeed = 20});
    pros::delay(1000);
    clawOpen();
    Arm.move_absolute(1500, 1000);
    chassis.turnToPoint(37, 93, 1000);
    chassis.moveToPoint(37, 93, 1000);2
    pros::delay(200);
    Arm.move_absolute(700, 1000);
    Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(1000);
    clawClose();
    Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
}
// Fixed-position up/down steps for the lift.
// Tune these numbers after testing on the real robot.
const double ARM_STEP_DEGREES = 750.0;
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

        chassis.arcade(leftY, rightX * 0.75);

        // =========================
        // Arm
        // L1/L2 = manual hold control
        // Up/Down = one fixed step each press
        // =========================
        bool manualUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool manualDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        if (manualUp) {
            Arm.move_velocity(100);
  //          ArmRight.move_velocity(100);
            armStepMoving = false;
        }
        else if (manualDown) {
            Arm.move_velocity(-75);
          //  ArmRight.move_velocity(-75);
            armStepMoving = false;
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            Arm.move_relative(ARM_STEP_DEGREES, ARM_STEP_VELOCITY);
           // ArmRight.move_relative(ARM_STEP_DEGREES, ARM_STEP_VELOCITY);
            armStepMoving = true;
            armStepStartTime = pros::millis();
            armTimeout = 2000; // Timeout in milliseconds
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            Arm.move_absolute(8, ARM_STEP_VELOCITY);
           // ArmRight.move_absolute(0, ARM_STEP_VELOCITY);
            armStepMoving = true;
            armStepStartTime = pros::millis();
            armTimeout = 2000; // Timeout in milliseconds
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            if (goal == 1) {
                goal = 0;
            }
            else {
                goal++;
            }
            updateRealLevel();
        }
        // else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {                      

        // }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            updateRealLevel();
            if (stack == 4 || stack == 5) {
                Arm.move_absolute(625 * stack + (goal * 260), ARM_STEP_VELOCITY);
                armStepMoving = true;
                armStepStartTime = pros::millis();
                armTimeout = 10000; // Timeout in milliseconds   
            }
            else{
            Arm.move_absolute(reallevel, ARM_STEP_VELOCITY);
            armStepMoving = true;
            armStepStartTime = pros::millis();
            armTimeout = 10000; // Timeout in milliseconds
                        if (stack == 6) {
                stack = 0;
            }
            else {
                stack++;
            }
            updateRealLevel();
            }
                
        }

        else if (!armStepMoving) {
            Arm.brake();
           // ArmRight.brake();
        }


 
        // =========================
        // Claw
        // X = Toggle
        // =========================
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            clawOpenState = !clawOpenState;
            if (clawOpenState) {
                clawClose();
            }
            else {
                clawOpen();                 
                Arm.move_relative(350.0, ARM_STEP_VELOCITY);
          //      ArmRcight.move_relative(90.0, ARM_STEP_VELOCITY);
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