#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "main.h"

/* ------------------------------- Controller ------------------------------- */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* --------------------------------- Motors --------------------------------- */
// drivetrain motors (keep signs as you set them)
pros::MotorGroup left_drive({8, -9, -10}, pros::MotorGearset::blue);
pros::MotorGroup right_drive({1, -2, 3}, pros::MotorGearset::blue);

// intake motors: removed duplicate single-motor declarations and kept the motor group + intake3
pros::MotorGroup intake_group12({-5, 4}, pros::MotorGearset::blue);
pros::Motor intake3(6, pros::MotorGearset::blue);
pros::Motor intake2(-5, pros::MotorGearset::blue);
pros::Motor intake1(4,pros::MotorGearset::blue);

/* --------------------------------- Sensors -------------------------------- */
pros::Rotation vert(-7);
pros::IMU inertial(15);

/* --------------------------------- Pistons -------------------------------- */
pros::adi::DigitalOut ml('H');
pros::adi::DigitalOut ears('F');
pros::adi::DigitalOut descore('G');
pros::adi::DigitalOut intakelift('E'); // added because opcontrol toggles it

/* ----------------------------- Tracking Wheels ---------------------------- */
// wheel scale / direction left as you provided; you can flip sign if it reports backwards
lemlib::TrackingWheel vert_wheel(&vert, lemlib::Omniwheel::NEW_2, -0.9375);

/* ---------------------------- Drivetrain Setup ---------------------------- */
lemlib::Drivetrain drivetrain(
    &left_drive,
    &right_drive,
    11.33,                       // track width (in)
    lemlib::Omniwheel::NEW_325,  // wheel type for left/right (not tracking wheel)
    450,                         // max wheel RPM used by LemLib for calculations
    2                            // number of driven wheels per side (affects some kinematics)
);

lemlib::OdomSensors sensors(
    &vert_wheel,
    nullptr,
    nullptr,
    nullptr,
    &inertial
);

/* ------------------------------- PID Values ------------------------------- */
lemlib::ControllerSettings lateral_controller( 5,    // kP - proportional
    0.03,   // kI - small integral
    6.7,    // kD - derivative
    3,    // anti-windup
    1,    // small error range (inches)
    200,    // small error timeout (ms)
    3.0,    // large error range
    400,    // large error timeout (ms)
    25      // max acceleration / slew
);

lemlib::ControllerSettings angular_controller(  2.0,    // kP
    0.0,    // kI
    11.5,   // kD
    3.0,    // anti-windup
    1.5,    // small error range (degrees)
    120,    // small error timeout (ms)
    5.0,    // large error range (degrees)
    300,    // large error timeout (ms)
    0       // max acceleration (slew)
);

/* ----------------------------- Create Chassis ----------------------------- */
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors
);

/* ---------------------------- Global Variables ---------------------------- */
bool ml_down = false;
bool ears_down = false;
bool intakelift_down = false;
bool descore_down= false;



/* -------------------------------- Initialize ------------------------------- */
void initialize() {
    // Motor brake modes
    left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    // LCD + odom setup
    pros::lcd::initialize();

    // Calibrate / start LemLib odometry
    chassis.calibrate();

    // Small delay to let IMU begin calibration
    // ensure pose starts at 0,0,0 and start odom loop (some LemLib versions need start())
     // safe even if your LemLib ignores it; required on some versions

    // Brain Screen (debug)
    pros::Task screen_task([&]() {
        while (true) {
            // Pose
            pros::lcd::print(2, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(3, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(4, "Theta: %.2f", chassis.getPose().theta);



            pros::delay(50);
        }
    });
}

void disabled() {}
void competition_initialize() {}

/* ------------------------------ Autonomous -------------------------------- */
void autonomous() {
    // Example safety: ensure pose is correct before running autonomous routines
	chassis.setPose(0,0,0);

       chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    intake_group12.move(127);
    intake3.move(127);
	chassis.moveToPoint(-1, 20, 700,{.maxSpeed=90,.minSpeed=85});
    chassis.moveToPoint(-7, 36, 700,{.maxSpeed=65,.minSpeed=65});
    chassis.waitUntil(8);
    ml.set_value(1);
    chassis.moveToPoint(-29, 50, 600);
     ml.set_value(0);
    chassis.moveToPoint(-29, 51.5, 1400,{.maxSpeed=50,.minSpeed=50});
    ml.set_value(1);
    chassis.moveToPoint(-7, 36, 700,{.forwards=false});

    chassis.turnToPoint(-35.5, 9, 600);
    chassis.moveToPoint(-35.5, 9, 1000);

    chassis.turnToPoint(-35.5, 26, 600,{.forwards=false});
    chassis.moveToPoint(-35.5, 29, 900,{.forwards=false});

    
    chassis.waitUntilDone();
    ears.set_value(1);
    pros::delay(1000);

    //matchloader
    chassis.turnToPoint(-34, 5, 400);
    chassis.moveToPoint(-34, 5, 1000,{.maxSpeed=90,.minSpeed=85});
    intake3.move(127);
    pros::delay(500);
    chassis.moveToPoint(-34, -6.7, 1000,{.maxSpeed=50,.minSpeed=45});
    ears.set_value(0);
    
    pros::delay(750);

    chassis.moveToPoint(-34, 10,1000,{.forwards=false});

//middle goal
   chassis.turnToPoint(-7, 36, 600,{.forwards=false});
    chassis.moveToPoint(-7, 36, 1000,{.forwards=false,.maxSpeed=90,.minSpeed=85});
   intake_group12.move(0);
    chassis.moveToPoint(5, 48, 1600,{.forwards=false});
    chassis.waitUntilDone();
    intake3.move(-100);
    intake_group12.move(-127);
    pros::delay(100);
    intake_group12.move(80);
    intake3.move(-50);
    
   

    pros::delay(1000);
    chassis.moveToPoint(-22, 24, 1000);
    ml.set_value(0);
    chassis.turnToPoint(-22, 40, 600);
    chassis.moveToPoint(-22, 40, 1000,{.maxSpeed=85,.minSpeed=90});
    chassis.turnToHeading(6, 600);


    chassis.waitUntilDone();
    pros::delay(500);
     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	
	
}

/* ------------------------------- Operator --------------------------------- */
void opcontrol() {
    int dead_zone = 8;

    int intake_speed = 0;
    int intake3_speed = 0;

    bool intake3_allowed = true;
    uint32_t state_change_time = 0;
    const int OFF_RPM = 60;
    const int ON_RPM = 65;
    const int DEBOUNCE_MS = 200;
    const int KICKSTART_MS = 300;
    uint32_t r1_press_time = 0;

    while (true) {
        /* -------------------- Drive -------------------- */
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (abs(leftY) < dead_zone) leftY = 0;
        if (abs(rightX) < dead_zone) rightX = 0;

        left_drive.move(leftY + rightX);
        right_drive.move(leftY - rightX);

        /* -------------------- Intake Logic -------------------- */
        bool L1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool R1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool R2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        if (R2) {
            intake_speed = -90;
            intake3_speed = -120;
            intake3_allowed = true;
            r1_press_time = 0;
        }
        else if (R1) {
            if (r1_press_time == 0) r1_press_time = pros::millis();

            intake_speed = 127;
            intake3_speed = 127;

            if (ears_down) {
                double rpm = intake3.get_actual_velocity();
                uint32_t now = pros::millis();

                if (now - r1_press_time > KICKSTART_MS) {
                    if (intake3_allowed && std::abs(rpm) < OFF_RPM) {
                        if (state_change_time == 0) state_change_time = now;
                        if (now - state_change_time > DEBOUNCE_MS) {
                            intake3_allowed = false;
                            state_change_time = 0;
                        }
                    } else if (!intake3_allowed && std::abs(rpm) > ON_RPM) {
                        if (state_change_time == 0) state_change_time = now;
                        if (now - state_change_time > DEBOUNCE_MS) {
                            intake3_allowed = true;
                            state_change_time = 0;
                        }
                    } else {
                        state_change_time = 0;
                    }
                }

                if (!intake3_allowed) intake3_speed = 30;
            }
            else {
                intake3_allowed = true;
            }
        }
        else if (L1) {
            intake1.move(120);
            intake2.move(40);
            intake3.move(-35);

            r1_press_time = 0;
        }
        else {
            intake_speed = 0;
            intake3_speed = 0;
            intake3_allowed = true;
            r1_press_time = 0;
        }

        /* -------- Apply motor commands (skip group during L1) -------- */
        if (!L1) {
            intake_group12.move(intake_speed);
            intake3.move(intake3_speed);
        }

        /* -------------------- Pistons -------------------- */
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            ears_down = !ears_down;
            ears.set_value(ears_down);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            intakelift_down = !intakelift_down;
            intakelift.set_value(intakelift_down);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            descore_down = !descore_down;
           descore.set_value(descore_down);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            ml.set_value(1);
        } else {
            ml.set_value(0);
        }

        controller.print(
            0, 0,
            "RPM:%.1f Mode:%s",
            intake3.get_actual_velocity(),
            intake3_allowed ? "FULL" : "SLOW"
        );

        pros::delay(20);
    }
}