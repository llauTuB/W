#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "config/robot_config.hpp"
#include "config/mcl_config.hpp"
#include "config/lemlib_config.hpp"
#include "globals/all_const.hpp" // IWYU pragma: keep


pros::Controller master(pros::E_CONTROLLER_MASTER);
const std::vector<std::string> STATE_MAP = {
    "STORE",
    "HIGH",
    "MEDIUM",
    "LOW",
    "OFF"
};

const std::vector<std::string> AUTO_MAP = {
    "FPL",
    "FPR",
    "MGL",
    "SOLO",
    "FPL",
    "FPR",
    "MGL",
    "LGR",
    "SKILLS"
};

const std::vector<lemlib::Pose> AUTO_STARTING_POSITIONS = {
    lemlib::Pose(-15, -45.5, 0),    // AWP_LEFT_BLUE
    lemlib::Pose(15, -45.5, 0),     // AWP_RIGHT_BLUE
    lemlib::Pose(-15, -45.5, 0),    // AWP_LEFT_RED
    lemlib::Pose(21, -48, 90),     // AWP_RIGHT_RED
    lemlib::Pose(-15, -45.5, 0),    // ELIM_LEFT_BLUE
    lemlib::Pose(15, -45.5, 0),     // ELIM_RIGHT_BLUE
    lemlib::Pose(-15, -45.5, 0),    // ELIM_LEFT_RED
    lemlib::Pose(15, -45.5, 0),     // ELIM_RIGHT_RED
    lemlib::Pose(0, 0, 0)           // SKILLS
};


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {

    pros::lcd::initialize(); // initialize brain screen
    robot.calibrate(); // calibrate sensors

    master = pros::Controller(pros::E_CONTROLLER_MASTER);
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
            pros::lcd::print(0, "X: %f", robot.getPose().x); // x
            pros::lcd::print(1, "Y: %f", robot.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", robot.getPose().theta); // heading
            pros::lcd::print(3, "Init: %s", filter.is_initialized() ? "Enabled" : "Disabled"); // heading
            pros::lcd::print(4, "Enabled: %s", filter.is_enabled() ? "Enabled" : "Disabled"); // heading
            pros::lcd::print(5, "MCL Pose: (%.2f, %.2f, %.2f)", filter.get_prediction().x, filter.get_prediction().y, filter.get_prediction().theta); // number of particles
            master.print(0, 0, "X: %.2f Y: %.2f", robot.getPose().x, robot.getPose().y);

           

            // log position telemetry
            lemlib::telemetrySink()->info("robot pose: {}", robot.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}


void autonomous() {
   filter.set_enabled(true);
   filter.set_starting_pose(lemlib::Pose(48, 8, 0), true);
   robot.moveToPoint(48, 46, 2000, {true, 100, 10, 1});
   robot.turnToHeading(90, 700, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   loader.set_value(127);
   robot.moveToPoint(55, 48, 10000, {true, 70, 10, 1});
}


void opcontrol() {
    master = pros::Controller(pros::E_CONTROLLER_MASTER);
    bool upEncoderState = false;
    bool loaderState = false;
    bool middleDescoreState = false;
    bool upScoreDescoreState = false;
    bool upIntakeState = false;

    while (true) {
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        robot.arcade(leftY, rightX);

        // master.print(0, 0, "X: %.2f Y: %.2f", robot.getPose().x, robot.getPose().y);

        if (master.get_digital_new_press(FILTER_TOGGLE)) {
            filter.set_enabled(filter.is_enabled() ? false : true);
        }


        if (master.get_digital(INTAKE_STATE_LOAD)) {
            intakeStage1.move_voltage(12000);
            intakeStage2.move_voltage(12000);
            intakeStage3.move_voltage(12000);
        } 
        else if (master.get_digital(INTAKE_STATE_LOWSCORE)) {
            intakeStage1.move_voltage(-12000);
            intakeStage2.move_voltage(-12000);
            intakeStage3.move_voltage(-8000);
        } 
        else if (master.get_digital(INTAKE_STATE_MIDDLESCORE)) {
            intakeStage1.move_voltage(12000);
            intakeStage2.move_voltage(8000);
            intakeStage3.move_voltage(-7000);
        } 
        else {
            intakeStage1.move_voltage(0);
            intakeStage2.move_voltage(0);
            intakeStage3.move_voltage(0);
        }
     

        if (master.get_digital_new_press(LOADER_TOGGLE)) {
            loaderState = !loaderState;
            loader.set_value(loaderState);
        } 

        if (master.get_digital_new_press(MIDDLE_DESCORE_TOGGLE)) {
            middleDescoreState = !middleDescoreState;
            middle_descore.set_value(middleDescoreState);
        } 

        if (master.get_digital_new_press(UPSCORE_DESCORE_TOGGLE)) {
            upScoreDescoreState = !upScoreDescoreState;                 
            upScore.set_value(upScoreDescoreState);    
            descore.set_value(upScoreDescoreState);
        }

        if (master.get_digital_new_press(INTAKE_UPPER_TOGGLE)) {
            upIntakeState = !upIntakeState;
            intake_upper.set_value(upIntakeState); 
        }

        if (master.get_digital_new_press(ENCODER_UPPER_TOGGLE)) {
            upEncoderState = !upEncoderState;
            encoder_upper.set_value(upEncoderState); 
        }  


        pros::delay(10);
    }
}
