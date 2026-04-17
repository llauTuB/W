#include "utils/autons.hpp"
#include "config/lemlib_config.hpp"
#include "config/mcl_config.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "config/robot_config.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "utils/distance_reset.hpp"


// void moveStraight(float length, int timeout, lemlib::MoveToPointParams params) {
//     if (robot.isInMotion()) robot.waitUntilDone();
//     params.forwards = length > 0;
//     lemlib::Pose pose = robot.getPose();
//     robot.moveToPoint(pose.x + length * sin(lemlib::degToRad(pose.theta)),
//                         pose.y + length * cos(lemlib::degToRad(pose.theta)), timeout, params);
// }

//  void wait_red(){//я пыталась
//     uint32_t start = pros::millis();
//     // intake.move_voltage(12000);
//     while(pros::millis() - start < 1000){
//         if(optical.get_hue() >= 0 && optical.get_hue() <= 30){
//          intakeStage1.move_voltage(0);
//          intakeStage2.move_voltage(0);
//          // intakeStage3.move_voltage(0);
//             return;
//         }
//         pros::delay(10);
//     }
//  }

bool autoIntakeActive = false;

void autoIntakeTaskFn() {
    while (true) {
            double hue = optical.get_hue();
            double prox = optical.get_proximity();
            bool isRed = (prox > 110) && (hue > 340 || hue < 20);
            bool isBlue = (prox > 110) && (hue > 190 && hue < 250);if (autoIntakeActive) {

            intakeStage1.move_voltage(12000);
            intakeStage2.move_voltage(12000);

            if (isRed || isBlue) intakeStage3.move_voltage(0); 
            else intakeStage3.move_voltage(12000);
        }
        pros::delay(10); 
    }
}

 void loadBasket(){
   autoIntakeActive = true;
   intakeStage1.move_voltage(12000);
   intakeStage2.move_voltage(12000);
   intakeStage3.move_voltage(12000);
 }

 void loadBasket_slower(){
   autoIntakeActive = true;
   intakeStage1.move_voltage(8000);
   intakeStage2.move_voltage(3000);
   intakeStage3.move_voltage(2000);
 }


 void stopIntake(){
   autoIntakeActive = false;
	intakeStage1.move_voltage(0);
   intakeStage2.move_voltage(0);
   intakeStage3.move_voltage(0);
 }


 void keep_intake_alive(){
	intakeStage1.move_voltage(5000);
   intakeStage2.move_voltage(2000);
   intakeStage3.move_voltage(0);
 }

 void scoreUp() {
   autoIntakeActive = false;
   upScore.set_value(1);
   intakeStage1.move_voltage(12000);
   intakeStage2.move_voltage(12000);
   intakeStage3.move_voltage(12000);
 }

  void scoreUp_slower() {
   autoIntakeActive = false;
   upScore.set_value(127);
   intakeStage1.move_voltage(12000);
   intakeStage2.move_voltage(12000);
   intakeStage3.move_voltage(12000);
 }



 void scoreMiddle() {
   autoIntakeActive = false;
   intakeStage1.move_voltage(12000);
   intakeStage2.move_voltage(7000);
   intakeStage3.move_voltage(-7000);
 }


  void scoreMiddle_slower() {
   autoIntakeActive = false;
   intakeStage1.move_voltage(12000);
   intakeStage2.move_voltage(4000);
   intakeStage3.move_voltage(-4000);
 }

 void scoreMiddleDown() {
   autoIntakeActive = false;
   intakeStage1.move_voltage(-6000);
   intakeStage2.move_voltage(-12000);
   intakeStage3.move_voltage(-12000);
 }

//  void park_blue(){
//    uint32_t start = pros::millis();
//    robot.tank(110,110);
//    while(pros::millis() - start < 4000){
//        if(park_opt.get_hue() >= 180 && park_opt.get_hue() <= 210){
//          robot.tank(0,0);

//            return;
//        }
//       pros::delay(5);

//    }
//  }

//   void park_red(){
//    uint32_t start = pros::millis();
//    robot.tank(115,115);
//    while(pros::millis() - start < 4000){
//        if(park_opt.get_hue() >= 5 && park_opt.get_hue() <= 20){
//                pros::delay(50);

//          robot.tank(0,0);

//            return;
//        }
//       pros::delay(5);

//    }
//  }


//  void wait_blue(){
//    uint32_t start = pros::millis();
//    scoreMiddle_slower();
//    while(pros::millis() - start < 3000){
//        if(optical.get_hue() >= 200 && optical.get_hue() <= 220){
//             // pros::delay(550);
//         intakeStage2.move_voltage(-12000);
//           intakeStage3.move_voltage(-12000);
//       // stopIntake();
//            return;
//        }
//    }
// }


void lowLong() { //4 long 5low
   descore.set_value(127);
   robot.setPose(-48, -10, 180);
   robot.moveToPoint(-48, -45.5, 900, {true, 127, 60, 1});
   robot.turnToHeading(270, 450, {lemlib::AngularDirection::AUTO, 127, 30, 1});
   loader.set_value(127);
   robot.moveToPoint(-63, -47.5, 1100, {true, 60, 1, 1});
   loadBasket();
   robot.waitUntilDone();
   distance_reset();
   robot.moveToPoint(-25, -47.5, 1000, {false, 120, 10, 1});
   robot.waitUntilDone();
   distance_reset();
   scoreUp();
   pros::delay(1100);
   stopIntake();
   loader.set_value(0);

   robot.turnToHeading(5, 500, {lemlib::AngularDirection::AUTO, 127, 50, 1});
   robot.waitUntilDone();
   intakeStage1.move_voltage(12000);
   intakeStage2.move_voltage(1000);
   intakeStage3.move_voltage(1000);

   upScore.set_value(0);


   robot.moveToPoint(-25, -27, 800, {true, 120, 30, 1});
   pros::delay(380);
   loader.set_value(1);
   pros::delay(400);
   loader.set_value(0);

   

   intakeStage2.move_voltage(-12000);
   intakeStage3.move_voltage(-12000);
   pros::delay(200);
stopIntake();
   robot.turnToHeading(43, 500, {lemlib::AngularDirection::AUTO, 127, 50, 1});
   robot.waitUntilDone();
   robot.moveToPoint(-10.5, -14, 800, {true, 120, 20, 1});
   intake_upper.set_value(1);
   robot.waitUntilDone();
   scoreMiddleDown();
   pros::delay(1200);
   stopIntake();
      robot.moveToPoint(-36, -37, 900, {false, 120, 100, 1});
      robot.turnToHeading(105, 500, {lemlib::AngularDirection::AUTO, 127, 120, 1});

      intake_upper.set_value(0);
      robot.moveToPoint(-18.5, -39, 1100, {true, 120, 110, 1});
      descore.set_value(0);

      robot.turnToHeading(55, 500, {lemlib::AngularDirection::AUTO, 127,127, 1});


      

   
}

void left43() { //4 on long goal and 3 on mid goal
   descore.set_value(1);
   robot.setPose(-50, 16, 67);
   loadBasket();
   robot.moveToPoint(-25, 23, 800, {true, 127, 90, 1});
   pros::delay(380);
   descore.set_value(1);
   loader.set_value(1);
   robot.swingToHeading(120, lemlib::DriveSide::LEFT, 1000, {lemlib::AngularDirection::CW_CLOCKWISE, 127, 120, 15});
   robot.moveToPose(-25, 47, 270, 1000, {false, 2, 0.2, 127, 90, 0});

   // robot.waitUntilDone();
   // distance_reset_FR();
   //    robot.moveToPoint(-25, 47, 750, {false, 120, 120, 1});
   robot.waitUntilDone();
   robot.tank(-127, -127);
   distance_reset_FR();
   scoreUp();
   pros::delay(950);
   stopIntake();
   robot.tank(0, 0);
   upScore.set_value(0);
   loadBasket();
   robot.moveToPoint(-69, 48, 1300, {true, 70, 1, 1});
   robot.waitUntilDone();
   distance_reset_FR();
   // long goal + ziga

   intakeStage2.move_voltage(-4000);
   intakeStage3.move_voltage(-4000);
   pros::delay(400);
   stopIntake();
   middle_descore.set_value(1);
   upScore.set_value(1);
   robot.moveToPose(-12, 11, 318, 1600, {false, 2, 0.2, 100, 0, 0});

   robot.waitUntilDone();
   scoreMiddle_slower();
   loader.set_value(0);

   pros::delay(1500);
   stopIntake();
   robot.moveToPoint(-32, 35.5, 850, {true, 127, 80, 2});
   // robot.turnToHeading(270, 300, {lemlib::AngularDirection::CW_CLOCKWISE, 127, 127, 1});
   // robot.moveToPoint(-17, 40, 2000, {false, 127, 127, 0});
   robot.moveToPose(-15, 36, 270, 1200, {false, 2, 0.2, 127, 100, 0});
    descore.set_value(0);

   robot.turnToHeading(270, 100000, {lemlib::AngularDirection::AUTO, 10, 1, 0});
}
   // // robot.moveToPoint(-17, 40, 2000, {false, 127, 127, 0});
   // robot.moveToPose(-14, 39, 270, 1200, {false, 2, 0.2, 127, 100, 0});
   // robot.turnToHeading(290, 100000, {lemlib::AngularDirection::AUTO, 10, 5, 0});

   

void right43() { //4 on long goal and 5 on mid goal
   robot.setPose(-50, -16, 109);
   loadBasket();
   robot.moveToPoint(-25, -24, 1000, {true, 127, 100, 10});

   pros::delay(470);
   // robot.moveToPose(-23, -26, 140, 900, {true, 3, 0.2, 127, 120, 10}, false);

   descore.set_value(1);
   loader.set_value(1);

   robot.swingToHeading(230, lemlib::DriveSide::RIGHT, 400, {lemlib::AngularDirection::AUTO, 127, 120, 10});
   robot.moveToPose(-46, -47, 270, 900, {true, 2, 0.1, 127, 80, 0});
   robot.waitUntilDone();
   robot.moveToPoint(-25, -47, 700, {false, 120, 80, 1});
   robot.waitUntilDone();
   robot.tank(-127, -127);
   distance_reset_FL();
   scoreUp();
   pros::delay(700);
   robot.tank(0, 0);
   stopIntake();
   loader.set_value(1);
   loadBasket();
   upScore.set_value(0);

   robot.moveToPoint(-67, -49, 1300, {true, 60, 1, 1});
   robot.waitUntilDone();
   distance_reset_FL();
   
   robot.moveToPose(-25, -25, 215, 1100, {false, 2, 0.1, 127, 10, 0});
     intakeStage2.move_voltage(-4000);
   intakeStage3.move_voltage(-4000);
   pros::delay(600);
   stopIntake();
   robot.turnToHeading(46, 450, {lemlib::AngularDirection::AUTO, 127, 127, 2});
   loader.set_value(0);
      robot.moveToPose(-16, -12, 45, 800, {true, 2, 0.1, 127, 100, 0});

   robot.waitUntilDone();
    loader.set_value(1);
   intake_upper.set_value(1);
   scoreMiddleDown();
      pros::delay(1300);
   stopIntake();
      robot.moveToPose(-32, -34, 90, 1100, {false, 2, 0.1, 127, 100, 0});
      descore.set_value(0);
    loader.set_value(0);

   robot.moveToPose(-13.5, -37, 80, 1600, {true, 2, 0.1, 110, 10, 0});

   robot.turnToHeading(55, 100000, {lemlib::AngularDirection::AUTO, 120, 80, 0});

   // robot.turnToHeading(220, 350, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   // robot.moveToPoint(-33, -39, 400, {true, 127, 127, 3});
   // // robot.turnToHeading(270, 400, {lemlib::AngularDirection::AUTO, 127, 127, 1});
   // robot.waitUntilDone();
   // descore.set_value(0);
   // loader.set_value(0);
   // robot.moveToPose(-13, -58, 270, 1600, {false, 2, 0.1, 90, 10, 0});

   // robot.turnToHeading(290, 100000, {lemlib::AngularDirection::AUTO, 10, 5, 0});

   
}


void rightLong7()
 { //7 blocks on right side goal
   descore.set_value(127);
   robot.setPose(-46, -8, 90);
   loadBasket();
   robot.moveToPoint(-21, -26, 1500, {true, 120, 10, 1});
    pros::delay(650);
   loader.set_value(127);
   pros::delay(1400);
   robot.turnToHeading(75, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.waitUntilDone();
   // loader.set_value(0);
   // stopIntake();
   robot.moveToPoint(-43, -47, 900, {false, 110, 10, 1});
   robot.waitUntilDone();
  pros::delay(100);
   robot.turnToHeading(270, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});

   robot.moveToPoint(-24, -48.5, 900, {false, 120, 10, 1});
   robot.turnToHeading(270, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.waitUntilDone();
   
   scoreUp();
   pros::delay(1800);
   stopIntake();
   robot.moveToPoint(-62, -48.5, 900, {true, 55, 5, 1});
   loadBasket();
   robot.waitUntilDone();
   pros::delay(100);
   robot.moveToPoint(-55, -48.5, 900, {false, 80, 10, 1});
   robot.waitUntilDone();
   robot.moveToPoint(-63, -48.5, 900, {true, 80, 10, 1});
   pros::delay(100);
   robot.moveToPoint(-24, -49.5, 900, {false, 120, 10, 1});
   robot.waitUntilDone();
   scoreUp();
   pros::delay(1400);
   stopIntake();
   robot.moveToPoint(-40, -49, 1400, {true, 110, 10, 1});
    robot.turnToHeading(330, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.moveToPoint(-35, -60, 900, {false, 110, 10, 1});
    robot.turnToHeading(270, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
    descore.set_value(0);
   robot.moveToPoint(-11, -61.5, 1200, {false, 60, 10, 1});
   


}

void awp_2long_1middle()
{
  
   descore.set_value(1);
   robot.setPose(48, 10, 180);
   
   loadBasket();
   robot.moveToPoint(48, 3, 400, {true, 100, 70, 4});
   robot.waitUntilDone();
   pros::delay(100);

   distance_reset_BL();
   pros::delay(300);
   robot.moveToPoint(48, 45.5, 1000, {false, 127, 10, 1});
   robot.turnToHeading(90, 500, {lemlib::AngularDirection::AUTO, 100, 10, 1});
   loader.set_value(1);
   robot.moveToPoint(64, 48, 1100, {true, 60, 3, 1});
   loadBasket();
   robot.waitUntilDone();
   distance_reset_FL();
   robot.moveToPoint(24.5, 48, 700, {false, 120, 100, 1});
   robot.waitUntilDone();

   robot.tank(-127, -127);
   distance_reset_FL();
   scoreUp();
   pros::delay(1050);
   robot.tank(0, 0);
   stopIntake();
   loader.set_value(0);
   robot.turnToHeading(180, 550, {lemlib::AngularDirection::AUTO, 100, 30, 1});
   loadBasket();
   // robot.moveToPoint(28.5, 26, 700, {true, 120, 40, 1});
   robot.moveToPose(28, 26, 170, 700, {true, 2, 0.2, 127, 40, 1});

   upScore.set_value(0);
   pros::delay(250);
   loader.set_value(127);
   pros::delay(200);
   loader.set_value(0);
   robot.moveToPoint(27.5, -17.5, 950, {true, 127, 50, 1});
   pros::delay(800);
   loader.set_value(127);
   pros::delay(300);
   robot.turnToHeading(110, 450, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.moveToPoint(42, -45,700, {true, 127, 10, 1});
   // robot.moveToPose(40, -45, 90, 800, {true, 2, 0.25, 127, 30, 1});
   loader.set_value(0);

   robot.turnToHeading(90, 350, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.moveToPoint(25, -46, 500, {false, 120, 70, 1});
   robot.waitUntilDone();
   scoreUp();
   robot.tank(-127, -127);
   distance_reset_FL();
   pros::delay(1150);
   robot.tank(0, 0);
   stopIntake();
   loader.set_value(127);
   robot.moveToPoint(68, -46, 1400, {true, 60, 1, 1});

   intakeStage1.move_voltage(12000);
   intakeStage2.move_voltage(2000);
      robot.moveToPose(13, -9, 135, 1100, {false, 2, 0.3, 120, 0, 0});
   middle_descore.set_value(1);

      pros::delay(300);
   intakeStage1.move_voltage(-12000);
   intakeStage2.move_voltage(-12000);
   intakeStage3.move_voltage(-12000);
   stopIntake();
   upScore.set_value(1);
   robot.waitUntilDone();
   scoreMiddle_slower(); 
}

// void skills() {//skilss

// //1 loader
//    descore.set_value(127);
//    robot.setPose(48, 8, 0);
//    robot.moveToPoint(48, 47, 1000, {true, 100, 10, 1});
//    robot.turnToHeading(90, 450, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    loader.set_value(127);
//    robot.moveToPoint(62, 48, 1200, {true, 70, 10, 1});
//    loadBasket();
//    robot.waitUntilDone();
//    distance_reset();
//    robot.waitUntilDone();
//    robot.moveToPoint(57, 48, 700, {false, 80, 30, 1});
//    robot.moveToPoint(62, 48, 800, {true, 70, 10, 1});
//    robot.waitUntilDone();
//    pros::delay(500);
//    robot.moveToPoint(33, 59, 1000, {false, 100, 10, 1});
//    robot.waitUntilDone();
//    loader.set_value(0);
//    robot.turnToHeading(90, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    stopIntake();
//    //    //2 loader
//    robot.moveToPoint(-41, 57, 1500, {false, 110, 10, 1});
// // robot.moveToPoint(-48, 55, 800, {false, 110, 10, 1});
// pros::delay(100);
// stopIntake();
//    robot.turnToHeading(0, 400, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    robot.waitUntilDone();
//    distance_reset();
//    robot.moveToPoint(-41, 45, 900, {false, 110, 10, 1});
//    robot.turnToHeading(270, 400, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    robot.moveToPoint(-25, 45, 700, {false, 110, 30, 1});
//       robot.waitUntilDone();
//       scoreUp();
//       pros::delay(2500);
//       stopIntake();     
//       loader.set_value(127);
//       robot.moveToPoint(-64, 46, 1100, {true, 60, 5, 1});
//       robot.waitUntil(5);
//       upScore.set_value(0);
//       loadBasket();
//       robot.waitUntilDone();
//       pros::delay(500);
//       distance_reset();
//       robot.moveToPoint(-55, 48, 700, {false, 80, 30, 1});
//       robot.moveToPoint(-62, 48, 1500, {true, 70, 10, 1});
//       robot.waitUntilDone();
//       pros::delay(800);
//          robot.moveToPoint(-26, 48, 900, {false, 110, 30, 1});
//       robot.waitUntilDone();
//       scoreUp();
//       pros::delay(2200);
//       stopIntake();
//       loader.set_value(0);
//       // robot.moveToPoint(-38, 48, 800, {true, 110, 10, 1});
//       // robot.waitUntilDone();
//       // upScore.set_value(0);
//       // robot.moveToPoint(-25, 48, 900, {false, 80, 20, 1});
//       //middle up scoring
//    robot.turnToHeading(170, 1000, {lemlib::AngularDirection::AUTO, 80, 10, 1});
//    robot.waitUntilDone();
//    upScore.set_value(0);
//    loadBasket_slower();
//    robot.moveToPoint(-23.45, 22.87, 2000, {true, 90, 10, 1});
//    robot.waitUntil(9.8);
//    loader.set_value(127);
//    robot.turnToHeading(-46, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    robot.moveToPoint(-8, 9.8147, 1000, {false, 70, 10, 01});
//    intakeStage3.move_voltage(-5000);
//    intakeStage2.move_voltage(-5000);
//    pros::delay(500);
//    intakeStage3.move_voltage(0);
//    intakeStage2.move_voltage(0);
//    robot.waitUntilDone();
//    scoreMiddle_slower();
//    loader.set_value(0);
//    pros::delay(2000);
//    //middle blocks 
//    robot.moveToPoint(-15.5, 13.5, 800, {true, 90, 10, 1});
//    robot.turnToHeading(-167, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    loadBasket();
//    robot.moveToPoint(-23.5, -23.5, 1000, {true, 100, 10, 1});
//    upScore.set_value(127);
//    robot.turnToHeading(-139, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    robot.moveToPoint(-40, -46, 1000, {true, 100, 10, 1});
//    robot.waitUntilDone();
//    stopIntake();
//    robot.turnToHeading(270, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    robot.waitUntilDone();
//    // robot.moveToPoint(-25, -45.5, 1000, {false, 100, 10, 1});
//    // robot.waitUntilDone();
//    // upScore.set_value(127);
//    // scoreUp();
//    // pros::delay(1500);
//    // stopIntake();
//    //    robot.moveToPoint(-40, 48, 2000, {true, 110, 10, 1});
//    //    robot.turnToHeading(180, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    //    robot.moveToPoint(-40, -47.5, 2000, {true, 110, 10, 1});
//    //    robot.turnToHeading(270, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});      
//       robot.moveToPoint(-65, -45.5, 900, {true, 60, 5, 1});
//       loader.set_value(127);
//       robot.waitUntil(10);
//       upScore.set_value(0);
//       loadBasket();
//       robot.waitUntilDone();
//       pros::delay(300);
//       robot.moveToPoint(-58, -47.5, 700, {false, 80, 30, 1});
//       robot.moveToPoint(-63, -47.5, 1500, {true, 70, 10, 1});
//       distance_reset();
//       robot.waitUntilDone();
//       pros::delay(450);
//    //    robot.moveToPoint(-48, -48, 2000, {false, 110, 10, 1});
//    //    robot.turnToHeading(210, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    //    loader.set_value(0);
//    //    robot.moveToPoint(-48, -58, 2000, {true, 110, 10, 1});
//    //    stopIntake();
//    //    robot.turnToHeading(90, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    //    //4 loader

//    robot.moveToPoint(-30, -60, 1200, {false, 100, 10, 1});
//    robot.turnToHeading(270, 600, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    stopIntake();
//    robot.moveToPoint(41, -59, 1500, {false, 100, 10, 1});
//    loader.set_value(0);
// //robot.moveToPoint(-48, 55, 800, {false, 110, 10, 1});
// intakeStage2.move_voltage(-12000);
// intakeStage3.move_voltage(-12000);
// pros::delay(100);
// stopIntake();
//    robot.turnToHeading(180, 400, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    robot.waitUntilDone();
//    distance_reset();
//    robot.moveToPoint(43, -45.5, 900, {false, 110, 10, 1});
//    robot.turnToHeading(90, 400, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    robot.moveToPoint(25, -45.5, 700, {false, 110, 30, 1});

//    robot.waitUntilDone();
//    scoreUp();
// //    //    robot.moveToPoint(38, -59, 1500, {true, 110, 10, 1});
// //    //    robot.moveToPoint(44, -52, 800, {true, 110, 10, 1});
// //    //    robot.turnToHeading(0, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
// //    //    robot.moveToPoint(44, -47, 2000, {true, 110, 10, 1});
// //    //    robot.turnToHeading(90, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
// //    //    robot.moveToPoint(22, -46.5, 1000, {false, 110, 10, 1});
// //    //    robot.waitUntilDone();
// //    //    scoreUp();
//       pros::delay(2000);
//       loader.set_value(127);
//       robot.moveToPoint(65, -46, 1500, {true, 60, 5, 1});
//       robot.waitUntil(7);
//       upScore.set_value(0);
//       loadBasket();
//       robot.waitUntilDone();
//       pros::delay(400);
//       robot.moveToPoint(58, -46, 700, {false, 80, 30, 1});
//       robot.moveToPoint(63, -46, 1500, {true, 70, 10, 1});
//       distance_reset();
//       robot.waitUntilDone();
//       pros::delay(600);
//       robot.moveToPoint(25, -48, 900, {false, 110, 30, 1});
//       robot.waitUntilDone();
//       scoreUp();
//       pros::delay(2500);
//       stopIntake();
//       loader.set_value(0);
//       robot.moveToPoint(35, -48, 900, {true, 110, 10, 1});
//       robot.waitUntilDone();
//       upScore.set_value(0);
//       robot.moveToPoint(25, -48, 1000, {false, 80, 10, 1});
// //middle blocks to 2nd middle middle   blyaaa kak ze ya ustal
//    robot.turnToHeading(-10, 1000, {lemlib::AngularDirection::AUTO, 80, 10, 1});
//    robot.waitUntilDone();
//    loadBasket_slower();
//    robot.moveToPoint(23.45, -22.87, 2000, {true, 90, 10, 1});
//    robot.waitUntil(9.8);
//    loader.set_value(127);
//    robot.turnToHeading(134, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    robot.moveToPoint(9.5, -8.8147, 1000, {false, 70, 10, 01});
//    intakeStage3.move_voltage(-5000);
//    intakeStage2.move_voltage(-5000);
//    pros::delay(500);
//    intakeStage3.move_voltage(0);
//    intakeStage2.move_voltage(0);
//    robot.waitUntilDone();
//    scoreMiddle_slower();
//    loader.set_value(0);
//    pros::delay(2000);
//    stopIntake();
//       robot.moveToPose(64, -28, 0, 4000, {true, 8, 0.4, 100, 10, 1});
//       robot.turnToHeading(0, 400, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//       loader.set_value(127);
//       pros::delay(400);
//       encoder_upper.set_value(127);
//       intakeStage1.move_voltage(-12000);
//       robot.moveToPoint(66, 3, 3000, {true, 120, 120});
//    //    upScore.set_value(0);
//    //    robot.turnToHeading(315, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    //    loadBasket();
//    //    robot.moveToPoint(24, -22, 1500, {true, 120, 10, 1});
//    //    robot.turnToHeading(135, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    //    robot.moveToPoint(13, -9.5, 900, {false, 110, 30, 1});
//    //    robot.waitUntilDone();
//    //    scoreMiddle();
//    //    pros::delay(1050);
//    //    stopIntake();
//    //    robot.moveToPoint(68, -25, 1800, {true, 120, 10, 1});
//    //    robot.turnToHeading(0, 800, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    //    robot.waitUntilDone();
//    //    loader.set_value(127);

//    //    robot.moveToPoint(72, 22, 2000, {true, 127, 120, 1});
//    //       robot.waitUntilDone();
//    //    loader.set_value(0);
//    //    scoreUp();

  
// }


// void skills(){
//    robot.setPose(56, 0, 90);
//    //park clearing
//    encoder_upper.set_value(127);
//    descore.set_value(127);
//    loadBasket();
//    pros::delay(200);
//    robot.tank(80, 80);
//    pros::delay(600); 
//    robot.tank(-80, -80);
//    pros::delay(300);
//    robot.tank(90, 100);
//    pros::delay(1000);
//    // robot.tank(100, 110);
//    // pros::delay(500);
//    robot.tank(-80, -90);
//    pros::delay(400);
//    robot.tank(0, 0);
//    pros::delay(200);
//    robot.tank(95, 95);
//    pros::delay(1000);
//    robot.tank(0, 0);
//    pros::delay(200);
//    // robot.tank(-80, -80);
//    // pros::delay(300);
//    // robot.tank(0, 0);
//    // pros::delay(500);
//    // robot.tank(90, 90);
//    // pros::delay(600);
//    robot.tank(-120, -120);
//    pros::delay(400);
//    robot.tank(0, 0);
//    pros::delay(200);
//    robot.tank(60, 60);
//    pros::delay(1500);
//    robot.tank(0, 0);
//    pros::delay(500);
//    encoder_upper.set_value(0);
//    stopIntake();
//    pros::delay(300);
//    distance_reset_for_skills_beginning();
//    pros::delay(100);
//    //low goal
//    moveStraight(-5, 800, {true, 80, 10, 1});
//    robot.turnToHeading(-29, 450, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    robot.waitUntilDone();
//    intakeStage1.move(2000);
//    robot.moveToPoint(26, 26, 1000, {true, 100, 10, 1});
//    pros::delay(100);
//    robot.turnToHeading(-130, 450, {lemlib::AngularDirection::AUTO, 127, 10, 1});
//    robot.moveToPoint(9.5, 13, 1000, {true, 90, 5, 1});
//    robot.waitUntilDone();
//    loader.set_value(127);
//    pros::delay(50);
//    intake_upper.set_value(127);
//    intakeStage1.move(-5000);
//    pros::delay(500);
//    scoreMiddleDown(); }

void skills() {
   robot.setPose(47, -8, 270);
   park_opt.set_led_pwm(60);
   optical.set_led_pwm(100);
   optical.set_integration_time(5.0);

   //score middle 2 blocks
   loadBasket();
   middle_descore.set_value(1);
   robot.moveToPoint(24, -25, 1500, {true, 115, 5, 1});
   pros::delay(560);
   loader.set_value(1);

   robot.turnToHeading(135, 450, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   intakeStage3.move(-4000);
   intakeStage2.move(-4000);
   pros::delay(300);
   intakeStage3.move(0);
   intakeStage2.move(0);


   robot.moveToPoint(11, -11, 1000, {false, 115, 10, 1});
   robot.waitUntilDone();
   loader.set_value(0);
   upScore.set_value(1);
   scoreMiddle_slower();
   pros::delay(2200);
   stopIntake();

   //1st loader
   robot.moveToPoint(44, -45, 1200, {true, 110, 10, 1});
   robot.waitUntil(8);
   upScore.set_value(0);
   middle_descore.set_value(0);
   robot.turnToHeading(90, 450, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.waitUntilDone();
   loader.set_value(1);
   loadBasket();
   robot.moveToPoint(67, -47, 1300, {true, 55, 0, 1});
   robot.waitUntilDone();
   // robot.tank(70, 70);
   // pros::delay(400);
   robot.tank(-70, -70);
   pros::delay(200);
   robot.tank(70, 70);
   pros::delay(1700);
   robot.tank(0, 0);
   // intakeStage1.move(2000);
   // stopIntake();
   keep_intake_alive();
   distance_reset();

   //1st long goal
   robot.moveToPoint(31, -60, 1000, {false, 110, 10, 1});
   robot.waitUntilDone();
   // stopIntake();
   // loader.set_value(0);
   distance_reset();
   robot.turnToHeading(90, 450, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.moveToPoint(-22, -60, 1200, {false, 110, 10, 1});
   robot.waitUntilDone();
   loader.set_value(0);
   distance_reset();
   robot.moveToPoint(-38, -50, 1200, {false, 110, 10, 1});

   robot.turnToHeading(270, 450, {lemlib::AngularDirection::AUTO, 127, 10, 0});
   robot.waitUntilDone();
   upScore.set_value(1);
   robot.moveToPoint(-20, -50, 900, {false, 100, 10, 1});
   robot.waitUntilDone();
   robot.waitUntilDone();
   robot.tank(-100, -100);
   scoreUp();
   pros::delay(1800);
   robot.tank(-0, -0);
   loader.set_value(1);

   //2nd loader
   robot.moveToPoint(-68, -49, 1400, {true, 60, 0, 1});
   robot.waitUntil(5);
   loadBasket();
   upScore.set_value(0);
   robot.waitUntilDone();
   // robot.tank(60, 60);
   // pros::delay(400);
   robot.tank(-70, -70);
   pros::delay(200);
   robot.tank(60, 60);
   pros::delay(1700);
   robot.tank(0, 0);

   keep_intake_alive();
   distance_reset();


   //2nd long goal
   robot.moveToPoint(-26, -48, 900, {false, 110, 10, 1});
   robot.waitUntilDone();
   scoreUp_slower();
   pros::delay(2000);
   loader.set_value(0);
   distance_reset();


   // // //park clearing
   // robot.moveToPoint(-64, -26, 900, {true, 100, 10, 1});
   // robot.turnToHeading(340, 400, {lemlib::AngularDirection::AUTO, 90, 0, 0});

   // robot.waitUntilDone();
   // upScore.set_value(0);

//    robot.setPose(-61.67, -27, 337),
   
//    loadBasket();
//    encoder_upper.set_value(127);

//    robot.tank(127,127);
//    pros::delay(100);
//    loader.set_value(1);
//    pros::delay(300);
//    loader.set_value(0);
//    robot.tank(100,100);
//    pros::delay(300);
//       robot.tank(120,120);
//    pros::delay(400);
//       loader.set_value(1);
//    pros::delay(300);

//    robot.tank(0,0);
//    pros::delay(500);
//    loader.set_value(0);

//    // stopIntake();
//    // distance_reset();




//    robot.tank(-80,-95);
//    encoder_upper.set_value(0);
//    pros::delay(1300);
//    robot.tank(0,0);
//    pros::delay(900);


//    robot.turnToHeading(90, 1000, {lemlib::AngularDirection::AUTO, 100, 10, 1});
//    pros::delay(300);

// distance_reset_for_skills_beginning();
//    pros::delay(1000);


//    //scoring middle goal

//    // robot.moveToPoint(-63, 20, 500, {true, 100, 10, 1});
//    // robot.waitUntilDone();
//    stopIntake();
// //comment ends
//    robot.waitUntilDone();
//    intakeStage1.move(10000);
//    robot.moveToPoint(-27, 22, 900, {true, 115, 0, 1});
//    robot.waitUntilDone();
//    middle_descore.set_value(1);
//    intakeStage1.move(0);

   // robot.turnToHeading(314, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1}, false);
   // robot.moveToPose(-9, 9, 314, 1100, {false, 2, 0.1, 120, 20, 0});

   // robot.waitUntilDone();
   // robot.tank(-70, -70);
   // upScore.set_value(1);
   // intakeStage3.move_voltage(-12000);
   // intakeStage2.move_voltage(-12000);
   // intakeStage1.move_voltage(-1000);
   // loader.set_value(1);

   // pros::delay(300);
   // intakeStage3.move_voltage(0);
   // intakeStage2.move_voltage(0);
   // intakeStage1.move_voltage(0);
   // pros::delay(200);











   // scoreMiddle_slower();
   // pros::delay(2800);
   // robot.tank(0, 0);
   // stopIntake();

    
   // // // wait_blue();




   // //3rd loader



   loadBasket();
   robot.turnToHeading(350, 600, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.waitUntilDone();
   upScore.set_value(0);
   robot.moveToPoint(-24, -27, 1500, {true, 80, 10, 1});
   // robot.moveToPose(-23, -27, 0, 1100, {true, 2, 0.2, 110, 10, 0});

   pros::delay(400);
   loader.set_value(1);
   pros::delay(400);
   loader.set_value(0);
   robot.turnToHeading(45, 600, {lemlib::AngularDirection::AUTO, 127, 10, 1});

   robot.moveToPose(-12, -11, 45, 800, {true, 2, 0.2, 127, 20, 1});
   robot.waitUntilDone();
   loader.set_value(1);
   pros::delay(100);

   intake_upper.set_value(1);
   scoreMiddleDown();
   pros::delay(1500);
   stopIntake();
   // intake_upper.set_value(0);
   pros::delay(50);
   loader.set_value(0);
   // robot.turnToHeading(0, 600, {lemlib::AngularDirection::AUTO, 127, 10, 1});

   robot.moveToPoint(-27, -26, 900, {false, 115, 10, 1});
   robot.waitUntilDone();
   intake_upper.set_value(0);

   robot.moveToPoint(-28, 19,1500, {true, 110, 10, 1});
   loadBasket();
   pros::delay(1000);
   loader.set_value(1);
   pros::delay(500);
   // loader.set_value(0);


   // robot.moveToPose(-15, 9, 318, 2000, {false, 2, 0.25, 127, 10, 1});
   // robot.waitUntil(10);
   // intakeStage3.move_voltage(-3000);
   // intakeStage2.move_voltage(-3000);
   // // intakeStage1.move_voltage(-2000);
   // robot.waitUntilDone();

   // // middle_descore.set_value(1);
   // upScore.set_value(1);


   // pros::delay(100);
   // scoreMiddle_slower();









   robot.turnToHeading(330, 450, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.moveToPoint(-48, 45, 1300, {true, 110, 10, 1});
   robot.turnToHeading(270, 450, {lemlib::AngularDirection::AUTO, 127, 10, 1});

   robot.moveToPoint(-26, 45, 1200, {false, 100, 10, 1});
   robot.waitUntilDone();
   scoreUp_slower();
   pros::delay(1400);
   // loader.set_value(1);
   distance_reset();





   // robot.moveToPoint(-42, 48, 2500, {true, 110, 10, 1});
   // robot.waitUntil(8);
   // middle_descore.set_value(0);
   // upScore.set_value(0);
   // robot.turnToHeading(270, 450, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   // robot.waitUntilDone();
   // loader.set_value(1);

   loadBasket();

   robot.moveToPoint(-67, 47, 1300, {true, 55, 0, 1});
   robot.waitUntil(8);
   middle_descore.set_value(0);
   upScore.set_value(0);
   robot.waitUntilDone();

   robot.tank(-70, -70);
   pros::delay(200);
   robot.tank(90, 90);
   pros::delay(2000);
   robot.tank(0, 0);
   keep_intake_alive();
   distance_reset();


   //3nd long goal
   robot.moveToPoint(-31, 62, 1000, {false, 100, 10, 1});
   robot.waitUntilDone();
   distance_reset();
   robot.turnToHeading(270, 450, {lemlib::AngularDirection::AUTO, 127, 10, 0});
   robot.moveToPoint(22, 63, 1300, {false, 110, 10, 1});
   robot.waitUntilDone();
   loader.set_value(0);
   distance_reset();
   robot.moveToPoint(42, 48, 900, {false, 110, 10, 1});

   robot.turnToHeading(90, 450, {lemlib::AngularDirection::AUTO, 127, 10, 0});
   robot.waitUntilDone();
   upScore.set_value(1);
   robot.moveToPoint(22, 49, 950, {false, 100, 10, 1});
   robot.waitUntilDone();
   scoreUp();
   pros::delay(2200);
   stopIntake();
   loader.set_value(1);



   //4th loader
   robot.moveToPoint(67, 49, 2000, {true, 50, 0, 1});
   robot.waitUntil(5);
   loadBasket();
   upScore.set_value(0);
   robot.waitUntilDone();
   robot.tank(70, 70);
   pros::delay(200);
   robot.tank(-70, -70);
   pros::delay(200);
   robot.tank(90, 90);
   pros::delay(1800);
   robot.tank(0, 0);

   stopIntake();
   distance_reset();


   //4th long goal
   robot.moveToPoint(26, 49.5, 900, {false, 100, 10, 1});
   robot.waitUntilDone();
   scoreUp_slower();
   pros::delay(2000);
   loader.set_value(0);
   distance_reset();


   //park clearing

   robot.moveToPoint(63, 26, 900, {true, 120, 50, 1});
   robot.turnToHeading(165, 400, {lemlib::AngularDirection::AUTO, 127, 127, 1});

   robot.waitUntilDone();
   upScore.set_value(0);

   // // robot.setPose(-60, -28, 353);
   intakeStage1.move_voltage(-12000);
   intakeStage2.move_voltage(-12000);
   intakeStage3.move_voltage(-12000);   
   loader.set_value(1);
   encoder_upper.set_value(127);

   robot.moveToPoint(68, 4, 5000, {true, 127, 127});
   robot.waitUntilDone();
   // // park_red();
   loader.set_value(0);

   stopIntake();
}

void awp() {
   descore.set_value(127);
   robot.setPose(48, 10, 0);
   robot.moveToPoint(48, 45.5, 1000, {true, 127, 90, 1});
   robot.turnToHeading(90, 450, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   loader.set_value(127);
   robot.moveToPoint(63, 47.5, 1100, {true, 60, 1, 1});
   loadBasket();
   robot.waitUntilDone();
   robot.moveToPoint(25, 47.5, 1000, {false, 127, 80, 1});
   robot.waitUntilDone();
   distance_reset();
   scoreUp();
   pros::delay(900);
   stopIntake();
   loader.set_value(0);
   robot.waitUntilDone();
   intakeStage2.move_voltage(-12000);
   intakeStage3.move_voltage(-12000);
   robot.turnToHeading(180, 600, {lemlib::AngularDirection::AUTO, 127, 15, 1});
   loadBasket();
   robot.moveToPoint(26, 28, 1200, {true, 120, 80, 1});
   upScore.set_value(0);

   pros::delay(100);
   robot.moveToPoint(26.5, -17, 950, {true, 127, 60, 1});
   pros::delay(800);
   loader.set_value(127);
   pros::delay(300);
   loader.set_value(0);
   robot.turnToHeading(105, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   // robot.moveToPoint(43, -33, 500, {true, 127, 110, 1});
   robot.moveToPoint(50, -44, 850, {true, 127, 110, 1});
   robot.turnToHeading(90, 450, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.moveToPoint(26.5, -45, 800, {false, 120, 70, 1});
   robot.waitUntilDone();
   distance_reset();
   scoreUp();
   pros::delay(1100);
   stopIntake();
   loader.set_value(127);
   robot.moveToPoint(65, -46, 1500, {true, 60, 1, 1});

   intakeStage1.move_voltage(12000);
   intakeStage2.move_voltage(2000);
   robot.moveToPoint(14.5, -13, 1100, {false, 127, 10, 0});
      pros::delay(300);
   intakeStage1.move_voltage(-12000);
   intakeStage2.move_voltage(-12000);
   intakeStage3.move_voltage(-12000);
   stopIntake();
   upScore.set_value(0);
   robot.waitUntilDone();
   scoreMiddle(); 
   loader.set_value(0);

}

void rushleft(){
   descore.set_value(127);
   robot.setPose(50, -7, 270);
   loadBasket();
   robot.moveToPoint(24, -23, 1500, {true, 127, 100, 1});
   pros::delay(450);
   loader.set_value(127);
   robot.turnToHeading(141, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   loader.set_value(0);
   descore.set_value(127);
   robot.moveToPoint(41, -47.6, 800, {true, 127, 100, 1});   
   robot.turnToHeading(90, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.moveToPoint(26, -47.6, 700, {false, 120, 100, 1});
   robot.waitUntilDone();
   scoreUp();
   pros::delay(1000);
   stopIntake();  
   robot.moveToPoint(45, -45, 1000, {true, 127, 100, 1}); 
   robot.turnToHeading(147, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.moveToPoint(30, -37, 1000, {false, 120, 100, 1});
   robot.turnToHeading(90, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   descore.set_value(0);
   robot.moveToPoint(12, -36.5, 1000000, {false, 120, 100, 1});
}
void long9right(){
   descore.set_value(127);
   robot.setPose(50, 7, 270);
   loadBasket();
   robot.moveToPoint(24, 23, 1500, {true, 127, 10, 1});
    pros::delay(650);
   loader.set_value(127);
   pros::delay(300);
   loader.set_value(0); 
   robot.moveToPoint(13, 41.5, 1500, {true, 90, 10, 1}); 
   pros::delay(600);
   loader.set_value(127);
   robot.moveToPoint(35, 32, 900, {false, 127, 40, 1});   
   robot.waitUntilDone();
   intakeStage1.move_voltage(-12000);
   intakeStage2.move_voltage(-12000);
   intakeStage3.move_voltage(-12000);
   pros::delay(100);
   stopIntake();
   robot.moveToPoint(45, 46, 1000, {true, 127, 40, 1});   
   robot.turnToHeading(90, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.moveToPoint(26, 46, 1000, {false, 120, 40, 1});
   robot.waitUntilDone();  

   scoreUp();
   pros::delay(1600);
   stopIntake();   
   robot.moveToPoint(64, 47, 1500, {true, 70, 10, 1});
   loadBasket();
   upScore.set_value(0);
   
   robot.waitUntilDone();
   distance_reset();
   robot.moveToPoint(26, 47, 900, {false, 120, 30, 1});
   robot.waitUntilDone();  
   scoreUp();
   pros::delay(1300);
   stopIntake();   
   robot.moveToPoint(37, 48, 900, {true, 120, 30, 1});
   robot.moveToPoint(26, 48, 900, {false, 120, 100, 1});
   upScore.set_value(0);


   // pros::delay(1000);
   // stopIntake();
   // robot.moveToPoint(-25, 49, 900, {false, 127, 10, 1});   
   // robot.waitUntilDone();
   // scoreUp();
   // pros::delay(1700);
   // stopIntake();
   // loader.set_value(1); 
   // robot.moveToPoint(-62.5, 48, 1200, {true, 60, 5, 1});
   // loadBasket();
   // robot.waitUntilDone();

   // robot.moveToPoint(-53, 48, 1200, {false, 80, 10, 1});
   // robot.waitUntilDone();
   // robot.moveToPoint(-63, 48, 900, {true, 70, 10, 1});
   
   // // // wait_red();



   // robot.moveToPoint(-26, 49, 1200, {false, 120, 10, 1});
   // robot.waitUntilDone();
   // loader.set_value(0); 
   // scoreUp();
   // pros::delay(1400);
   
   // robot.moveToPoint(-35, 49, 1200, {true, 127, 40, 1});
   // robot.waitUntilDone();
   // robot.moveToPoint(-26, 49, 1200, {false, 127, 120, 1});









   // robot.turnToHeading(345, 700, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   // robot.moveToPoint(-32, 35.8, 1200, {false, 127, 10, 1});
   
   // robot.turnToHeading(270,500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   // descore.set_value(0);
   // robot.moveToPoint(-15.2, 35.4, 1200, {false, 127, 30, 1});
}

void long9left(){
   robot.setPose(50, -7, 270);
   loadBasket();
   robot.moveToPoint(24, -25, 800, {true, 127, 90, 1});
  descore.set_value(127);
    pros::delay(550);
      loader.set_value(127);
   pros::delay(300);
   loader.set_value(0); 

   // robot.turnToHeading(203, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.moveToPoint(17, -42, 300, {true, 127, 10, 1}); 
   robot.moveToPoint(9.8, -43.7, 400, {true, 127, 10, 1}); 
   pros::delay(270);
   loader.set_value(127);
   robot.moveToPoint(22, -23, 600, {false, 127, 80, 1});   
   // robot.waitUntilDone();
   robot.turnToHeading(133, 200, {lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1});

   
   robot.moveToPoint(45, -48, 800, {true, 127, 40, 1});   
   robot.turnToHeading(90, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.waitUntilDone();
   distance_reset();
   pros::delay(100);
   robot.moveToPoint(62, -48, 1500, {true, 70, 5, 1});
   loadBasket();
   upScore.set_value(0);

   pros::delay(200);
   robot.moveToPoint(23, -47.5, 700, {false, 120, 10, 1});
   robot.waitUntilDone();  
   robot.tank(-50,-50);
   scoreUp(); 


}

void rushleft6()
{
   robot.setPose(50, -7, 270);
   loadBasket();
   robot.moveToPoint(24, -25, 800, {true, 127, 90, 1});
  descore.set_value(127);
    pros::delay(550);
      loader.set_value(127);
   pros::delay(300);
   loader.set_value(0); 

   // robot.turnToHeading(203, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.moveToPoint(17, -42, 300, {true, 127, 10, 1}); 
   robot.moveToPoint(10.2, -44, 400, {true, 127, 10, 1}); 
   pros::delay(270);
   loader.set_value(127);
   robot.moveToPoint(22, -30, 600, {false, 127, 80, 1});   
   // robot.waitUntilDone();
   robot.turnToHeading(133, 200, {lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, 127, 10, 1});

   robot.moveToPoint(38, -48, 800, {true, 127, 80, 1});   
   robot.turnToHeading(90, 300, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.moveToPoint(22, -47.5, 700, {false, 120, 10, 1});
   loader.set_value(0);
   robot.waitUntilDone();  
   distance_reset();
   scoreUp();
   pros::delay(1100);
   stopIntake();
   robot.moveToPoint(40, -48, 800, {true, 127, 80, 1});  
   robot.turnToHeading(150, 200, {lemlib::AngularDirection::AUTO, 127, 10, 1});

   robot.moveToPoint(32, -37.5, 800, {false, 127, 10,1});  
   robot.turnToHeading(90, 200, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.waitUntilDone();
   descore.set_value(0);
   robot.moveToPoint(14, -38, 600, {false, 127, 10,1});  
   robot.turnToHeading(120, 200, {lemlib::AngularDirection::AUTO, 127, 127, 0});
   robot.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

      robot.turnToHeading(140, 20000, {lemlib::AngularDirection::AUTO, 20, 0, 0});




}
void awp_middle_push()
{
//loader
   descore.set_value(127);
   robot.setPose(-48, -10, 180);
   robot.moveToPoint(-48, -46.5, 900, {true, 120, 10, 1});
   robot.turnToHeading(270, 450, {lemlib::AngularDirection::AUTO, 127, 30, 1});
   robot.waitUntilDone();
   distance_reset_FL();


   loader.set_value(1);
   robot.moveToPoint(-63, -48, 1100, {true, 40, 1, 1});
   loadBasket();
   robot.waitUntilDone();
   distance_reset_FL();

   robot.waitUntilDone();
   //score long goal
   robot.moveToPoint(-25, -48, 900, {false, 120, 5, 1});
   robot.waitUntilDone();
   distance_reset();
   scoreUp();
   pros::delay(1100);
   stopIntake();
   loader.set_value(0);
   robot.turnToHeading(5, 500, {lemlib::AngularDirection::AUTO, 127, 50, 1});
   robot.waitUntilDone();
   intakeStage1.move_voltage(12000);
   intakeStage2.move_voltage(1000);
   intakeStage3.move_voltage(1000);
   upScore.set_value(0);
   //3 blocks
   robot.moveToPoint(-25, -25, 800, {true, 100, 5, 1});
   pros::delay(400);
   loader.set_value(1);
   pros::delay(400);
   loader.set_value(0);
   // intakeStage2.move_voltage(-12000);
   // intakeStage3.move_voltage(-12000);
   // pros::delay(200);
   // stopIntake();
   robot.turnToHeading(45, 500, {lemlib::AngularDirection::AUTO, 120, 10, 1});
   robot.waitUntilDone();
   //low goal
   robot.moveToPoint(-11, -11, 800, {true, 120, 5, 1});
   robot.waitUntilDone();
   loader.set_value(1);
   pros::delay(200);

   intake_upper.set_value(1);
   scoreMiddleDown();
   pros::delay(1200);
   stopIntake();
   intake_upper.set_value(0);
   //3 blocks
   robot.moveToPoint(-27, -26, 900, {false, 120, 10, 1});
   robot.moveToPoint(-26, 23,1300, {true, 120, 10, 1});
   loader.set_value(0);
   
   loadBasket();
   pros::delay(850);
   loader.set_value(127);
   pros::delay(500);
   loader.set_value(0);
   //middle score
   middle_descore.set_value(1); 
   robot.turnToHeading(315, 500, {lemlib::AngularDirection::AUTO, 127, 50, 1});
   robot.moveToPoint(-9, 8,1100, {false, 120, 10, 0});
   robot.turnToHeading(315, 500, {lemlib::AngularDirection::AUTO, 127, 50, 1});
   robot.waitUntilDone();
   upScore.set_value(1);
   intakeStage3.move_voltage(-12000);
   intakeStage2.move_voltage(-12000);
   pros::delay(200);
   scoreMiddle_slower();
   pros::delay(1800);
   stopIntake();
   robot.moveToPoint(-18, 15,600, {true, 120, 100, 1});
   robot.waitUntilDone();
 middle_descore.set_value(0); 
   robot.moveToPoint(-11, 9,1100, {false, 90, 10, 1});
}
void awp_middle(){
   //loader
   descore.set_value(1);
   robot.setPose(-48, -17.5, 180);
   robot.moveToPoint(-48, -46, 700, {true, 127, 30, 1});
   robot.turnToHeading(270, 400, {lemlib::AngularDirection::AUTO, 127, 127, 5});
   robot.waitUntilDone();
   distance_reset_FL();
   loader.set_value(1);
   robot.moveToPoint(-72, -49, 1200, {true, 60, 5, 1});
   loadBasket();
   robot.waitUntilDone();
   distance_reset_FL();
   robot.waitUntilDone();
   //score long goal
   robot.moveToPoint(-25, -48, 700, {false, 120, 30, 4});
   robot.waitUntilDone();
   robot.tank(-127, -127);

   distance_reset_FL();

   scoreUp();
   pros::delay(950);
   robot.tank(0, 0);

   stopIntake();
   loader.set_value(0);
   robot.turnToHeading(0, 500, {lemlib::AngularDirection::AUTO, 127, 10, 1});
   robot.waitUntilDone();
   intakeStage1.move_voltage(12000);
   intakeStage2.move_voltage(1000);
   intakeStage3.move_voltage(500);
   upScore.set_value(0);
   //3 blocks
   robot.moveToPose(-28, -26, 5, 700, {true, 2, 0.2, 127, 40, 1});
   pros::delay(400);
   loader.set_value(1);
   pros::delay(400);
   loader.set_value(0);
   // intakeStage2.move_voltage(-12000);
   // intakeStage3.move_voltage(-12000);
   // pros::delay(200);
   // stopIntake();
   // robot.turnToHeading(45, 500, {lemlib::AngularDirection::AUTO, 127, 30, 1});
   // robot.waitUntilDone();
   // //low goal
   // robot.moveToPoint(-11, -10, 700, {true, 120, 80, 1});

   robot.moveToPose(-13, -10, 45, 800, {true, 2, 0.2, 127, 20, 1});


   robot.waitUntilDone();
   loader.set_value(1);
   pros::delay(100);

   intake_upper.set_value(1);
   scoreMiddleDown();
   pros::delay(1300);
   stopIntake();
   // intake_upper.set_value(0);
   pros::delay(50);
   loader.set_value(0);


   //3 blocks
   robot.moveToPoint(-27, -26, 900, {false, 115, 30, 1});
   robot.waitUntilDone();
   intake_upper.set_value(0);

   robot.moveToPoint(-28, 18,1500, {true, 110, 20, 1});
   
   loadBasket();
   pros::delay(850);
   loader.set_value(1);
   pros::delay(500);
   loader.set_value(0);
   //2 blocks from line
   // pros::delay(1200);
   robot.turnToHeading(31, 450, {lemlib::AngularDirection::CW_CLOCKWISE, 127, 50, 1});
   robot.moveToPoint(-15.5 , 39,1300, {true, 115, 10, 1});
  
   pros::delay(500);
   loader.set_value(1);
   robot.turnToHeading(40, 500, {lemlib::AngularDirection::AUTO, 127, 50, 1});

   //middle score
   middle_descore.set_value(1); 

   // robot.moveToPoint(-9, 8,1100, {false, 120, 10, 0});
   // robot.turnToHeading(315, 500, {lemlib::AngularDirection::AUTO, 127, 50, 1});
      
   
   robot.moveToPose(-15, 9, 318, 1400, {false, 2, 0.25, 127, 70, 5});
   robot.waitUntil(10);
   intakeStage3.move_voltage(-3000);
   intakeStage2.move_voltage(-3000);
   // intakeStage1.move_voltage(-2000);
   robot.waitUntilDone();

   // middle_descore.set_value(1);
   upScore.set_value(1);


   pros::delay(100);
   scoreMiddle_slower();

   // pros::delay(1100);
   // stopIntake();
   //push
//    robot.moveToPoint(-16, 11,600, {true, 120, 100, 1});
//    robot.waitUntilDone();
//  middle_descore.set_value(1); 
//    robot.moveToPoint(-12, 7.5,1100, {false, 100, 10, 0});
}
void rushright6undergoal(){

   robot.setPose(-47, -11, 120);
   loadBasket();
   // scoreMiddle_slower();

   robot.moveToPoint(-23, -24, 900, {true, 127, 127, 13}, false);

   descore.set_value(1);

   robot.moveToPose(-5, -49, 155, 800, {true, 2, 0.25, 127, 20, 14});
   pros::delay(750);
   loader.set_value(1);
   // pros::delay(350);
   // loader.set_value(0); 

   // robot.turnToHeading(70, 500, {lemlib::AngularDirection::AUTO, 127, 10, 0});
   robot.moveToPoint(-15, -38, 400, {false, 127, 80, 1});
   // robot.waitUntilDone();
   // loader.set_value(0); 
   robot.turnToHeading(65, 400, {lemlib::AngularDirection::AUTO, 127, 50, 5},false);
   robot.moveToPoint(-20, -38, 400, {false, 127, 127, 2}, false);


   robot.tank(-127, -80);
   pros::delay(1100);
   robot.tank(-127, -127);
   loader.set_value(0);

   scoreUp();
   distance_reset_FL();
   pros::delay(1400);
   robot.tank(0, 0);


    


   // robot.setPose(-26, -47, 270);
   // pros::delay(200);
   // distance_reset_FL();

   robot.turnToHeading(230, 400, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.moveToPoint(-36, -58, 500, {true, 127, 127, 5});
   robot.turnToHeading(270, 400, {lemlib::AngularDirection::AUTO, 127, 127, 1});
   robot.waitUntilDone();
   descore.set_value(0);

   robot.moveToPoint(-17, -59, 2000, {false, 127, 127, 0});
   robot.turnToHeading(290, 100000, {lemlib::AngularDirection::AUTO, 10, 5, 0});
}
void rushright_6loader(){
   robot.setPose(-50, -16, 109);
   loadBasket();
   // scoreMiddle_slower();

   robot.moveToPoint(-25, -22, 1000, {true, 127, 80, 10});

   pros::delay(450);


   // 
   // robot.moveToPose(-23, -26, 140, 900, {true, 3, 0.2, 127, 120, 10}, false);

   descore.set_value(1);
   loader.set_value(1);

   robot.swingToHeading(220, lemlib::DriveSide::RIGHT, 400, {lemlib::AngularDirection::AUTO, 127, 100, 20});
   robot.moveToPose(-51, -47, 270, 1200, {true, 2, 0.1, 127, 90, 10});
   robot.waitUntilDone();
   distance_reset_FL();

   robot.moveToPoint(-66, -47, 750, {true, 120, 10, 1});
   robot.waitUntilDone();
   distance_reset_FL();
      
   robot.moveToPoint(-25, -48, 700, {false, 120, 80, 1});
   robot.waitUntilDone();
   robot.tank(-127, -127);
   loader.set_value(0);
   distance_reset_FL();
   scoreUp();
   pros::delay(1100);
   robot.tank(0, 0);


   robot.turnToHeading(220, 350, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.moveToPoint(-37, -58, 400, {true, 127, 127, 3});
   // robot.turnToHeading(270, 400, {lemlib::AngularDirection::AUTO, 127, 127, 1});
   robot.waitUntilDone();
   descore.set_value(0);
   loader.set_value(0);


   // robot.moveToPoint(-17, -59, 2000, {false, 127, 127, 0});

   robot.moveToPose(-13, -58, 270, 1600, {false, 2, 0.1, 90, 10, 0});

   robot.turnToHeading(290, 100000, {lemlib::AngularDirection::AUTO, 10, 5, 0});
}
void rushright9() {
   robot.setPose(-50, -16, 109);
   loadBasket();
   // scoreMiddle_slower();
   robot.moveToPoint(-25, -24, 900, {true, 127, 60, 10}, false);
   loader.set_value(1);
   descore.set_value(1);
   pros::delay(300);
   loader.set_value(0);


   robot.moveToPose(-3, -48, 145, 1000, {true, 2, 0.25, 127, 10, 5});
   pros::delay(700);
   loader.set_value(1);
   // pros::delay(350);
   // loader.set_value(0); 

   // robot.turnToHeading(70, 500, {lemlib::AngularDirection::AUTO, 127, 10, 0});
   robot.moveToPoint(-20, -30, 680, {false, 127, 127, 1});

   robot.moveToPose(-50, -46, 270, 1400, {true, 2, 0.2, 127, 80, 16});
   robot.waitUntilDone();
   distance_reset_FL();

   robot.moveToPoint(-63, -49, 900, {true, 70, 10, 1});
   robot.waitUntilDone();
   distance_reset_FL();

   robot.moveToPoint(-25, -48, 700, {false, 127, 80, 1});
   robot.waitUntilDone();
   robot.tank(-127, -127);
   loader.set_value(0);
   distance_reset_FL();
   scoreUp();
   pros::delay(1700);
   robot.tank(0, 0);


   robot.turnToHeading(230, 400, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.moveToPoint(-36, -58, 500, {true, 127, 127, 5});
   robot.turnToHeading(270, 400, {lemlib::AngularDirection::AUTO, 127, 127, 1});
   robot.waitUntilDone();
   upScore.set_value(0);
   descore.set_value(0);
   // loader.set_value(0);

robot.moveToPose(-13, -57, 270, 1200, {false, 2, 0.1, 127, 70, 0});   
// robot.turnToHeading(280, 500, {lemlib::AngularDirection::AUTO, 100, 100, 0});
   robot.swingToHeading(290, lemlib::DriveSide::RIGHT, 700, {lemlib::AngularDirection::AUTO, 70, 10, 0});

}
void solo_awp_15_right_1longgoal_9() {
   robot.setPose(-47, -11, 118);
   loadBasket();

   robot.moveToPoint(-24.5, -24, 900, {true, 127, 127, 13});

   descore.set_value(1);

   robot.moveToPose(-4, -51, 152, 800, {true, 2, 0.25, 127, 50, 14});
   pros::delay(750);
   loader.set_value(1);
   // pros::delay(350);
   // loader.set_value(0); 

   // robot.turnToHeading(70, 500, {lemlib::AngularDirection::AUTO, 127, 10, 0});
   robot.moveToPoint(-20, -30, 1000, {false, 127, 127, 3});

   robot.moveToPose(-51, -46, 270, 1400, {true, 2, 0.3, 127, 80, 16});

   distance_reset_FL();

   robot.moveToPoint(-63, -50, 800, {true, 80, 5, 1});
   robot.waitUntilDone();
   distance_reset_FL();

   robot.moveToPoint(-25, -48, 700, {false, 127, 80, 1});
   robot.waitUntilDone();
   robot.tank(-127, -127);
   loader.set_value(0);
   distance_reset_FL();
   scoreUp();
   pros::delay(1800);
   robot.tank(0, 0);
   loadBasket_slower();

   robot.turnToHeading(360, 400, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.waitUntilDone();
   upScore.set_value(0);
   robot.moveToPoint(-28, 14, 1400, {true, 120, 120, 5});
   robot.waitUntilDone();
   loader.set_value(1);
   // robot.turnToHeading(315, 350, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   middle_descore.set_value(1);
   robot.swingToHeading(316, lemlib::DriveSide::LEFT, 400, {lemlib::AngularDirection::AUTO, 127, 80, 5});
   robot.moveToPoint(-13, 7, 700, {false, 120, 90, 1});
   robot.waitUntilDone();
   
   scoreMiddle_slower();
   pros::delay(1700);
   loadBasket();
   
   robot.moveToPoint(-50, 43, 1000, {true, 127, 10, 1});
   robot.waitUntilDone();
   upScore.set_value(0);
   middle_descore.set_value(0);
   robot.turnToHeading(270, 400, {lemlib::AngularDirection::AUTO, 127, 127, 1});
   robot.waitUntilDone();
   distance_reset_FR();
   
   robot.moveToPoint(-63, 47, 1000, {true, 127, 10, 1});
   robot.waitUntilDone();
   distance_reset_FR();

   robot.moveToPoint(-25, 50, 600, {false, 127, 80, 5});
   robot.waitUntilDone();
   robot.tank(-100, -100);
   loader.set_value(0);
   distance_reset_FR();
   scoreUp();
   pros::delay(1400);
   descore.set_value(0);
   upScore.set_value(0);
   stopIntake();

   // robot.turnToHeading(230, 300, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   // robot.moveToPoint(-36, 40, 500, {true, 127, 127, 1});
   // robot.turnToHeading(270, 300, {lemlib::AngularDirection::CW_CLOCKWISE, 127, 127, 1});
   // robot.waitUntilDone();
   // descore.set_value(0);

   // robot.moveToPoint(-17, 41, 2000, {false, 127, 127, 0});
   // robot.turnToHeading(290, 100000, {lemlib::AngularDirection::AUTO, 10, 5, 0});
}
void solo_awp_13_right_1longgoal_9() {

   robot.setPose(-47, -11, 120);
   loadBasket();
   // scoreMiddle_slower();

   robot.moveToPoint(-23, -24, 900, {true, 127, 60, 13}, false);
   descore.set_value(1);
   loader.set_value(1);

   robot.swingToHeading(220, lemlib::DriveSide::RIGHT, 500, {lemlib::AngularDirection::AUTO, 127, 100, 20});
   
   robot.moveToPose(-50, -47, 270, 1200, {true, 2, 0.2, 127, 90, 10});
   robot.waitUntilDone();
   distance_reset_FL();

   robot.moveToPoint(-63, -48, 750, {true, 120, 10, 1});
   robot.waitUntilDone();
   distance_reset_FL();
      
   robot.moveToPoint(-25, -48, 700, {false, 120, 80, 1});
   robot.waitUntilDone();
   robot.tank(-127, -127);
   loader.set_value(0);
   distance_reset_FL();
   scoreUp();
   pros::delay(1100);
   robot.tank(0, 0);

   robot.turnToHeading(360, 400, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.waitUntilDone();
   upScore.set_value(0);
   robot.moveToPoint(-28, 14, 1400, {true, 120, 120, 5});
   robot.waitUntilDone();
   loader.set_value(1);
   // robot.turnToHeading(315, 350, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   middle_descore.set_value(1);
   robot.swingToHeading(316, lemlib::DriveSide::LEFT, 400, {lemlib::AngularDirection::AUTO, 127, 80, 5});
   robot.moveToPoint(-13, 7, 700, {false, 120, 90, 1});
   robot.waitUntilDone();
   
   scoreMiddle_slower();
   pros::delay(1500);
   loadBasket();
   
   robot.moveToPoint(-50, 43, 1000, {true, 127, 10, 1});
   robot.waitUntilDone();
   upScore.set_value(0);
   middle_descore.set_value(0);
   robot.turnToHeading(270, 400, {lemlib::AngularDirection::AUTO, 127, 127, 1});
   robot.waitUntilDone();
   distance_reset_FR();
   
   robot.moveToPoint(-63, 47, 900, {true, 127, 10, 1});
   robot.waitUntilDone();
   distance_reset_FR();

   robot.moveToPoint(-25, 50, 600, {false, 127, 80, 5});
   robot.waitUntilDone();
   robot.tank(-100, -100);
   loader.set_value(0);
   distance_reset_FR();
   scoreUp();
   pros::delay(1100);

   robot.turnToHeading(230, 300, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.moveToPoint(-36, 39, 600, {true, 127, 127, 1});
   robot.turnToHeading(270, 300, {lemlib::AngularDirection::CW_CLOCKWISE, 127, 127, 1});
   robot.waitUntilDone();
   descore.set_value(0);

   robot.moveToPoint(-17, 41, 2000, {false, 127, 127, 0});
   robot.turnToHeading(290, 100000, {lemlib::AngularDirection::AUTO, 10, 5, 0});
}
void rushleft6loader() {
   robot.setPose(-50, 16, 67);
   loadBasket();
   // scoreMiddle_slower();

   robot.moveToPoint(-26, 23, 1000, {true, 127, 80, 10});
   pros::delay(450);



   // 
   // robot.moveToPose(-23, -26, 140, 900, {true, 3, 0.2, 127, 120, 10}, false);

   descore.set_value(1);
   loader.set_value(1);

   robot.swingToHeading(310, lemlib::DriveSide::LEFT, 400, {lemlib::AngularDirection::AUTO, 127, 100, 20});
   robot.moveToPose(-50, 47, 270, 1100, {true, 2, 0.2, 127, 90, 10});
   robot.waitUntilDone();
   distance_reset_FR();
   robot.moveToPoint(-69, 47.5, 900, {true, 70, 10, 1});
   robot.waitUntilDone();
   distance_reset_FR();
   // long goal + ziga
   robot.moveToPoint(-25, 49.5, 750, {false, 120, 30, 1});
   robot.waitUntilDone();
   robot.tank(-127, -127);
   loader.set_value(0);
   distance_reset_FR();
   scoreUp();
   pros::delay(1100);
   robot.tank(0, 0);
   robot.turnToHeading(220, 300, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.waitUntilDone();
   loader.set_value(0);

   descore.set_value(0);
   robot.moveToPoint(-32, 38, 350, {true, 127, 127, 2});
   // robot.turnToHeading(270, 300, {lemlib::AngularDirection::CW_CLOCKWISE, 127, 127, 1});
   // robot.moveToPoint(-17, 40, 2000, {false, 127, 127, 0});
   robot.moveToPose(-14, 40, 270, 1600, {false, 2, 0.1, 100, 20, 0});
      // robot.swingToHeading(280, lemlib::DriveSide::RIGHT, 700, {lemlib::AngularDirection::AUTO, 40, 10, 0});
   robot.turnToHeading(300, 100000, {lemlib::AngularDirection::AUTO, 10, 5, 0});
}
void rushleft9() {
   robot.setPose(-47, 11, 58);
   loadBasket();
   robot.moveToPoint(-23, 24, 900, {true, 127, 127, 13}, false);
   descore.set_value(1);

   robot.moveToPose(-6, 50, 45, 800, {true, 2, 0.25, 127, 20, 1});
   pros::delay(750);
   loader.set_value(1);
   // pros::delay(350);
   // loader.set_value(0); 

   robot.moveToPoint(-20, 30, 700, {false, 127, 80, 1});
   robot.moveToPose(-50, 45, 270, 1400, {true, 2, 0.2, 127, 20, 1});
   robot.waitUntilDone();
   distance_reset_FR();

   robot.moveToPoint(-63, 48, 800, {true, 70, 10, 1});
   robot.waitUntilDone();
   distance_reset_FR();



   // long goal + ziga
   robot.moveToPoint(-25, 48, 700, {false, 120, 30, 1});
   robot.waitUntilDone();
   robot.tank(-127, -127);
   loader.set_value(0);
   distance_reset_FR();
   scoreUp();
   pros::delay(1100);
   robot.tank(0, 0);

   robot.turnToHeading(220, 300, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.waitUntilDone();
   descore.set_value(0);

   robot.moveToPoint(-32, 38, 350, {true, 127, 127, 2});
   // robot.turnToHeading(270, 300, {lemlib::AngularDirection::CW_CLOCKWISE, 127, 127, 1});
   // robot.moveToPoint(-17, 40, 2000, {false, 127, 127, 0});
   robot.moveToPose(-14, 39, 270, 1200, {false, 2, 0.2, 127, 110, 0});
   robot.turnToHeading(290, 100000, {lemlib::AngularDirection::AUTO, 10, 5, 0});
}
void playoff_push_proga_left() {
   robot.setPose(-50, 16, 67);
   loadBasket();
   robot.moveToPoint(-24, 25, 1800, {true, 120, 10, 1});
   descore.set_value(1);
   pros::delay(450);
   loader.set_value(1);
   pros::delay(300);
   loader.set_value(0);
   robot.moveToPose(-5, 48, 45, 1100, {true, 2, 0.2, 100, 10, 0});

   pros::delay(700);
   loader.set_value(1);
   middle_descore.set_value(1); 

   pros::delay(500);

   // robot.turnToHeading(40, 500, {lemlib::AngularDirection::AUTO, 127, 50, 1});

   // robot.moveToPoint(-9, 8,1100, {false, 120, 10, 0});
   // robot.turnToHeading(315, 500, {lemlib::AngularDirection::AUTO, 127, 50, 1});
      
   // middle score   
   robot.swingToHeading(40, lemlib::DriveSide::RIGHT, 400, {lemlib::AngularDirection::AUTO, 127, 50, 1});
   robot.moveToPose(-9, 12, 315, 2000, {false, 2, 0.2, 127, 50, 1});
   robot.waitUntil(4);
   intakeStage3.move_voltage(-6000);
   intakeStage2.move_voltage(-4000);
   intakeStage1.move_voltage(0);

   robot.waitUntilDone();

   upScore.set_value(1);
   scoreMiddle_slower();

   pros::delay(1100);
   // loader.set_value(0);
   stopIntake();

   // robot.moveToPose(-50, 43, 270, 1200, {true, 2, 0.2, 127, 50, 5});
   robot.moveToPoint(-46, 47, 1300, {true, 127, 50, 1});
   robot.swingToHeading(270, lemlib::DriveSide::LEFT, 200, {lemlib::AngularDirection::AUTO, 127, 100, 1});


   // robot.turnToHeading(270, 300, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.waitUntilDone();
   upScore.set_value(0);
   middle_descore.set_value(0);
   loadBasket();
   robot.moveToPoint(-72, 49, 700, {true, 70, 10, 1});
   robot.waitUntilDone();
   distance_reset_FR();



   robot.moveToPose(-26, 38, 270, 1000, {false, 2, 0.2, 127, 80, 5});
   robot.swingToHeading(270, lemlib::DriveSide::RIGHT, 200, {lemlib::AngularDirection::AUTO, 127, 100, 1});
   // robot.moveToPose(-19, 41, 270, 1200, {false, 2, 0.2, 127, 5, 0});

   robot.moveToPoint(-11, 41, 700, {false, 127, 0, 0});

   robot.waitUntilDone();
   // pros::delay(500);

   descore.set_value(0);

   robot.moveToPoint(-36, 41, 900, {true, 127, 100, 1});


   robot.swingToHeading(210, lemlib::DriveSide::LEFT, 400, {lemlib::AngularDirection::AUTO, 127, 127, 10});







      // long goal + ziga
   robot.moveToPoint(-24, 49.5, 800, {false, 127, 127, 1});
   robot.waitUntilDone();
   robot.tank(-127, -127);
   pros::delay(200);

   loader.set_value(0);
   distance_reset_FR();
   scoreUp();
   pros::delay(1100);
   robot.tank(0, 0);

   robot.turnToHeading(220, 300, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.waitUntilDone();
   descore.set_value(0);

   robot.moveToPoint(-32, 38, 300, {true, 127, 127, 2});
   // robot.turnToHeading(270, 300, {lemlib::AngularDirection::CW_CLOCKWISE, 127, 127, 1});
   // robot.moveToPoint(-17, 40, 2000, {false, 127, 127, 0});
   robot.moveToPose(-14, 40, 270, 1800, {false, 2, 0.1, 90, 20, 0});
   robot.turnToHeading(290, 1000, {lemlib::AngularDirection::AUTO, 10, 5, 0});
}

void playoff_proga_left() {
   robot.setPose(-50, 16, 67);
   loadBasket();
   robot.moveToPoint(-24, 25, 1800, {true, 120, 10, 1});
   descore.set_value(1);
   pros::delay(450);
   loader.set_value(1);
   pros::delay(300);
   loader.set_value(0);
   robot.moveToPose(-5, 48, 45, 1100, {true, 2, 0.2, 100, 10, 0});

   pros::delay(700);
   loader.set_value(1);
   middle_descore.set_value(1); 

   pros::delay(500);

   // robot.turnToHeading(40, 500, {lemlib::AngularDirection::AUTO, 127, 50, 1});

   // robot.moveToPoint(-9, 8,1100, {false, 120, 10, 0});
   // robot.turnToHeading(315, 500, {lemlib::AngularDirection::AUTO, 127, 50, 1});
      
   // middle score   
   robot.swingToHeading(40, lemlib::DriveSide::RIGHT, 400, {lemlib::AngularDirection::AUTO, 127, 50, 1});
   robot.moveToPose(-9, 12, 315, 2000, {false, 2, 0.2, 127, 50, 1});
   robot.waitUntil(4);
   intakeStage3.move_voltage(-6000);
   intakeStage2.move_voltage(-4000);
   intakeStage1.move_voltage(0);

   robot.waitUntilDone();
   pros::delay(100);
   upScore.set_value(1);
   scoreMiddle_slower();

   pros::delay(1100);
   // loader.set_value(0);
   stopIntake();

   // robot.moveToPose(-50, 43, 270, 1200, {true, 2, 0.2, 127, 50, 5});
   robot.moveToPoint(-46, 47, 1300, {true, 127, 50, 1});
   robot.swingToHeading(270, lemlib::DriveSide::LEFT, 200, {lemlib::AngularDirection::AUTO, 127, 100, 1});


   // robot.turnToHeading(270, 300, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.waitUntilDone();
   upScore.set_value(0);
   middle_descore.set_value(0);
   loadBasket();
   robot.moveToPoint(-72, 49, 700, {true, 70, 10, 1});
   robot.waitUntilDone();
   distance_reset_FR();



   robot.moveToPose(-26, 38, 270, 1000, {false, 2, 0.2, 127, 80, 5});
   robot.swingToHeading(270, lemlib::DriveSide::RIGHT, 200, {lemlib::AngularDirection::AUTO, 127, 100, 1});
   // robot.moveToPose(-19, 41, 270, 1200, {false, 2, 0.2, 127, 5, 0});

   robot.moveToPoint(-11, 41, 700, {false, 127, 0, 0});

   robot.waitUntilDone();
   // pros::delay(500);

   descore.set_value(0);

   robot.moveToPoint(-36, 41, 900, {true, 127, 100, 1});


   robot.swingToHeading(210, lemlib::DriveSide::LEFT, 400, {lemlib::AngularDirection::AUTO, 127, 10, 1});







      // long goal + ziga
   robot.moveToPoint(-24, 49.5, 800, {false, 127, 127, 1});
   robot.waitUntilDone();
   robot.tank(-127, -127);
   pros::delay(200);

   loader.set_value(0);
   distance_reset_FR();
   scoreUp();
   pros::delay(1100);
   robot.tank(0, 0);

   robot.turnToHeading(220, 400, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.waitUntilDone();
   descore.set_value(0);

   robot.moveToPoint(-32, 38, 300, {true, 127, 10, 2});
   // robot.turnToHeading(270, 300, {lemlib::AngularDirection::CW_CLOCKWISE, 127, 127, 1});
   // robot.moveToPoint(-17, 40, 2000, {false, 127, 127, 0});
   robot.moveToPose(-26, 40, 270, 1800, {false, 2, 0.1, 90, 20, 0});
   // robot.turnToHeading(270, 1000, {lemlib::AngularDirection::AUTO, 120, 5, 0});
}

void playoff_push_proga_right() {
   robot.setPose(-50, -16, 109);
   loadBasket();
   robot.moveToPoint(-25, -23, 1800, {true, 127, 30, 1});
   descore.set_value(1);
   pros::delay(450);
   loader.set_value(1);
   pros::delay(200);
   loader.set_value(0);
   robot.moveToPose(-5, -47, 135, 1100, {true, 2, 0.2, 127, 10, 0});

   pros::delay(700);
   loader.set_value(1);
   middle_descore.set_value(1); 

   pros::delay(400);

   // robot.turnToHeading(40, 500, {lemlib::AngularDirection::AUTO, 127, 50, 1});

   // robot.moveToPoint(-9, 8,1100, {false, 120, 10, 0});
   // robot.turnToHeading(315, 500, {lemlib::AngularDirection::AUTO, 127, 50, 1});
      
   // middle score   
   // robot.swingToHeading(150, lemlib::DriveSide::LEFT, 400, {lemlib::AngularDirection::AUTO, 127, 50, 1});
   robot.moveToPoint(-20, -25,700, {false, 127, 60, 5});
   robot.waitUntilDone();
   loader.set_value(0);
   pros::delay(100);

   robot.turnToHeading(40, 400, {lemlib::AngularDirection::AUTO, 127, 50, 1});

   robot.moveToPose(-12, -8, 42, 700, {true, 2, 0.1, 127, 50, 1});


   robot.waitUntilDone();
   loader.set_value(1);
   pros::delay(100);

   intake_upper.set_value(1);
   scoreMiddleDown();
   pros::delay(1200);
   stopIntake();
   intake_upper.set_value(0);
   pros::delay(50);
   loader.set_value(0);

   // robot.moveToPose(-50, 43, 270, 1200, {true, 2, 0.2, 127, 50, 5});
   robot.moveToPoint(-42, -41, 1000, {false, 127, 80, 3});
   robot.waitUntil(10);
      loader.set_value(1);

   robot.turnToHeading(270, 500, {lemlib::AngularDirection::AUTO, 127, 10, 0});


   // robot.turnToHeading(270, 300, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.waitUntilDone();
   upScore.set_value(0);
   middle_descore.set_value(0);
   loadBasket();
   robot.moveToPoint(-66, -45, 1000, {true, 80, 10, 1});
   robot.waitUntilDone();
   distance_reset_FR();



   robot.moveToPose(-26, -59, 270, 1000, {false, 2, 0.2, 127, 80, 5});
   robot.swingToHeading(270, lemlib::DriveSide::RIGHT, 200, {lemlib::AngularDirection::AUTO, 127, 100, 1});
   // robot.moveToPose(-19, -58, 270, 1200, {false, 2, 0.2, 127, 5, 0});

   robot.moveToPoint(-10, -58, 700, {false, 127, 0, 0});

   robot.waitUntilDone();
   // pros::delay(500);
   loader.set_value(0);
   descore.set_value(0);

   robot.moveToPoint(-40, -58, 900, {true, 127, 100, 1});

   robot.turnToHeading(215, 400, {lemlib::AngularDirection::AUTO, 127, 80, 1});

   // robot.swingToHeading(210, lemlib::DriveSide::LEFT, 300, {lemlib::AngularDirection::AUTO, 127, 127, 10});







      // long goal + ziga
   robot.moveToPoint(-24, -48.5, 800, {false, 127, 127, 1});
   robot.waitUntilDone();
   robot.tank(-127, -127);
   pros::delay(200);

   loader.set_value(0);
   distance_reset_FR();
   scoreUp();
   pros::delay(800);
   robot.tank(0, 0);

   robot.turnToHeading(220, 300, {lemlib::AngularDirection::AUTO, 127, 127, 10});
   robot.waitUntilDone();
   descore.set_value(0);

   robot.moveToPoint(-32, -58, 300, {true, 127, 127, 2});
   // robot.turnToHeading(270, 300, {lemlib::AngularDirection::CW_CLOCKWISE, 127, 127, 1});
   // robot.moveToPoint(-17, 40, 2000, {false, 127, 127, 0});
   robot.moveToPose(-11, -58, 270, 1800, {false, 2, 0.1, 90, 20, 0});
   robot.turnToHeading(290, 1000, {lemlib::AngularDirection::AUTO, 10, 5, 0});
}