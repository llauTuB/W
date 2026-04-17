#include "config/lemlib_config.hpp"
#include "config/robot_config.hpp"
#include "lemlib/pose.hpp"
#include <math.h>

const double OFF_LEFT = 5.25; 
const double OFF_RIGHT = 5.25; 
const double OFF_BACK = 4; 
const double OFF_FRONT = 5; 

const double FIELD_HALF_WIDTH = 72.0; 
const double MAX_RESET_DIST = 45.0; 
//35.0
const double MAX_RESET_DIST_FOR_SKILLS_BEGINNING = 68.0; 

const double MAX_TILT = 30.0;        

void distance_reset() {
    lemlib::Pose p = robot.getPose();
    double theta = p.theta;
    while (theta < 0) theta += 360;
    while (theta >= 360) theta -= 360;

    double dL = distance_left.get_distance() / 25.4;
    double dR = distance_right.get_distance() / 25.4;
    double dB = distance_back.get_distance() / 25.4;
    double dF = distance_front.get_distance() / 25.4;

    int facing = std::round(theta / 90.0);
    if (facing == 4) facing = 0;

    // --- DEBUG PRINT ---
    printf("RESET DEBUG: Facing: %d | Old X: %.2f Old Y: %.2f\n", facing, p.x, p.y);
    printf("SENSORS: Left: %.2f | Right: %.2f | Back: %.2f | Front: %.2f\n", dL, dR, dB, dF);

    double targetAngle = facing * 90.0;
    double tilt = theta - targetAngle;

    while (tilt > 180) tilt -= 360;
    while (tilt <= -180) tilt += 360;

    if (std::abs(tilt) > MAX_TILT) return;

    double correction = std::cos(tilt * M_PI / 180.0);

    double newX = p.x;
    double newY = p.y;

    if (facing == 0) {
        if (p.x < 0 && dL < MAX_RESET_DIST) newX = -FIELD_HALF_WIDTH + (dL * correction) + OFF_LEFT;
        else if (p.x > 0 && dR < MAX_RESET_DIST) newX = FIELD_HALF_WIDTH - (dR * correction) - OFF_RIGHT;
        
        if (p.y < 0 && dB < MAX_RESET_DIST) newY = -FIELD_HALF_WIDTH + (dB * correction) + OFF_BACK;
        else if (p.y > 0 && dF < MAX_RESET_DIST) newY = FIELD_HALF_WIDTH - (dF * correction) - OFF_FRONT;
    }
    else if (facing == 1) {
        if (p.x < 0 && dB < MAX_RESET_DIST) newX = -FIELD_HALF_WIDTH + (dB * correction) + OFF_BACK;
        else if (p.x > 0 && dF < MAX_RESET_DIST) newX = FIELD_HALF_WIDTH - (dF * correction) - OFF_FRONT;

        if (p.y > 0 && dL < MAX_RESET_DIST) newY = FIELD_HALF_WIDTH - (dL * correction) - OFF_LEFT;
        else if (p.y < 0 && dR < MAX_RESET_DIST) newY = -FIELD_HALF_WIDTH + (dR * correction) + OFF_RIGHT;
    }
    else if (facing == 2) {
        if (p.x > 0 && dL < MAX_RESET_DIST) newX = FIELD_HALF_WIDTH - (dL * correction) - OFF_LEFT;
        else if (p.x < 0 && dR < MAX_RESET_DIST) newX = -FIELD_HALF_WIDTH + (dR * correction) + OFF_RIGHT;
        
        if (p.y > 0 && dB < MAX_RESET_DIST) newY = FIELD_HALF_WIDTH - (dB * correction) - OFF_BACK;
        else if (p.y < 0 && dF < MAX_RESET_DIST) newY = -FIELD_HALF_WIDTH + (dF * correction) + OFF_FRONT;
    }
    else if (facing == 3) {
        if (p.x > 0 && dB < MAX_RESET_DIST) newX = FIELD_HALF_WIDTH - (dB * correction) - OFF_BACK;
        else if (p.x < 0 && dF < MAX_RESET_DIST) newX = -FIELD_HALF_WIDTH + (dF * correction) + OFF_FRONT;

        if (p.y < 0 && dL < MAX_RESET_DIST) newY = -FIELD_HALF_WIDTH + (dL * correction) + OFF_LEFT;
        else if (p.y > 0 && dR < MAX_RESET_DIST) newY = FIELD_HALF_WIDTH - (dR * correction) - OFF_RIGHT;
    }

    robot.setPose(newX, newY, p.theta);
    
    printf("RESET COMPLETE: New X: %.2f | New Y: %.2f\n", newX, newY);
}


void distance_reset_FL() {
    lemlib::Pose p = robot.getPose();
    double theta = p.theta;
    while (theta < 0) theta += 360;
    while (theta >= 360) theta -= 360;

    double dL = distance_left.get_distance() / 25.4;
    double dF = distance_front.get_distance() / 25.4;

    int facing = std::round(theta / 90.0);
    if (facing == 4) facing = 0;

    double targetAngle = facing * 90.0;
    double tilt = theta - targetAngle;
    while (tilt > 180) tilt -= 360;
    while (tilt <= -180) tilt += 360;
    if (std::abs(tilt) > MAX_TILT) return;
    double correction = std::cos(tilt * M_PI / 180.0);

    double newX = p.x;
    double newY = p.y;

    if (facing == 0) {
        if (p.x < 0 && dL < MAX_RESET_DIST) newX = -FIELD_HALF_WIDTH + (dL * correction) + OFF_LEFT;
        if (p.y > 0 && dF < MAX_RESET_DIST) newY = FIELD_HALF_WIDTH - (dF * correction) - OFF_FRONT;
    }
    else if (facing == 1) {
        if (p.x > 0 && dF < MAX_RESET_DIST) newX = FIELD_HALF_WIDTH - (dF * correction) - OFF_FRONT;
        if (p.y > 0 && dL < MAX_RESET_DIST) newY = FIELD_HALF_WIDTH - (dL * correction) - OFF_LEFT;
    }
    else if (facing == 2) {
        if (p.x > 0 && dL < MAX_RESET_DIST) newX = FIELD_HALF_WIDTH - (dL * correction) - OFF_LEFT;
        if (p.y < 0 && dF < MAX_RESET_DIST) newY = -FIELD_HALF_WIDTH + (dF * correction) + OFF_FRONT;
    }
    else if (facing == 3) {
        if (p.x < 0 && dF < MAX_RESET_DIST) newX = -FIELD_HALF_WIDTH + (dF * correction) + OFF_FRONT;
        if (p.y < 0 && dL < MAX_RESET_DIST) newY = -FIELD_HALF_WIDTH + (dL * correction) + OFF_LEFT;
    }

    robot.setPose(newX, newY, p.theta);
}



void distance_reset_FR() {
    lemlib::Pose p = robot.getPose();
    double theta = p.theta;
    while (theta < 0) theta += 360;
    while (theta >= 360) theta -= 360;

    double dR = distance_right.get_distance() / 25.4;
    double dF = distance_front.get_distance() / 25.4;

    int facing = std::round(theta / 90.0);
    if (facing == 4) facing = 0;

    double targetAngle = facing * 90.0;
    double tilt = theta - targetAngle;
    while (tilt > 180) tilt -= 360;
    while (tilt <= -180) tilt += 360;
    if (std::abs(tilt) > MAX_TILT) return;
    double correction = std::cos(tilt * M_PI / 180.0);

    double newX = p.x;
    double newY = p.y;

    if (facing == 0) {
        if (p.x > 0 && dR < MAX_RESET_DIST) newX = FIELD_HALF_WIDTH - (dR * correction) - OFF_RIGHT;
        if (p.y > 0 && dF < MAX_RESET_DIST) newY = FIELD_HALF_WIDTH - (dF * correction) - OFF_FRONT;
    }
    else if (facing == 1) {
        if (p.x > 0 && dF < MAX_RESET_DIST) newX = FIELD_HALF_WIDTH - (dF * correction) - OFF_FRONT;
        if (p.y < 0 && dR < MAX_RESET_DIST) newY = -FIELD_HALF_WIDTH + (dR * correction) + OFF_RIGHT;
    }
    else if (facing == 2) {
        if (p.x < 0 && dR < MAX_RESET_DIST) newX = -FIELD_HALF_WIDTH + (dR * correction) + OFF_RIGHT;
        if (p.y < 0 && dF < MAX_RESET_DIST) newY = -FIELD_HALF_WIDTH + (dF * correction) + OFF_FRONT;
    }
    else if (facing == 3) {
        if (p.x < 0 && dF < MAX_RESET_DIST) newX = -FIELD_HALF_WIDTH + (dF * correction) + OFF_FRONT;
        if (p.y > 0 && dR < MAX_RESET_DIST) newY = FIELD_HALF_WIDTH - (dR * correction) - OFF_RIGHT;
    }

    robot.setPose(newX, newY, p.theta);
}



void distance_reset_BL() {
    lemlib::Pose p = robot.getPose();
    double theta = p.theta;
    while (theta < 0) theta += 360;
    while (theta >= 360) theta -= 360;

    double dL = distance_left.get_distance() / 25.4;
    double dB = distance_back.get_distance() / 25.4;

    int facing = std::round(theta / 90.0);
    if (facing == 4) facing = 0;

    double targetAngle = facing * 90.0;
    double tilt = theta - targetAngle;
    while (tilt > 180) tilt -= 360;
    while (tilt <= -180) tilt += 360;
    if (std::abs(tilt) > MAX_TILT) return;
    double correction = std::cos(tilt * M_PI / 180.0);

    double newX = p.x;
    double newY = p.y;

    if (facing == 0) {
        if (p.x < 0 && dL < MAX_RESET_DIST) newX = -FIELD_HALF_WIDTH + (dL * correction) + OFF_LEFT;
        if (p.y < 0 && dB < MAX_RESET_DIST) newY = -FIELD_HALF_WIDTH + (dB * correction) + OFF_BACK;
    }
    else if (facing == 1) {
        if (p.x < 0 && dB < MAX_RESET_DIST) newX = -FIELD_HALF_WIDTH + (dB * correction) + OFF_BACK;
        if (p.y > 0 && dL < MAX_RESET_DIST) newY = FIELD_HALF_WIDTH - (dL * correction) - OFF_LEFT;
    }
    else if (facing == 2) {
        if (p.x > 0 && dL < MAX_RESET_DIST) newX = FIELD_HALF_WIDTH - (dL * correction) - OFF_LEFT;
        if (p.y > 0 && dB < MAX_RESET_DIST) newY = FIELD_HALF_WIDTH - (dB * correction) - OFF_BACK;
    }
    else if (facing == 3) {
        if (p.x > 0 && dB < MAX_RESET_DIST) newX = FIELD_HALF_WIDTH - (dB * correction) - OFF_BACK;
        if (p.y < 0 && dL < MAX_RESET_DIST) newY = -FIELD_HALF_WIDTH + (dL * correction) + OFF_LEFT;
    }

    robot.setPose(newX, newY, p.theta);
}

void distance_reset_BR() {
    lemlib::Pose p = robot.getPose();
    double theta = p.theta;
    while (theta < 0) theta += 360;
    while (theta >= 360) theta -= 360;

    double dR = distance_right.get_distance() / 25.4;
    double dB = distance_back.get_distance() / 25.4;

    int facing = std::round(theta / 90.0);
    if (facing == 4) facing = 0;

    double targetAngle = facing * 90.0;
    double tilt = theta - targetAngle;
    while (tilt > 180) tilt -= 360;
    while (tilt <= -180) tilt += 360;
    if (std::abs(tilt) > MAX_TILT) return;
    double correction = std::cos(tilt * M_PI / 180.0);

    double newX = p.x;
    double newY = p.y;

    if (facing == 0) {
        if (p.x > 0 && dR < MAX_RESET_DIST) newX = FIELD_HALF_WIDTH - (dR * correction) - OFF_RIGHT;
        if (p.y < 0 && dB < MAX_RESET_DIST) newY = -FIELD_HALF_WIDTH + (dB * correction) + OFF_BACK;
    }
    else if (facing == 1) {
        if (p.x < 0 && dB < MAX_RESET_DIST) newX = -FIELD_HALF_WIDTH + (dB * correction) + OFF_BACK;
        if (p.y < 0 && dR < MAX_RESET_DIST) newY = -FIELD_HALF_WIDTH + (dR * correction) + OFF_RIGHT;
    }
    else if (facing == 2) {
        if (p.x < 0 && dR < MAX_RESET_DIST) newX = -FIELD_HALF_WIDTH + (dR * correction) + OFF_RIGHT;
        if (p.y > 0 && dB < MAX_RESET_DIST) newY = FIELD_HALF_WIDTH - (dB * correction) - OFF_BACK;
    }
    else if (facing == 3) {
        if (p.x > 0 && dB < MAX_RESET_DIST) newX = FIELD_HALF_WIDTH - (dB * correction) - OFF_BACK;
        if (p.y > 0 && dR < MAX_RESET_DIST) newY = FIELD_HALF_WIDTH - (dR * correction) - OFF_RIGHT;
    }

    robot.setPose(newX, newY, p.theta);
}

void distance_reset_for_skills_beginning() {
    lemlib::Pose p = robot.getPose();
    double theta = p.theta;
    while (theta < 0) theta += 360;
    while (theta >= 360) theta -= 360;

    double dL = distance_left.get_distance() / 25.4;
    double dR = distance_right.get_distance() / 25.4;
    double dB = distance_back.get_distance() / 25.4;
    double dF = distance_front.get_distance() / 25.4;

    int facing = std::round(theta / 90.0);
    if (facing == 4) facing = 0;

    // --- DEBUG PRINT ---
    printf("RESET DEBUG: Facing: %d | Old X: %.2f Old Y: %.2f\n", facing, p.x, p.y);
    printf("SENSORS: Left: %.2f | Right: %.2f | Back: %.2f | Front: %.2f\n", dL, dR, dB, dF);

    double targetAngle = facing * 90.0;
    double tilt = theta - targetAngle;

    while (tilt > 180) tilt -= 360;
    while (tilt <= -180) tilt += 360;

    if (std::abs(tilt) > MAX_TILT) return;

    double correction = std::cos(tilt * M_PI / 180.0);

    double newX = p.x;
    double newY = p.y;

    if (facing == 0) {
        if (p.x < 0 && dL < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newX = -FIELD_HALF_WIDTH + (dL * correction) + OFF_LEFT;
        else if (p.x > 0 && dR < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newX = FIELD_HALF_WIDTH - (dR * correction) - OFF_RIGHT;
        
        if (p.y < 0 && dB < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newY = -FIELD_HALF_WIDTH + (dB * correction) + OFF_BACK;
        else if (p.y > 0 && dF < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newY = FIELD_HALF_WIDTH - (dF * correction) - OFF_FRONT;
    }
    else if (facing == 1) {
        if (p.x < 0 && dB < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newX = -FIELD_HALF_WIDTH + (dB * correction) + OFF_BACK;
        else if (p.x > 0 && dF < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newX = FIELD_HALF_WIDTH - (dF * correction) - OFF_FRONT;

        if (p.y > 0 && dL < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newY = FIELD_HALF_WIDTH - (dL * correction) - OFF_LEFT;
        else if (p.y < 0 && dR < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newY = -FIELD_HALF_WIDTH + (dR * correction) + OFF_RIGHT;
    }
    else if (facing == 2) {
        if (p.x > 0 && dL < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newX = FIELD_HALF_WIDTH - (dL * correction) - OFF_LEFT;
        else if (p.x < 0 && dR < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newX = -FIELD_HALF_WIDTH + (dR * correction) + OFF_RIGHT;
        
        if (p.y > 0 && dB < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newY = FIELD_HALF_WIDTH - (dB * correction) - OFF_BACK;
        else if (p.y < 0 && dF < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newY = -FIELD_HALF_WIDTH + (dF * correction) + OFF_FRONT;
    }
    else if (facing == 3) {
        if (p.x > 0 && dB < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newX = FIELD_HALF_WIDTH - (dB * correction) - OFF_BACK;
        else if (p.x < 0 && dF < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newX = -FIELD_HALF_WIDTH + (dF * correction) + OFF_FRONT;

        if (p.y < 0 && dL < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newY = -FIELD_HALF_WIDTH + (dL * correction) + OFF_LEFT;
        else if (p.y > 0 && dR < MAX_RESET_DIST_FOR_SKILLS_BEGINNING) newY = FIELD_HALF_WIDTH - (dR * correction) - OFF_RIGHT;
    }

    robot.setPose(newX, newY, p.theta);
    
    printf("RESET COMPLETE: New X: %.2f | New Y: %.2f\n", newX, newY);
}