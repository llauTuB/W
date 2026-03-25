#include "config/robot_config.hpp"
#include "globals/robot_const.hpp"

bool descoreState = false;
bool loaderState = false;
bool doubleParkState = false;
bool upScoreState = false;
bool upIntakeState = false;


void loader_toggle() {
    if(master.get_digital_new_press(LOADER_TOGGLE)) {
        loaderState = !loaderState;
        loader.set_value(loaderState);
    } 
}

void descore_toggle() {
    if(master.get_digital_new_press(DESCORE_TOGGLE)) {
        descoreState = !descoreState;
        descore.set_value(descoreState);
    } 
}



void upScore_toggle() {
  if (master.get_digital_new_press(INTAKE_UPSCORE_TOGGLE)) {
    upScoreState = !upScoreState;                 // toggle true/false
    upScore.set_value(upScoreState ? 127 : 0);    // if true -> 127, else -> 0
  }
}



void doublePark_toggle() {
    if(master.get_digital_new_press(DOUBLE_PARK_TOGGLE)) {
        doubleParkState = !doubleParkState;
        doublePark.set_value(doubleParkState); 
       
    } 
}