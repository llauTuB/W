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

void upScore_toggle() {
  if (master.get_digital_new_press(UPSCORE_DESCORE_TOGGLE)) {
    upScoreState = !upScoreState;                 // toggle true/false
    upScore.set_value(upScoreState ? 127 : 0);    // if true -> 127, else -> 0
  }
}
