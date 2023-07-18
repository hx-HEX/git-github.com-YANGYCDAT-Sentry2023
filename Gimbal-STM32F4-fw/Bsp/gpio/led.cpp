#include "led.h"



void Led::SetGreen(bool state) {
    if (state) {
        LED_GREEN_ON();
    } else {
        LED_GREEN_OFF();
    }
}


void Led::ToggleGreen(void) {
    LED_GREEN_TOGGLE();
}

void Led::SetBlue(bool state) {
    if (state) {
        LED_BLUE_ON();
    } else {
        LED_BLUE_OFF();
    }
}


void Led::ToggleBlue(void) {
    LED_BLUE_TOGGLE();
}