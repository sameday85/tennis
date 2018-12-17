/*
 * Copyright (C) 2018 The Automatic Tennis Ball Collector Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/
#include <wiringPi.h>
#include "led.h"
#include "pins.h"
#include "utils.h"

bool g_red_led_on = false;

void Led::turn_on_led(int pin) {
    digitalWrite(pin, HIGH);
}

void Led::turn_off_led(int pin) {
    digitalWrite(pin, LOW);
}

void Led::turn_on_red_led(){
    if (g_red_led_on)
        return;
    digitalWrite(PIN_LED_RED, HIGH);
    g_red_led_on=true;
}

void Led::turn_off_red_led() {
    if (!g_red_led_on)
        return;
    digitalWrite(PIN_LED_RED, LOW);
    g_red_led_on=false;
}

void Led::set_led_state(int state) {
    if (state & LED_STATE_RED_ON)
        turn_on_red_led();
    else
        turn_off_red_led();
    if (state & LED_STATE_GREEN_ON)
        turn_on_led(PIN_LED_GREEN);
    else
        turn_off_led(PIN_LED_GREEN);
    if (state & LED_STATE_BLUE_ON)
        turn_on_led(PIN_LED_BLUE);
    else
        turn_off_led(PIN_LED_BLUE);
}

//make a long or shot buzzle
void Led::buzzle (bool long_time) {
    int duration = long_time ? 2000 : 300;
    for(int i=0;i<(duration/2);i++) { 
        digitalWrite(PIN_BUZZER,HIGH);// sound 
        Utils::delay_ms(1);//delay1ms 
        digitalWrite(PIN_BUZZER,LOW);//not sound 
        Utils::delay_ms(1);//ms delay
    }
    digitalWrite(PIN_BUZZER, LOW);
}
