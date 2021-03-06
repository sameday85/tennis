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
#include <iostream>

#include <wiringPi.h>

#include "observer.h"
#include "led.h"
#include "utils.h"
#include "pins.h"

using namespace std;

//constructor
Observer::Observer(Config *config, Motor *motor, Location *location, int *flag_interruption, int *user_action) : Component(config) {
    m_motor = motor;
    m_location = location;
    
    p_interruption = flag_interruption;
    p_user_action = user_action;
    
    the_thread = 0;
}

Observer::~Observer() {
}

//initialize GPIO pins
bool Observer::init() {
    Component::init();
    
    //led, button and buzzer GPIOs
    pinMode(PIN_BTN1,  INPUT);
    pinMode(PIN_BTN2,  INPUT);
    pinMode(PIN_LED_RED,    OUTPUT);
    pinMode(PIN_LED_GREEN,  OUTPUT);
    pinMode(PIN_LED_BLUE,   OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    Led::set_led_state(0);

    return true;
}

//Observing obstacles, stop the car if needed.  
//the interruption flag in the context will be set if an obstacle is very close.
//this function will be running in a background thread
//@param arg - pointer to the current class's instance
void* Observer::_monitor(void *arg) {
    Observer *the_observer = (Observer*)arg;
    while (!the_observer->is_done()) {
        long front_obstacle = the_observer->m_location->measure_front_distance();
        long rear_obstacle = the_observer->m_location->measure_rear_distance();
        int car_state = the_observer->m_motor->get_car_state();
        if ((car_state & 1) == 1) {//the is moving forward
            if (front_obstacle <= FRONT_DISTANCE_DANGEROUS) {
                if (the_observer->debug)
                    cout << "!!!!!!!!!!!!!!Danger front, stop now: " << front_obstacle << endl;
                the_observer->m_motor->stop_car();
                *the_observer->p_interruption = INT_FRONT_OBSTACLE;
            }
        }
        else if (car_state != CAR_STATE_STOPPED) { //moving backwards
            if (rear_obstacle <= REAR_DISTANCE_DANGEROUS) {
                if (the_observer->debug)
                    cout << "!!!!!!!!!!!!!!Danger rear, stop now: " << rear_obstacle << endl;
                the_observer->m_motor->stop_car();
                *the_observer->p_interruption = INT_REAR_OBSTACLE;
            }
        }
        //Check if any button is pressed        
        if (digitalRead (PIN_BTN1) == HIGH) {
            Utils::delay_ms(100);
            if (digitalRead (PIN_BTN1) == HIGH) {
                the_observer->btn_event_handle(BTN_01);
            }
        }
        if (digitalRead (PIN_BTN2) == HIGH) {
            Utils::delay_ms(100);
            if (digitalRead (PIN_BTN2) == HIGH) {
                the_observer->btn_event_handle(BTN_02);
            }
        }
        Utils::delay_ms(the_observer->get_config()->get_frame_time_ms() >> 1);
    }
    return arg;//suppress the unused param warning
}

//Start a backgrund thread to monitor obstacle distances and button status
void Observer::start() {
    Component::start();
    pthread_create(&the_thread, NULL, _monitor, this);
}

void Observer::stop() {
    Component::stop();
    if (the_thread)
        pthread_join(the_thread, NULL);
    Led::set_led_state(0);
}

//Process button events
//button one is for changing the value
//button two is for selecting options
//options:
//red on    - pause/resume
//green on  - camera calibration
//blue on   - motor calibration
void* Observer::btn_event_handle(int event) {
    if ((event & 0xff) == BTN_01) { //select options
        switch (*p_user_action) {
            case UA_NONE:
                *p_user_action = UA_NAV2_PAUSE;
                Led::set_led_state(LED_STATE_RED_ON);
                break;
            case UA_PAUSE:
                *p_user_action = UA_NAV2_RESUME;
                Led::set_led_state(0);
                break;
            case UA_PRE_CAMERA_CALIBRATION:
                *p_user_action=UA_CAMERA_CALIBRATION;
                Led::set_led_state(LED_STATE_RED_ON|LED_STATE_GREEN_ON);
                break;
            case UA_PRE_MOTOR_CALIBRATION:
                *p_user_action=UA_MOTOR_CALIBRATION;
                Led::set_led_state(LED_STATE_RED_ON|LED_STATE_BLUE_ON);
                break;
        }
    }
    else if ((event & 0xff) == BTN_02) {//navigate through pause, speed calibration and camera calibrate
        switch (*p_user_action) {
            case UA_PAUSE:
                *p_user_action = UA_PRE_CAMERA_CALIBRATION;
                Led::set_led_state(LED_STATE_GREEN_ON);
                break;
            case UA_PRE_CAMERA_CALIBRATION:
                *p_user_action = UA_PRE_MOTOR_CALIBRATION;
                Led::set_led_state(LED_STATE_BLUE_ON);
                break;
            case UA_PRE_MOTOR_CALIBRATION:
                *p_user_action=UA_PAUSE;
                Led::set_led_state(LED_STATE_RED_ON);
                break;
        }
    }
    return NULL;
}
