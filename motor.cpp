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
#include <softPwm.h>

#include "motor.h"
#include "utils.h"
#include "led.h"
#include "pins.h"


using namespace std;

//the constructor with the configuration
Motor::Motor(Config *config) : Component (config) {
    //do not cache speed_base as it may be changed during running
    active_config = m_config->get_active_config();
    the_thread = 0;
}

Motor::~Motor() {
}

//Initialize GPIO pins and setup the soft PWM
bool Motor::init() {
    pinMode(PIN_LMOTORS_1, OUTPUT);
    pinMode(PIN_LMOTORS_2, OUTPUT);
    pinMode(PIN_RMOTORS_1, OUTPUT);
    pinMode(PIN_RMOTORS_2, OUTPUT);

    pinMode(PIN_COLLECTOR_1, OUTPUT);
    pinMode(PIN_COLLECTOR_2, OUTPUT);
    
    softPwmCreate (PIN_PWM_LEFT,  0, PWM_MAX);
    softPwmCreate (PIN_PWM_RIGHT, 0, PWM_MAX);
    //force to stop all motors
    stop_motor(MOTORS_LEFT);
    stop_motor(MOTORS_RIGHT);
    stop_motor(MOTOR_COLLECTOR);
    m_car_state = CAR_STATE_STOPPED;
    m_collector_state = COLLECTOR_STATE_STOPPED;

    return true;
}

void Motor::stop() {
    Component::stop();
    stop_car();
    stop_collector();
}

//start the given motor. The motor may be one of the robot car motors or the collector (which is a heavy duty motor).
//@param motor - the motor number
//@param dir - the direction: forward or backward
void Motor::start_motor(int motor, int dir) {
    int pin1 = 0, pin2 = 0;
    switch (motor) {
        case MOTORS_LEFT:
            pin1 = PIN_LMOTORS_1;
            pin2 = PIN_LMOTORS_2;
            break;
        case MOTORS_RIGHT:
            pin1 = PIN_RMOTORS_1;
            pin2 = PIN_RMOTORS_2;
            break;
        case MOTOR_COLLECTOR:
            pin1 = PIN_COLLECTOR_1;
            pin2 = PIN_COLLECTOR_2;
            break;
    }
    int value1=LOW, value2=LOW;
    switch (dir) {
        case DIR_FORWARD:
            value1=HIGH;
            break;
        case DIR_BACKWARD:
            value2=HIGH;
            break;
    }
    digitalWrite(pin1, value1);
    digitalWrite(pin2, value2);
}

//stop the given motor
//@param motor - the motor number
void Motor::stop_motor(int motor) {
    int pin1 = 0, pin2 = 0;
    switch (motor) {
        case MOTORS_LEFT:
            pin1 = PIN_LMOTORS_1;
            pin2 = PIN_LMOTORS_2;
            break;
        case MOTORS_RIGHT:
            pin1 = PIN_RMOTORS_1;
            pin2 = PIN_RMOTORS_2;
            break;
        case MOTOR_COLLECTOR:
            pin1 = PIN_COLLECTOR_1;
            pin2 = PIN_COLLECTOR_2;
            break;
    }
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
}

//set the speed for the left and right motors
void Motor::set_speed_left_right(int left_speed, int right_speed) {
    softPwmWrite (PIN_PWM_LEFT,  left_speed);
    softPwmWrite (PIN_PWM_RIGHT, right_speed);
}

//set the motor speed for the given car state
void Motor::set_speed(int desired_state) {
    switch (desired_state) {
        case CAR_STATE_MOVING_FORWARD:
        case CAR_STATE_MOVING_BACKWARD:
            set_speed_left_right(MOVING_SPEED+active_config->speed_base, MOVING_SPEED+active_config->speed_base);
            break;
        case CAR_STATE_TURNING_LEFT_FORWARD:
        case CAR_STATE_TURNING_RIGHT_BACKWARD: //car header direction change
            set_speed_left_right(TURNING_SPEED_MIN+active_config->speed_base, TURNING_SPEED_MAX+active_config->speed_base);
            break;
        case CAR_STATE_TURNING_RIGHT_FORWARD:
        case CAR_STATE_TURNING_LEFT_BACKWARD: //car header direction change
            set_speed_left_right(TURNING_SPEED_MAX+active_config->speed_base, TURNING_SPEED_MIN+active_config->speed_base);
            break;
    }
}

//set the motor speeds and start motors to  move forward
void Motor::move_car_forward() {
    if (m_car_state==CAR_STATE_MOVING_FORWARD)
        return;
    if (debug)
        cout << "%%%%%Moving forward" << endl;
    set_speed(CAR_STATE_MOVING_FORWARD);
    start_motor(MOTORS_LEFT, DIR_FORWARD);
    start_motor(MOTORS_RIGHT,DIR_FORWARD);
    m_car_state=CAR_STATE_MOVING_FORWARD;
}

//set the motor speeds and start motors to  move backward
void Motor::move_car_backward(){
    if (m_car_state==CAR_STATE_MOVING_BACKWARD)
        return; 
    if (debug)
        cout << "%%%%%Moving backward" << endl;
    set_speed(CAR_STATE_MOVING_BACKWARD);
    start_motor(MOTORS_LEFT, DIR_BACKWARD);
    start_motor(MOTORS_RIGHT,DIR_BACKWARD);
    m_car_state=CAR_STATE_MOVING_BACKWARD;
}

//set the motor speeds and turn the car to the left
//in this case right motors will be a little bit fast than left motors
void Motor::turn_car_left_forward() {
    if (m_car_state==CAR_STATE_TURNING_LEFT_FORWARD)
        return;
    if (debug)
        cout << "%%%%%Turning left forward" << endl;
    set_speed(CAR_STATE_TURNING_LEFT_FORWARD); 
    start_motor(MOTORS_LEFT, DIR_FORWARD);
    start_motor(MOTORS_RIGHT,DIR_FORWARD);
    m_car_state=CAR_STATE_TURNING_LEFT_FORWARD;
}

//set the motor speeds and turn the car to the right
//in this case left motors will be a little bit fast than right motors
void Motor::turn_car_right_forward() {
    if (m_car_state==CAR_STATE_TURNING_RIGHT_FORWARD)
        return;
    if (debug)
        cout << "%%%%%Turning right forward" << endl;
    set_speed(CAR_STATE_TURNING_RIGHT_FORWARD);
    start_motor(MOTORS_LEFT, DIR_FORWARD);
    start_motor(MOTORS_RIGHT,DIR_FORWARD);
    m_car_state=CAR_STATE_TURNING_RIGHT_FORWARD;
}

//set the motor speeds and move the car backward. the car will turn a little bit to the left
void Motor::turn_car_left_backward() {
    if (m_car_state==CAR_STATE_TURNING_LEFT_BACKWARD)
        return;
    if (debug)
        cout << "%%%%%Turning left backward" << endl;
    set_speed(CAR_STATE_TURNING_LEFT_BACKWARD); 
    start_motor(MOTORS_LEFT, DIR_BACKWARD);
    start_motor(MOTORS_RIGHT,DIR_BACKWARD);
    m_car_state=CAR_STATE_TURNING_LEFT_BACKWARD;
}

//set the motor speeds and move the car backward. the car will turn a little bit to the right
void Motor::turn_car_right_backward() {
    if (m_car_state==CAR_STATE_TURNING_RIGHT_BACKWARD)
        return;
    if (debug)
        cout << "%%%%%Turning right backward" << endl;
    set_speed(CAR_STATE_TURNING_RIGHT_BACKWARD);
    start_motor(MOTORS_LEFT, DIR_BACKWARD);
    start_motor(MOTORS_RIGHT,DIR_BACKWARD);
    m_car_state=CAR_STATE_TURNING_RIGHT_BACKWARD;
}

//stop all car motors
void Motor::stop_car() {
    if (m_car_state==CAR_STATE_STOPPED)
        return;
    if (debug)
        cout << "%%%%%Car Stopped" << endl;
    stop_motor(MOTORS_LEFT);
    stop_motor(MOTORS_RIGHT);
    m_car_state=CAR_STATE_STOPPED;
}

//rotatation in-situ: turning the car at its current position.
//this function will be running in a background thread.
//@param arg - pointer to this class's instance
void* Motor::_rotate_car_bg(void *arg) {
    Motor *the_motor = (Motor*)arg;
    
    int state =the_motor->rotate_to_state;

    bool clockwise = (state == CAR_STATE_ROTATING_RIGHT_FAST || state == CAR_STATE_ROTATING_RIGHT_SLOW);
    bool fast = (state == CAR_STATE_ROTATING_LEFT_FAST || state == CAR_STATE_ROTATING_RIGHT_FAST);
    int idle_speed = 2;
    int speed = the_motor->active_config->speed_base + (fast ? ROTATING_SPEED_FAST : ROTATING_SPEED_SLOW);
    int duration_ms= fast ? 1000 : 2000;
    int step = 100;

    int dir = DIR_FORWARD; bool off = false;
    int left_speed = 0, right_speed = 0;
    while (!the_motor->is_paused() && !off) {
        if ((dir == DIR_FORWARD && clockwise) || (dir == DIR_BACKWARD && !clockwise)) {
            left_speed = speed;
            right_speed= idle_speed;
        }
        else {
            left_speed = idle_speed;
            right_speed= speed;
        }
        the_motor->set_speed_left_right(left_speed, right_speed);
        the_motor->start_motor(MOTORS_LEFT,  dir);
        the_motor->start_motor(MOTORS_RIGHT, dir);
        for (int i = 0; i < (duration_ms / step); ++i) {
            if (the_motor->m_car_state != state) {
                off = true;
                break;
            }
            Utils::delay_ms(step);
        }
        dir = 1 - dir; //reverse
    }
    if (the_motor->debug)
        cout << "Exited from rotating car thread ...." << endl;

    return NULL;
}

//Rotate the car in a background thread
void Motor::_rotate_car(int to_state) {
    rotate_to_state = to_state;
    pthread_create(&the_thread, NULL, _rotate_car_bg, this);
}

//rotate the car fast.
//@param _direction: left or right
void Motor::rotate_car_fast(int _direction) {
    int desired_car_states = (_direction == TURNING_DIRECTION_RIGHT) ? CAR_STATE_ROTATING_RIGHT_FAST : CAR_STATE_ROTATING_LEFT_FAST;
    if (m_car_state == desired_car_states)
        return;
    if (debug)
        cout << "%%%%%Rotating car fast " << desired_car_states << endl;
    _rotate_car(desired_car_states);
    m_car_state = desired_car_states;
}

//rotate the car slowly.
//@param _direction: left or right
void Motor::rotate_car_slow(int _direction) {
    int desired_car_states = (_direction == TURNING_DIRECTION_RIGHT) ? CAR_STATE_ROTATING_RIGHT_SLOW : CAR_STATE_ROTATING_LEFT_SLOW;
    if (m_car_state == desired_car_states)
        return;
    if (debug)
        cout << "%%%%%Rotating car slow " << desired_car_states << endl;
    _rotate_car(desired_car_states);
    m_car_state = desired_car_states;
}

//turn the motor on
void Motor::start_collector() {
    if (m_collector_state==COLLECTOR_STATE_RUNNING)
        return;
    start_motor(MOTOR_COLLECTOR, DIR_FORWARD);
    m_collector_state=COLLECTOR_STATE_RUNNING;
}
//turn the motor off
void Motor::stop_collector() {
    if (m_collector_state==COLLECTOR_STATE_STOPPED)
        return;
    stop_motor(MOTOR_COLLECTOR);
    m_collector_state=COLLECTOR_STATE_STOPPED;
}

int Motor::get_car_state() {
    return m_car_state;
}

//moving the car back and forth to get its speed
int Motor::measure_speed(Location *p_location) {
    int duration_s = 4;

    long distance1 = p_location->measure_front_distance();
    move_car_forward();
    Utils::delay_ms(duration_s * 1000);
    stop_car();
    Utils::delay_ms(500); //wait for 1/2 second, then move back
    long distance2 = p_location->measure_front_distance();
        
    long distance3 = p_location->measure_front_distance();
    move_car_backward();
    Utils::delay_ms(duration_s * 1000);
    stop_car();
    Utils::delay_ms(500); //wait for 1/2 second
    long distance4 = p_location->measure_front_distance();

    long speed1 = (distance1 - distance2) / duration_s;
    long speed2 = (distance4 - distance3) / duration_s;
    
    return ((speed1 > speed2) ? speed1 : speed2);
}

//Move the car back and forth to get its speed and inc/dec the speed_base to adjust its speed to 
//meet the one defined by the TARGET_CALIBRATION_SPEED
bool Motor::calibrate(Location *p_location) {
    Led::turn_on_red_led();//we are going back to pause state
    Led::buzzle(false);
    
    int using_pin = PIN_LED_BLUE;
    Led::turn_on_led(using_pin);
    int speed = 0, allowed_deviation=2;
    bool calibrated = false;
    long cutoff = Utils::current_time_ms() + 60 * 1000;//set a time limitation
    while (!is_done() && !calibrated) {
        speed = measure_speed(p_location);
        if (debug)
            cout << "Current speed " << speed << "cm/s" << ",adjustment " << active_config->speed_base << endl;
        if (abs(speed - TARGET_CALIBRATION_SPEED) <= allowed_deviation) {
            calibrated = true;
        }
        else {
            if (speed < TARGET_CALIBRATION_SPEED) {
                m_config->get_active_config()->speed_base += 2;
            }
            else if (speed > TARGET_CALIBRATION_SPEED){
                m_config->get_active_config()->speed_base -= 2;
            }
        }
        if (Utils::current_time_ms() >= cutoff)
            break;
    }
    Led::turn_off_led(using_pin);
    Led::buzzle(!calibrated);
    
    return calibrated;
}
