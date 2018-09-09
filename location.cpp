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
#include <sys/time.h>

#include <wiringPi.h>

#include "location.h"
#include "utils.h"
#include "pins.h"

using namespace std;

//the constructor with configuration
Location::Location(Config *config) : Component(config) {
}

Location::~Location() {
}

//sometimes the front ultra sound sensor reports false distance.
//this function tries to filter out these false results.
long Location::measure_front_distance_ex(void) {
    long distance = measure_front_distance();
    long timestamp = Utils::current_time_ms() + 2000;//try at most two seconds
    while (distance > 1000) {//a false measurement
        Utils::delay_ms(200);
        distance = measure_front_distance();
        if (Utils::current_time_ms() >= timestamp)
            break;
    }
    return distance;
}

//measure the front or rear obstacle distance in cm
long Location::measure_distance(int pin_trig, int pin_echo) {
    struct timeval tv1;
    struct timeval tv2;
    long start, stop;

    digitalWrite(pin_trig, LOW);
    delayMicroseconds(2);

    digitalWrite(pin_trig, HIGH);
    delayMicroseconds(10);  
    digitalWrite(pin_trig, LOW);
    while (!(digitalRead(pin_echo) == 1));
    gettimeofday(&tv1, NULL);   

    while(!(digitalRead(pin_echo) == 0));
    gettimeofday(&tv2, NULL);           

    start = tv1.tv_sec * 1000000 + tv1.tv_usec;
    stop  = tv2.tv_sec * 1000000 + tv2.tv_usec;

    return (long)((float)(stop - start) / 1000000 * 34000 / 2);
}

//front obstracle distance in cm
long Location::measure_front_distance(void) {
    //This sensor sometimes reports a wrong distance as 4cm
    static long last_distance = 65535; //always ignore the first measurement
    long distance = measure_distance(PIN_TRIG_FRONT, PIN_ECHO_FRONT);
    long movement = abs(distance - last_distance);
    last_distance = distance;
    if ((movement > 40) && (distance <= 10 || last_distance <= 10)) {//take it as an invalid measurement
        if (debug)
            cout << "False distance " << distance << endl;
        distance = 65535;
    }
    return distance;
}

//rear obstacle distance in cm
long Location::measure_rear_distance(void) {
    return measure_distance(PIN_TRIG_REAR, PIN_ECHO_REAR);
}

//override, initialize the pins
bool Location::init () {
    Component::init();
    
    pinMode(PIN_ECHO_FRONT, INPUT);
    pinMode(PIN_TRIG_FRONT, OUTPUT);
    pinMode(PIN_ECHO_REAR, INPUT);
    pinMode(PIN_TRIG_REAR, OUTPUT);

    return true;    
}
