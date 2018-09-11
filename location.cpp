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
    front_idx = rear_idx = 0;
}

Location::~Location() {
}

void Location::start() {
    Component::start();
    //as the front sensor sometimes report a false distance, we will measure the distance for several times, 
    //then calculate the average after removing the biggest and shortest distances.
    for (int i = 0; i < SAMPLE_SIZE; ++i) {
        front[i]=measure_distance(PIN_TRIG_FRONT, PIN_ECHO_FRONT);
        rear[i] =measure_distance(PIN_TRIG_REAR, PIN_ECHO_REAR);
    }
    front_idx = rear_idx = SAMPLE_SIZE - 1;
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

//calculate the average distance after removing the nearest and farest distances
long Location::calculate_average(long *distances) {
    long smallest = distances[0], biggest = distances[0];
    for (int i = 1; i < SAMPLE_SIZE; ++i) {
        if (distances[i] < smallest)
            smallest = distances[i];
        else if (distances[i] > biggest) {
            biggest = distances[i];
        }
    }
    long sum = 0; int count = 0;
    for (int i = 0; i < SAMPLE_SIZE; ++i) {
        if (distances[i] == smallest) {
            smallest = -1;//remove only one "smallest"
        }
        else if (distances[i] == biggest) {
            biggest = -1; //remove only one "biggest"
        }
        else {
            ++count;
            sum += distances[i];
        }
    }
    return sum / count;
}

//front obstracle distance in cm
long Location::measure_front_distance(void) {
    front_idx = (front_idx + 1) % SAMPLE_SIZE;
    front[front_idx]=measure_distance(PIN_TRIG_FRONT, PIN_ECHO_FRONT);
    return calculate_average(front);
}

//rear obstacle distance in cm
long Location::measure_rear_distance(void) {
    rear_idx = (rear_idx + 1) % SAMPLE_SIZE;
    rear[rear_idx]=measure_distance(PIN_TRIG_REAR, PIN_ECHO_REAR);
    return calculate_average(rear);
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
