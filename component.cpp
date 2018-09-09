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
#include "component.h"
/*
 * Implementation of Component class. This is the base class of Vision, Location etc
 */
 
//the constructor with configuration
Component::Component(Config *config) {
    m_config = config;
    m_paused = m_done = false;
    debug = config->is_debug();
}

Config *Component::get_config() {
    return m_config;
}

//do nothing here
bool Component::init() {
    return true;
}
//start the sensor
void Component::start() {
    m_paused = m_done = false;
}
//pause the sensor
void Component::pause(bool b) {
    m_paused = b;
}
//stop the sensor
void Component::stop() {
    m_paused = m_done = true;
}

bool Component::is_paused() {
    return m_paused;
}

bool Component::is_done() {
    return m_done;
}

