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
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>

#include "config.h"

/*
 * Implementation of the Config class
 */
 
using namespace std; 
 
//the constructor
Config::Config() {
    //by default to use the indoor configuration
    activate_indoor_config();
    //by default the debug is on
    m_debug = true;
}

//load the configuration from the file
void Config::load_config() {
    //default values
    frames_per_second = 4;
    frame_time_ms = 1000 / frames_per_second;
    
    //initialize configurations, speed_base is now zero
    memset (&indoor_config, 0, sizeof (RobotConfig));
    indoor_config.minH = 60;  indoor_config.maxH = 90; //0-180
    indoor_config.minS = 170; indoor_config.maxS = 255; //0-255
    indoor_config.minV = 70;  indoor_config.maxV = 155; //0-255
    indoor_config.erosion_size = 3;
    indoor_config.dilation_size= 4;
    indoor_config.canny_thresh = 100;
    indoor_config.min_area = 10;
    memcpy(&outdoor_config, &indoor_config, sizeof (RobotConfig));
    
    //parse the configuration file
    RobotConfig *config = NULL;
    ifstream input(CONFIG_FILE, ifstream::in); //The input stream
    string line;
    while (input) {
        getline(input, line, '\n');
        if (line.find_first_of("#") == 0)
            continue;
        if (line.find_first_of("[") == 0) {
            if (line.compare("[indoor]") == 0) {
                config = &indoor_config;
            }
            else if (line.compare("[outdoor]") == 0) {
                config = &outdoor_config;
            }
        }
        else if (config) {
            string::size_type pos = line.find_first_of("=");
            if (pos != string::npos) {
                string key   = line.substr(0, pos);
                string value = line.substr(pos+1);
                if (key.compare("minH") == 0) {
                    config->minH = stoi(value);
                }
                else if (key.compare("maxH") == 0) {
                    config->maxH = stoi(value);
                }
                else if (key.compare("minS") == 0) {
                    config->minS = stoi(value);
                }
                else if (key.compare("maxS") == 0) {
                    config->maxS = stoi(value);
                }
                else if (key.compare("minV") == 0) {
                    config->minV = stoi(value);
                }
                else if (key.compare("maxV") == 0) {
                    config->maxV = stoi(value);
                }
                else if (key.compare("speedBase") == 0) {
                    config->speed_base = stoi(value);
                }
            }
        }
    }
    input.close();
}

//save configuration to the file
void Config::save_config() {
    std::ofstream out(CONFIG_FILE);
    
    out << "[indoor]" << endl; //do not change this section name
    out << "minH=" << indoor_config.minH << endl;
    out << "maxH=" << indoor_config.maxH << endl;
    out << "minS=" << indoor_config.minS << endl;
    out << "maxS=" << indoor_config.maxS << endl;
    out << "minV=" << indoor_config.minV << endl;
    out << "maxV=" << indoor_config.maxV << endl;
    out << "speedBase=" << indoor_config.speed_base << endl;
    
    out << "[outdoor]" << endl; //do not change this section name
    out << "minH=" << outdoor_config.minH << endl;
    out << "maxH=" << outdoor_config.maxH << endl;
    out << "minS=" << outdoor_config.minS << endl;
    out << "maxS=" << outdoor_config.maxS << endl;
    out << "minV=" << outdoor_config.minV << endl;
    out << "maxV=" << outdoor_config.maxV << endl;
    out << "speedBase=" << outdoor_config.speed_base << endl;
    
    out.close();
}

//set the debug mode
//@param d - true in debug mode
void Config::set_debug (bool d) {
    m_debug = d;
}

//@returns if the system is in debug mode
bool Config::is_debug() {
    return m_debug;
}

//@return the configuration of frames per seconds
int Config::get_frames_per_second() {
    return frames_per_second;
}

//@returns each frame time calculated based on the frames_per_seconds
int Config::get_frame_time_ms() {
    return frame_time_ms;
}

void Config::activate_indoor_config() {
    active_config = &indoor_config;
}

void Config::activate_outdoor_config() {
    active_config = &outdoor_config;    
}

RobotConfig *Config::get_active_config() {
    return active_config;
}


