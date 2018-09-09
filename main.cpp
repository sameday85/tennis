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
#include <stdlib.h>
#include <signal.h>

#include <wiringPi.h>

#include "picker.h"
#include "led.h"
#include "utils.h"
#include "pins.h"

using namespace std; 

Config *g_config = NULL;
Picker *g_picker = NULL;

//stop the program when ctr+c is received
void handle_signal(int signal) {
    // Find out which signal we're handling
    switch (signal) {
        case SIGINT://key 3
            if (g_picker)
                g_picker->stop();
            break;
    }
}

void hook_signal() {
    struct sigaction sa;
    
    // Setup the sighub handler
    sa.sa_handler = &handle_signal;
    // Restart the system call, if at all possible
    sa.sa_flags = SA_RESTART;
    if (sigaction(SIGINT, &sa, NULL) == -1) {
        printf("Failed to capture sigint 1\n");
    }
}

int main ( int argc,char **argv ) {
    hook_signal();
    wiringPiSetup();

    g_config = new Config();
    g_config->load_config();
    
    int startup_action = UA_NONE;
    for (int i = 1; i < argc; ++i) {
        if (strcmp (argv[i], "-boot") == 0) {//auto start on reboot
            //All leds on to indicate initialization
            Led::turn_on_red_led();
            Led::turn_on_led(PIN_LED_GREEN);
            Led::turn_on_led(PIN_LED_BLUE);
            Utils::delay_ms(10 * 1000); //wait for 10 seconds
            g_config->activate_outdoor_config();
            g_config->set_debug(false);
            startup_action = UA_NAV2_PAUSE;
            //turn off these two leds but keep red led on
            Led::turn_off_led(PIN_LED_GREEN);
            Led::turn_off_led(PIN_LED_BLUE);
            Led::buzzle(false);
        }
        else if (strcmp (argv[i], "-pause") == 0) {
            startup_action = UA_NAV2_PAUSE;
        }
        else if (strcmp (argv[i], "-debug") == 0) {
            startup_action = UA_DEBUG;
        }
    }
    
    g_picker = new Picker(g_config);
    g_picker->set_user_action(startup_action);
    g_picker->init();
    g_picker->run(); //run until ctl+c is received
    Utils::delay_ms(1000);//waiting for all background threads to be stopped
    g_picker->de_init();
    g_config->save_config();

    delete g_picker;
    delete g_config;

    return 0;
}

