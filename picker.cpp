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

#include "picker.h"
#include "utils.h"
#include "led.h"
#include "pins.h"

using namespace std;
//constructor with the configuration
Picker::Picker(Config *config) {
    m_config = config;
    //additional initialization
    debug = m_config->is_debug();
    m_user_action = UA_NAV2_RESUME;
    m_interruption = INT_NONE;
    memset (&m_context, 0, sizeof (RobotCtx));
}
//initialize all required components
bool Picker::init() {
    m_vision = new Vision(m_config);
    m_motor = new Motor(m_config);
    m_location = new Location(m_config);
    m_observer = new Observer(m_config, m_motor, m_location, &m_interruption, &m_user_action);
    
    bool succeed = m_motor->init();
    succeed &= m_location->init();
    succeed &= m_vision->init();
    succeed &= m_observer->init(); //depends on m_motor and m_location, last one to initialize

    return succeed;
}

//set start up user action, may pause the program at start up
void Picker::set_user_action(int act) {
    m_user_action = act;
}

//start to pick up balls
void Picker::run() {
    m_vision->start();
    m_motor->start();
    m_location->start();
    m_observer->start();
    while (m_user_action != UA_DONE) {
        switch (m_user_action) {
            case UA_NONE:
                picking_up();
                switch (m_interruption) {
                    case INT_FRONT_OBSTACLE:
                    case INT_REAR_OBSTACLE:
                        workaround_obstacle();
                        m_interruption = INT_NONE;
                        break;
                    case INT_NO_MORE_BALLS:
                        if (m_user_action != UA_DONE) {
                            m_user_action=UA_NAV2_PAUSE; //auto pause
                            m_interruption = INT_NONE;
                            Led::buzzle(true);
                        }
                        break;
                    default:
                        Utils::delay_ms(m_config->get_frame_time_ms());//go to pick up another ball
                        break;
                }//inner switch
                break;
            case UA_NAV2_PAUSE:
                m_motor->stop_car();
                m_motor->stop_collector();
                m_vision->pause(true);
                Led::turn_on_red_led();
                m_user_action=UA_PAUSE;
                break;
            case UA_NAV2_RESUME:
                m_motor->start_collector();
                m_vision->pause(false);
                Led::turn_off_red_led();
                //the observer should still be running (not paused)
                m_user_action = UA_NONE;
                break;
            case UA_MOTOR_CALIBRATION:
                if (m_motor->calibrate(m_location)) {
                    m_config->save_config();
                }
                if (m_user_action != UA_DONE) {
                    m_user_action=UA_NAV2_PAUSE;
                }
                break;
            case UA_CAMERA_CALIBRATION:
                if (m_vision->calibrate()) {
                    m_config->save_config();
                }
                if (m_user_action != UA_DONE) {
                    m_user_action=UA_NAV2_PAUSE;
                }
                break;
            case UA_DEBUG:
                cout << "Front distance " << m_location->measure_front_distance() << endl;
                m_motor->rotate_car_slow(TURNING_DIRECTION_LEFT);
                Utils::delay_ms(200);
                break;
            default: //UA_PAUSE, UA_PRE_MOTOR_CALIBRATION, UA_PRE_CAMERA_CALIBRATION, UA_WAITING etc
                Utils::delay_ms(1000);
                break;
        }//switch
    } //while
    
    m_observer->stop();
    m_vision->stop();
    m_motor->stop();
    m_location->stop();
}

//stop the picker, cannot be resumed
void Picker::stop() {
    m_user_action = UA_DONE;
}

void Picker::de_init() {
    delete m_vision;
    delete m_motor;
    delete m_location;
    delete m_observer;
}


//determine what direction we should turn the car
//@param recovering - true if we just lost tracking a ball and wanted to get it back
int Picker::choose_turning_driection(bool recovering) {
    int direction = TURNING_DIRECTION_LEFT;
    if (recovering) {
        //we lost the ball, try to get it back
        if (m_context.last_scene_w_balls.angle <= 0) {
            if (debug)
                cout << "Decision 1" << endl;
        }
        else {
            if (debug)
                cout << "Decision 2" << endl;
            direction = TURNING_DIRECTION_RIGHT;
        }
    }
    else {
        //after we picked up one ball
        if (m_context.last_scene_w_balls.balls_at_left > 0 &&  m_context.last_scene_w_balls.balls_at_right > 0) {
            if (m_context.last_scene_w_balls.nearest_ball_at_left < m_context.last_scene_w_balls.nearest_ball_at_right) {
                if (debug)
                    cout << "Decision 3" << endl;
                direction = TURNING_DIRECTION_RIGHT;
            }
            else if (m_context.last_scene_w_balls.nearest_ball_at_left > m_context.last_scene_w_balls.nearest_ball_at_right) {
                if (debug)
                    cout << "Decision 4" << endl;
            }
            else {
                if (debug)
                    cout << "Decision 5" << endl;
                direction=m_context.last_turn_direction;
            }
        }
        else if (m_context.last_scene_w_balls.balls_at_left > 0) {
            if (debug)
                cout << "Decision 6" << endl;
        }
        else if (m_context.last_scene_w_balls.balls_at_right > 0) {
            if (debug)
                cout << "Decision 7" << endl;
            direction = TURNING_DIRECTION_RIGHT;
        }
        else if (m_context.last_scene_w_balls.angle <= 0) {
            if (debug)
                cout << "Decision 8, no known balls left" << endl;
        }
        else {
            if (debug)
                cout << "Decision 9, no known balls left" << endl;
            direction=m_context.last_turn_direction;
        }
    }
    if (direction == TURNING_DIRECTION_UNKNOWN)
        direction = TURNING_DIRECTION_RIGHT;
    return direction;
}
//workaround the front or rear obstacle. do nothing for rear obstacle as the car already stopped.
void Picker::workaround_obstacle() {
    if (debug)
        cout << "Workaround obstacle" << endl;
    if (m_interruption == INT_FRONT_OBSTACLE) {
        m_motor->move_car_backward();
        Utils::delay_ms(2000);
        if (m_user_action == UA_DONE)
            return;
        int direction = choose_turning_driection(false);
        m_context.last_turn_direction = direction;
        m_motor->rotate_car_fast(direction);
        Utils::delay_ms(1000);
    }
    else if (m_interruption == INT_REAR_OBSTACLE) {
        //car already stopped, do nothing here
    }
}

//get one scene
//@returns true if one updated scene is available and retrieved, otherwise false.
bool Picker::get_stable_scene() {
    if (m_context.scene.balls > 1) {
        memcpy (&m_context.last_scene_w_balls, &m_context.scene, sizeof (Scene));
    }
    return m_vision->get_stable_scene(&m_context.scene);
}

//check if the target ball is still in the right position
//@param angle - the ball angle
//@pram distance - the ball distance in pixels
//@param strict - true if the ball should be very close to the center so that it can be picked up.
bool Picker::is_covered_raw(int angle, int distance, bool strict) {
    int abs_angle = abs(angle);
    int target_angle = 0;
    if (distance <= PIXEL_DISTANCE_PICK_FAR) {
        target_angle = PICKUP_ANGLE_FAR + (PICKUP_ANGLE_NEAR - PICKUP_ANGLE_FAR) * (PIXEL_DISTANCE_PICK_FAR - distance) / (PIXEL_DISTANCE_PICK_FAR - PIXEL_DISTANCE_PICK_NEAR);
    }
    else if (strict)
        target_angle = PERFECT_ANGLE; //i.e. 10 degree
    else
        target_angle = GOOD_ANGLE; //i.e. 30 degree
    return abs_angle <= target_angle;
}

//check if the target ball is still in the right position
bool Picker::is_covered(bool strict) {
    if (m_context.scene.balls <= 0)
        return false; //something wrong
    return is_covered_raw(m_context.scene.angle, m_context.scene.distance, strict);
}

//@return true if the ball is close enough and in allowed angle
bool Picker::is_ready_pickup() {
    if (m_context.scene.balls <= 0)
        return false;
    return (m_context.scene.distance <= PIXEL_DISTANCE_PICK_FAR) && is_covered(true);
}

//find one ball (turn the car around if needed) and adjust the car direction to make sure the ball is at 
//good positition so that it can be picked up later.
//@param recovering - true if we just lost the "nearest" ball and want to find it again.
//@returns true if at least one ball is found
bool Picker::targeting (bool recovering) {
    if (debug)
        cout << "@@@Targeting, recovering?" << recovering << endl;
    bool found = false;
    if ((m_context.scene.balls <= 0) || abs(m_context.scene.angle) > GOOD_ANGLE) {
        //step 1, rotate the car until we see a ball
        int direction = TURNING_DIRECTION_UNKNOWN;
        if (m_context.scene.balls > 0) {
            direction = m_context.scene.angle > 0 ? TURNING_DIRECTION_RIGHT : TURNING_DIRECTION_LEFT;
        }
        else {
            direction = choose_turning_driection(recovering);
        }
        if ((m_context.scene.balls <= 0) && m_context.consecutive_collected > 0) {
            if (direction == TURNING_DIRECTION_RIGHT) {
                m_motor->turn_car_right_backward();
            }
            else {
                m_motor->turn_car_left_backward();
            }
            Utils::delay_ms(BACK_AFTER_PICKUP_MS);
        }
        long till_ms = Utils::current_time_ms() + MAX_TURNING_90_MS * 4;//for 360 degree
        if (!recovering) {
            m_motor->rotate_car_fast(direction);
            found = false;
            while (!found && (m_interruption == INT_NONE) && (Utils::current_time_ms() < till_ms)) {
                Utils::delay_ms(m_config->get_frame_time_ms()>>1);
                if (m_user_action == UA_DONE || m_user_action == UA_PAUSE || !get_stable_scene()) {
                    break;
                }
                if (m_context.scene.balls > 0) {
                    found = true;
                }
            }
            //The car already turned one round, no ball found
            if (!found)
                return false;
        }
        m_motor->rotate_car_slow(direction);
        found = false;
        while (!found && (m_interruption == INT_NONE) && (Utils::current_time_ms() < till_ms)) {
            Utils::delay_ms(m_config->get_frame_time_ms()>>1);
            if (m_user_action == UA_DONE || m_user_action == UA_PAUSE || !get_stable_scene())
                break;
            if (m_context.scene.balls > 0) {
                if (abs(m_context.scene.angle) <= PERFECT_ANGLE) {
                    found = true;
                }
                else if ((m_context.scene.angle < 0) && (direction == TURNING_DIRECTION_LEFT))
                    continue;
                else if ((m_context.scene.angle > 0) && (direction == TURNING_DIRECTION_RIGHT))
                    continue;
                else {
                    found = true;
                }
            }
        }
        if (!found)
            return false;
    }
    if (found && (is_ready_pickup() || abs(m_context.scene.angle) <= PERFECT_ANGLE))
        return found;
    int last_angle = m_context.scene.angle;
    int direction = (last_angle > 0 ? TURNING_DIRECTION_RIGHT : TURNING_DIRECTION_LEFT);
    //If the car is too close to the ball, move back a little bit
    if (!is_covered(false)) {
        if (direction == TURNING_DIRECTION_RIGHT) {
            m_motor->turn_car_right_backward();
        }
        else if (direction == TURNING_DIRECTION_LEFT) {
            m_motor->turn_car_left_backward();
        }
        Utils::delay_ms(1000);
    }
    //step 2, rotating the car to target the ball
    m_motor->rotate_car_slow(direction);
    { 
        //reset again
        found = false;
        long till_ms = Utils::current_time_ms() + MAX_TURNING_90_MS * 4;
        int this_angle = last_angle;
        while (!found && (m_interruption == INT_NONE) && (Utils::current_time_ms() < till_ms)) {
            Utils::delay_ms(m_config->get_frame_time_ms()>>1);
            if (m_user_action == UA_DONE || m_user_action == UA_PAUSE || !get_stable_scene())
                break;
            if (m_context.scene.balls > 0) {
                this_angle = m_context.scene.angle;
                if (is_ready_pickup() || abs(m_context.scene.angle) <= PERFECT_ANGLE || (this_angle * last_angle < 0)) {//direction reversed
                    found = true;
                }
                last_angle = this_angle;
            }
        }
    }
    return found;
}

//when the car is moving towards the target ball, make sure the ball is still in good position.
//if the ball is a little off the path, adjust the car direction.
bool Picker::tracking() {
    if (debug)
        cout << "@@@Tracking ball" << endl;
    
    int total_counter=0;
    int counter_left = 0, counter_right = 0;//how many times we saw the ball at left or right
    bool ready_to_pickup = false;
    //tracking the ball
    while (m_interruption == INT_NONE && m_user_action == UA_NONE && get_stable_scene()) {
        if (m_context.scene.balls > 0) {
            if (is_ready_pickup()) {
                ready_to_pickup = true;
                break;
            }
            else if (is_covered(false)) {//still within allowed range
                int ball_angle = m_context.scene.angle;
                int abs_angle = abs(ball_angle);
                if (abs_angle >= PERFECT_ANGLE * 6 / 10) {//a little bit off the road
                    if (ball_angle < 0)
                        ++counter_left;
                    else if (ball_angle > 0)
                        ++counter_right;
                }
                if (++total_counter >= 2) {//we saw this ball two times
                    bool turned = false;
                    if (counter_left >= 2 || counter_right >= 2) { 
                        //the ball is "moving away" from the car
                        if (counter_left > counter_right) {
                            m_motor->turn_car_left_forward();
                            turned = true;
                        }
                        else if (counter_left < counter_right) {
                            m_motor->turn_car_right_forward();
                            turned = true;
                        }
                    }
                    if (!turned) {
                        m_motor->move_car_forward();//full speed
                    }
                    total_counter = counter_left = counter_right = 0;
                }
            }
            else if (targeting(true)) {//the ball is out of path
                m_motor->move_car_forward();
            }
        }
        else {
            //the car may move so quick
            ready_to_pickup = true;
            break;
        }
    }
    return ready_to_pickup;
}

//search and find one ball then move the car forward to try to pick up the ball
//@returns true if one ball is found and can be picked up later
bool Picker::searching() {
    if (debug)
        cout << "@@@Searching" << endl;
    
    if (!get_stable_scene ())
        return false;

    bool found = false;
    if ((m_context.scene.balls > 0) && is_covered(true))
        found = true;
    else {
        //turn around to find if there are more balls
        found = targeting(false);
        if ((m_interruption == INT_NONE) && m_context.scene.balls <= 0)
            m_interruption = INT_NO_MORE_BALLS;
    }
    if (found)
        m_motor->move_car_forward();
    return found;
}

//search to find one ball and pick it up
void Picker::picking_up() {
    if (debug)
        cout << "@@@Picking up" << endl;
    bool picked_one = false;
    if (m_user_action == UA_NONE) {
        if (searching()) {
            if (debug)
                cout << "**Found the ball to be picked up, angle " << m_context.scene.angle << endl;
            m_motor->move_car_forward();
            if (tracking()) {
                if (debug)
                    cout << "==============================>Ready to pickup" << endl;
                //wait for the ball to be out of scene while the car is moving forward
                m_motor->move_car_forward();
                int last_distance = m_context.scene.distance;
                long wait_until = Utils::current_time_ms() + 3000; //set a time limitation
                get_stable_scene();
                while (m_context.scene.balls > 0 && m_context.scene.distance <= last_distance) {
                    if (Utils::current_time_ms() > wait_until)
                        break;
                    last_distance = m_context.scene.distance;
                    Utils::delay_ms(m_config->get_frame_time_ms());
                    get_stable_scene();
                }
                if (m_user_action == UA_NONE && m_interruption == INT_NONE) {//we are still moving
                    Led::turn_on_led(PIN_LED_GREEN);//green led on
                    Utils::delay_ms(WAIT_BALL_OUT_OF_SCENE_MS);
                    m_motor->stop_car();
                    Utils::delay_ms(WAIT_BALL_PICKUP_MS);
                    Led::turn_off_led(PIN_LED_GREEN);//green led off
                    ++m_context.balls_collected;
                    picked_one = true;
                }
            }
        }
    }
    if (picked_one)
        ++m_context.consecutive_collected;
    else
        m_context.consecutive_collected = 0;
}

