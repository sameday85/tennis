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
    m_context.algorithm=ALGORITHM_NEAREST_FIRST;
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

//@returns true if no interruption and not paused.
bool Picker::should_continue() {
    if ((m_user_action == UA_DONE) || (m_user_action == UA_PAUSE))
        return false;
    return (m_interruption == INT_NONE);
}

//start to pick up balls
void Picker::run() {
    m_vision->start();
    m_motor->start();
    m_location->start();
    m_observer->start();
    Utils::delay_ms(1000);//let the camera warms up
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
                //cout << "Front distance " << m_location->measure_front_distance() << endl;
                m_motor->rotate_car_fast(TURNING_DIRECTION_LEFT);
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
    if (debug)
        cout << "Stopping ...................." << endl;
}

void Picker::de_init() {
    delete m_vision;
    delete m_motor;
    delete m_location;
    delete m_observer;
}


//determine what direction we should turn the car
int Picker::choose_turning_direction() {
    int direction = TURNING_DIRECTION_LEFT;
    
    if (m_context.scene.total_balls > 0) {
        direction = (m_context.scene.target_ball.angle > 0 ? TURNING_DIRECTION_RIGHT : TURNING_DIRECTION_LEFT);
        if (debug)
            cout << "Decision 1" << endl;
    }
    else if (m_context.this_turn_dir_hints != TURNING_DIRECTION_UNKNOWN) {
        direction = m_context.this_turn_dir_hints;
        if (debug)
            cout << "Decision 2" << endl;
    }
    
    if (direction == TURNING_DIRECTION_UNKNOWN)
        direction = TURNING_DIRECTION_RIGHT;

    return direction;
}

void Picker::reset_turning_hints() {
    if (m_context.next_turn_left_hints > m_context.next_turn_right_hints)
        m_context.this_turn_dir_hints = TURNING_DIRECTION_LEFT;
    else if (m_context.next_turn_left_hints < m_context.next_turn_right_hints) {
        m_context.this_turn_dir_hints = TURNING_DIRECTION_RIGHT;
    }
    m_context.next_turn_left_hints = m_context.next_turn_right_hints = 0;
}

//Envaluate balls at given side, returns a double value indicating how good to pick them up next
//for the score, it is the ball density, the smaller the better
double Picker::get_score(int side, int total, int max_y) {
    double x_factor = 1.5;
    if (max_y <= PIXEL_DISTANCE_PICK_FAR) {
        max_y = CAMERA_FRAME_HEIGHT; //all balls are good
    }
    int count = 0, sum_x = 0, sum_y = 0;
    for (int i = 0; i < m_context.scene.total_balls; ++i) {
        if (m_context.scene.all_balls[i].side == side) {
            //exclude the farest one
            if (m_context.scene.all_balls[i].y >= max_y)
                continue;
            ++count;
            sum_x += m_context.scene.all_balls[i].x;
            sum_y += m_context.scene.all_balls[i].y;
        }
    }
    if (count <= 0)
        return MAX_BALLS_AT_VENUE * CAMERA_FRAME_WIDTH * x_factor + MAX_BALLS_AT_VENUE * CAMERA_FRAME_HEIGHT;
    double avg_x = 1.0 * sum_x / count;
    double avg_y = 1.0 * sum_y / count;

    double diff_x = 0, diff_y = 0;
    for (int i = 0; i < m_context.scene.total_balls; ++i) {
        if (m_context.scene.all_balls[i].side == side) {
            if (m_context.scene.all_balls[i].y >= max_y)
                continue;
            diff_x += abs(m_context.scene.all_balls[i].x - avg_x);
            diff_y += abs(m_context.scene.all_balls[i].y - avg_y);
        }
    }
    //supress warning: unused parameter ‘total’
    if (total > 0) {
    }
    return diff_x * x_factor + diff_y;
}

//Analyse the current scene and try to predicate next turn after the current target ball is picked up
void Picker::analyse_scene() {
    if (m_context.scene.total_balls <= 1)
        return;
    if (m_context.scene.left_balls <= 0 && m_context.scene.right_balls <= 0)
        return;

    int max_center_y = 0, max_left_y = 0, max_right_y = 0;
    for (int i = 0; i < m_context.scene.total_balls; ++i) {
        if (m_context.scene.all_balls[i].side == BALL_SIDE_CENTER) {
            if (m_context.scene.all_balls[i].y > max_center_y)
                max_center_y = m_context.scene.all_balls[i].y;
        }
        else if (m_context.scene.all_balls[i].side == BALL_SIDE_RIGHT) {
            if (m_context.scene.all_balls[i].y > max_right_y)
                max_right_y = m_context.scene.all_balls[i].y;
        }
        else {
            if (m_context.scene.all_balls[i].y > max_left_y)
                max_left_y = m_context.scene.all_balls[i].y;
        }
    }
    double left_side_score = 0, right_side_score = 0;
    if (m_context.scene.left_balls > 0 && m_context.scene.right_balls <= 0) {
        //need to turn left
        right_side_score = 65535;
    }
    else if (m_context.scene.right_balls > 0 && m_context.scene.left_balls <= 0) {
        //need to turn right
        left_side_score = 65535;
    }
    else {
        left_side_score = get_score(BALL_SIDE_LEFT, m_context.scene.left_balls, max_left_y);
        right_side_score= get_score(BALL_SIDE_RIGHT, m_context.scene.right_balls, max_right_y);
    }
    if (left_side_score < right_side_score)
        ++m_context.next_turn_left_hints;
    else if (left_side_score > right_side_score) {
        ++m_context.next_turn_right_hints;
    }
    if (debug)
        cout << "------Left score " << left_side_score << ", right " << right_side_score << ", scene " << m_context.scene.seq << endl;
}

//workaround the front or rear obstacle. do nothing for rear obstacle as the car already stopped.
void Picker::workaround_obstacle() {
    if (debug)
        cout << "@@@@@@Workaround obstacle" << endl;
    m_motor->stop_car();
    if (m_interruption == INT_FRONT_OBSTACLE) {
        m_motor->move_car_backward();
        Utils::delay_ms(2000);
        if (m_user_action == UA_DONE)
            return;
        int direction = choose_turning_direction();
        m_motor->rotate_car_fast(direction);
        Utils::delay_ms(2000);
    }
    else if (m_interruption == INT_REAR_OBSTACLE) {
        //car already stopped, do nothing here
    }
}

//Find the target ball and also count how many balls are at its left, and
//how many balls are at its right
void Picker::consume_scene() {
    int total_balls = m_context.scene.total_balls;
    if (total_balls > 0) {
        int target_index = 0, target_value = 0;
        switch (m_context.algorithm) {
            case ALGORITHM_NEAREST_FIRST:
                for (int i = 0; i < total_balls; ++i) {
                    int distance = m_context.scene.all_balls[i].distance; 
                    if (i <= 0 || distance < target_value) {
                        target_value = distance;
                        target_index = i;
                    }
                }
                break;
            case ALGORITHM_RIGHTMOST_FIRST:
                for (int i = 0; i < total_balls; ++i) {
                    int angle = m_context.scene.all_balls[i].angle; 
                    if (i <= 0 || angle > target_value) {
                        target_value = angle;
                        target_index = i;
                    }
                }
                break;
        }//switch
        m_context.scene.all_balls[target_index].is_target = true;
        memcpy (&m_context.scene.target_ball, &m_context.scene.all_balls[target_index], sizeof (Ball));
    }
    if (total_balls > 1) {
        int target_angle = m_context.scene.target_ball.angle;
        for (int i = 0; i < total_balls; ++i) {
            if (m_context.scene.all_balls[i].is_target)
                continue;
            if (abs (m_context.scene.all_balls[i].angle - target_angle) <= PERFECT_ANGLE) {
                m_context.scene.all_balls[i].side = BALL_SIDE_CENTER;
                m_context.scene.center_balls++;
            }
            else if (m_context.scene.all_balls[i].angle < target_angle) {
                m_context.scene.all_balls[i].side = BALL_SIDE_LEFT;
                m_context.scene.left_balls++;
            }
            else if (m_context.scene.all_balls[i].angle > target_angle) {
                m_context.scene.all_balls[i].side = BALL_SIDE_RIGHT;
                m_context.scene.right_balls++;
            }
        }
    }
    if (debug && (total_balls > 0)) {
        cout << "#" << m_context.scene.seq << ": " << total_balls << ", angle " << m_context.scene.target_ball.angle << ", distance " << m_context.scene.target_ball.distance << ",L/R " << m_context.scene.left_balls << "/" <<  m_context.scene.right_balls <<  endl;
    }
}

//get one scene
//@returns true if one updated scene is available and retrieved, otherwise false.
bool Picker::get_stable_scene() {
    bool ret = m_vision->get_stable_scene(&m_context.scene);
    if (ret)
        consume_scene();
    return ret;
}

//@returns true if the ball is at least 61cm(2ft) away
bool Picker::is_far() {
    return (m_context.scene.total_balls > 0) && (m_context.scene.target_ball.y >= ((PIXEL_DISTANCE_FAR+PIXEL_DISTANCE_PICK_NEAR)/2));
}

//check if the target ball is still in the right position
//@param strict - true if the ball needs to be at a small angle
bool Picker::is_covered(bool strict) {
    if (m_context.scene.total_balls <= 0)
        return false;
    
    int abs_angle = abs(m_context.scene.target_ball.angle);
    int distance  = m_context.scene.target_ball.y;
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

//@return true if the ball is close enough and in allowed angle
bool Picker::is_ready_pickup() {
    if (m_context.scene.total_balls <= 0)
        return false;
    return (m_context.scene.target_ball.y <= PIXEL_DISTANCE_PICK_FAR) && is_covered(true);
}

//find one ball (turn the car around if needed) and adjust the car direction to make sure the ball is at 
//good positition so that it can be picked up later.
//@returns true if one ball is found
bool Picker::searching () {
    if (debug)
        cout << "@@@Searching" << endl;
    
    //done or paused?
    if (!get_stable_scene ())
        return false;
    
    analyse_scene();
    //there is one ball which is ready for picking up
    if (is_covered(true))
        return true;
    
    //step 1, rotate the car until we see a ball
    bool found = false;
    if (m_context.scene.total_balls <= 0) {
        int direction = choose_turning_direction();
        long till_ms = Utils::current_time_ms() + MAX_TURNING_360_MS;//for 360 degree
        m_motor->rotate_car_fast(direction);
        while (!found && (Utils::current_time_ms() < till_ms)) {
            Utils::delay_ms(m_config->get_frame_time_ms()>>1);
            if (!should_continue() || !get_stable_scene()) {
                break;
            }
            analyse_scene();
            //slow down once we see a ball
            if (m_context.scene.total_balls > 0) {
                found = true;
            }
        }
        //The car already turned one round, no ball found
        if (!found)
            return false;

        if (is_covered(true))
            return true;
    }

    //step 2, rotating the car slowly to target the ball
    found = false;
    {
        int direction = choose_turning_direction();
        //off track too much, move the car back first
        if (!is_covered(false) && !is_far()) {
            if (direction == TURNING_DIRECTION_RIGHT) {
                m_motor->turn_car_right_backward();
            }
            else if (direction == TURNING_DIRECTION_LEFT) {
                m_motor->turn_car_left_backward();
            }
            Utils::delay_ms(1000);
        }
        analyse_scene();
        m_motor->rotate_car_slow(direction);
        get_stable_scene();//update the scene, as the car just slowed down
        long till_ms = Utils::current_time_ms() + MAX_TURNING_360_MS;
        int last_angle = m_context.scene.target_ball.angle, this_angle = 0;
        while (!found && (Utils::current_time_ms() < till_ms)) {
            Utils::delay_ms(m_config->get_frame_time_ms()>>1);
            if (!should_continue() || !get_stable_scene())
                break;
            analyse_scene();
            if (m_context.scene.total_balls > 0) {
                this_angle = m_context.scene.target_ball.angle;
                if (is_covered(true) || (this_angle * last_angle < 0)) {//ball moved from one side to another side
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
    
    int failures = 0;
    int total_counter=0;
    int counter_left = 0, counter_right = 0;//how many times we saw the ball at left or right
    bool ready_to_pickup = false;
    //tracking the ball
    while (should_continue() && get_stable_scene()) {
        analyse_scene();
        if (m_context.scene.total_balls > 0) {
            if (is_ready_pickup()) {
                ready_to_pickup = true;
                break;
            }
            else if (is_covered(false)) {//still within allowed range
                int ball_angle = m_context.scene.target_ball.angle;
                int abs_angle = abs(ball_angle);
                if (abs_angle >= PERFECT_ANGLE) {//a little bit off the road
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
            else if (++failures > 2) {//the ball is off track too much, need fast rotation to target it
                break;
            }
        }
        else {
            //the car may move so quick
            ready_to_pickup = true; //just a guess
            break;
        }
    }
    return ready_to_pickup;
}

//search to find one ball and pick it up
void Picker::picking_up() {
    if (debug)
        cout << "@@@Picking up" << endl;
    
    //reseet hints
    reset_turning_hints();
    
    bool picked_one = false;
    int  wait_time = 0;
    if (searching()) {
        if (debug)
            cout << "**Found the ball to be picked up, angle " << m_context.scene.target_ball.angle << endl;
        m_motor->move_car_forward();
        if (tracking()) {
            if (debug)
                cout << "==============================>Ready to pickup" << endl;
            //move forward to pick it up
            m_motor->move_car_forward();
            int distance = m_context.scene.target_ball.y;
            if (get_stable_scene() && m_context.scene.total_balls > 0)
                distance = m_context.scene.target_ball.y;
            wait_time = (int)(distance / SPEED_PIXELS_PER_MS);
            if (wait_time <= 0)
                wait_time = 3500;
            if (should_continue()) {//the car is still moving
                if (debug)
                    cout << "==============================>picking up, wait for " << wait_time << "ms" << endl;
                Utils::delay_ms(wait_time);
                ++m_context.total_balls_collected;
                picked_one = true;
            }
        }
    }

    if (picked_one) {
        //Move back a little bit, get ready to pick up another ball
        long till_ms = Utils::current_time_ms() + wait_time;
        while (get_stable_scene() && Utils::current_time_ms() <= till_ms) {
            if (!should_continue() || (m_context.scene.total_balls > 1))
                break;
            m_motor->move_car_backward();
            Utils::delay_ms(m_config-> get_frame_time_ms());
        }
        Utils::delay_ms(1000);
    }
}

