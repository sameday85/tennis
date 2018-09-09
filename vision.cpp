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

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/photo.hpp"

#include <math.h>
#include <map>

#include "vision.h"
#include "utils.h"
#include "led.h"
#include "pins.h"

using namespace std;

/*
 * Implementation of the Vision class
 */
 
//the constructor with the configuration
Vision::Vision(Config *config) : Component(config) {
    camera_ready = false;
    scene_seq_consumed = 0;
    memset (&m_scene, 0, sizeof (Scene));
    the_thread = 0;
}

Vision::~Vision() {
}

//override, initialize the camera
bool Vision::init () {
    Component::init();
    
    m_camera = new raspicam::RaspiCam_Cv();
    m_camera->set( CV_CAP_PROP_FORMAT, CV_8UC3 );
    m_camera->set( CV_CAP_PROP_FRAME_WIDTH,  CAMERA_FRAME_WIDTH );
    m_camera->set( CV_CAP_PROP_FRAME_HEIGHT, CAMERA_FRAME_HEIGHT );
    m_camera->set( CV_CAP_PROP_WHITE_BALANCE_RED_V, 0);//0-100
    m_camera->set( CV_CAP_PROP_FPS, m_config->get_frames_per_second());

    camera_ready = m_camera->open();
    return camera_ready;
}

//override
void Vision::start() {
    Component::start();
    if (!camera_ready)
        return;
    pthread_create(&the_thread, NULL, sensor, this);    
}


void Vision::stop() {
    Component::stop();
    if (the_thread)
        pthread_join(the_thread, NULL);
    m_camera->release();
    delete m_camera;
}

//waiting for a scene to be available and copy it to the input argument
//@param output- pointer to the output secene data
//@returns true if a scene is available and retrieved successfully
bool Vision::get_stable_scene(Scene *output) {
    while (scene_seq_consumed == m_scene.seq && !is_paused()) {
        Utils::delay_ms(m_config->get_frame_time_ms() >> 1);
    }
    if (scene_seq_consumed == m_scene.seq) {
        return false;
    }
    memcpy (output, &m_scene, sizeof (Scene));
    scene_seq_consumed = output->seq;
    return true;
}


//capturing frames and analyse each frame to recognize balls, find the nearest ball and get its distance and angle. other information such as
//balls at the left of the nearest ball, balls at the right of the nearest ball are also available.
void* Vision::sensor(void *arg) {
    Vision *the_vision = (Vision*)arg;
    bool verbose = false;
    
    raspicam::RaspiCam_Cv *p_camera = the_vision->m_camera;
    Scene *p_scene = &the_vision->m_scene;
    RobotConfig *active_config= the_vision->get_config()->get_active_config();
    long frame_time_ms = the_vision->get_config()->get_frame_time_ms(); //convert to long
    
    cv::Mat frame;
    long frame_start;
    
    //erosion & dilation sizes will not change
    cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size( 2*active_config->erosion_size + 1, 2*active_config->erosion_size+1 ), cv::Point(active_config->erosion_size, active_config->erosion_size));
    cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size( 2*active_config->dilation_size + 1, 2*active_config->dilation_size+1 ),cv::Point(active_config->dilation_size, active_config->dilation_size));
    while (!the_vision->is_done()) {
        frame_start = Utils::current_time_ms();
        if (!the_vision->is_paused()) {
            //this will take abount 17ms
            p_camera->grab();
            p_camera->retrieve (frame);

            //this will take about 25ms
            cv::Mat hsv;
            cv::cvtColor (frame, hsv, CV_BGR2HSV);

            //this will take about 36ms
             //gray color
            cv::Mat mask = cv::Mat(frame.rows, frame.cols, CV_8UC1);
            cv::inRange(hsv, cv::Scalar(active_config->minH, active_config->minS, active_config->minV), cv::Scalar(active_config->maxH, active_config->maxS, active_config->maxV), mask);
            if (verbose)
                cv::imwrite("step1.jpg",mask);    

            //this will take about 72ms
            //https://docs.opencv.org/3.4.2/db/df6/tutorial_erosion_dilatation.html
            cv::erode(mask, mask, element1);  
            if (verbose)
                cv::imwrite("step2.jpg",mask);       
            
            //this will take about 88ms(200ms if the kernel size is 10!)
            cv::dilate( mask, mask, element2);
            if (verbose)
                cv::imwrite("step3.jpg",mask);            
            
            //find contours. this will take about 36ms
            vector<vector<cv::Point> > contours;
            vector<cv::Vec4i> hierarchy;
            cv::Mat canny_output;

            cv::Canny(mask, canny_output, active_config->canny_thresh, active_config->canny_thresh*2, 3 ); /// Detect edges using canny
            cv::findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );/// Find contours

            int half_width = frame.cols >> 1;
            int total = 0;
            long min_distance = 0;
            int ball_angle = 180, ball_distance_y = 0, center_x = 0;
            vector<cv::Point> all_positions;
            for( size_t i = 0; i < contours.size(); i++ ) {
                int area = cv::contourArea(contours[i]);
                if (area < active_config->min_area)
                    continue;

                cv::Moments mnt =  cv::moments( contours[i], false );
                cv::Point center(mnt.m10/mnt.m00 , mnt.m01/mnt.m00);
                
                int diff_x = center.x - half_width;
                int diff_y = frame.rows - center.y;
                if (diff_y <= 0)
                    continue;
                    
                all_positions.push_back(center);
                long distance = (long)diff_x * diff_x + (long)diff_y * diff_y;
                int angle = (int)(atan(1.0 * diff_x / diff_y) * 180.0 / 3.14);
                if (total++ <= 0 || (distance < min_distance)) {
                    center_x = center.x;
                    min_distance = distance;
                    ball_distance_y = diff_y;
                    ball_angle = angle;
                }
                if (verbose) {
                    cv::Scalar color = cv::Scalar( i * 40, i*20, 0 );
                    cv::drawContours( frame, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
                    cv::circle( frame, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );// draw the circle 
                    cv::imwrite("balls.jpg", frame);
                }
            }
            int balls_at_left = 0, balls_at_right = 0;
            int nearest_ball_at_left = 0, nearest_ball_at_right = 0;
            for (size_t i = 0; i < all_positions.size(); ++i) {
                cv::Point pt = all_positions.at(i);
                if (pt.x > center_x) {
                    ++balls_at_right;
                    if (pt.y > nearest_ball_at_right)
                        nearest_ball_at_right = pt.y;
                }
                else if (pt.x < center_x) {
                    ++balls_at_left;
                    if (pt.y > nearest_ball_at_left)
                        nearest_ball_at_left = pt.y;
                }
            }
            if (the_vision->debug && (total > 0)) {
                cout << "#" << (p_scene->seq+1) << ": " << total << ",target ball angle " << ball_angle << ",dist " << ball_distance_y  << ",balls(L/R) " <<  balls_at_left << "/" << balls_at_right << ",time=" << (Utils::current_time_ms() - frame_start) << endl;
            }
            p_scene->balls = total;
            p_scene->angle = ball_angle;
            p_scene->distance=ball_distance_y;
            p_scene->balls_at_left=balls_at_left;
            p_scene->nearest_ball_at_left=frame.rows - nearest_ball_at_left;
            p_scene->balls_at_right=balls_at_right;
            p_scene->nearest_ball_at_right=frame.rows - nearest_ball_at_right;
            p_scene->seq++;

            //save image 
            if (verbose) {
                char szFileName[255];
                sprintf(szFileName, "frame%03ld.jpg", p_scene->seq);
                cv::imwrite(szFileName, frame);
            }
        }//paused
        long left =  frame_time_ms - (Utils::current_time_ms() - frame_start);
        if (left > 0)
            Utils::delay_ms(left);
    }
    
    return arg;
}
 

//recognize one ball and get its hsv color range. the idea is to take a background picture first(the led is stead on, frame 1), 
//then put one ball in the background (the led is flashing) and get another picture(after the led stop flashing, frame 2). 
//substract frame1 from frame 2 to get the ball picture only with all other areas as black. pixels will then be retrieved
//from the ball picture(a circle) and min/max hsv values will be calculated and saved to the configuration file.
bool Vision::calibrate() {
    Led::turn_on_red_led(); //as we are going back to pause state
        
    int using_pin = PIN_LED_GREEN;
    Led::turn_on_led(using_pin);
    Utils::delay_ms(2000);
    
    //take the background image
    cv::Mat bg;
    for (int i = 0; i < 30; ++i) {
        m_camera->grab();
        m_camera->retrieve (bg);
    }
    if (debug)
        cv::imwrite("background1.jpg",bg);
    
    int trying = 0;
    cv::Mat frame;
    bool found = false, notified = false;
    while (!found && (trying <= 10) && !is_done()) { //at most ten seconds
        //flashing the led
        ++trying;
        Utils::delay_ms(1000);
        if (trying <= 5) {  //time for moving hands away
            if ((trying & 1) == 0) {
                Led::turn_off_led(using_pin);
            }
            else {
                Led::turn_on_led(using_pin);
            }
            continue;
        }
        else if (!notified) {
            Led::turn_on_led(using_pin); //stays on
            Led::buzzle(false);
            notified = true;
        }

        m_camera->grab();
        m_camera->retrieve (frame);
        if (debug)
            cv::imwrite("frame.jpg",frame);
        
        cv::Mat difference;
        cv::subtract(frame, bg, difference);
        if (debug)
            cv::imwrite("difference1.jpg",difference);
        
        cv::Mat gray;
        cv::cvtColor (difference, gray, CV_BGR2GRAY);
        
        //cv::fastNlMeansDenoising(gray, gray);
        cv::GaussianBlur( gray, gray, cv::Size(9, 9), 2, 2 );
        
        if (debug)
            cv::imwrite("difference2.jpg",gray);
        
        vector<cv::Vec3f> circles;
        /*
         * param2: was 8
         * minRadius: 4
         * maxRadius: rows / 4
         */
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 200, 50, 20, 4, frame.rows / 4 );
        //do not draw on the frame yet
        if (circles.size() == 1) {//just found the ball
            cv::Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
            float radius = cvRound(circles[0][2]) - 4;

            cv::Mat hsv;
            cv::cvtColor (frame, hsv, CV_BGR2HSV);
            int _minH=0, _minS=0, _minV=0, _maxH=0, _maxS=0, _maxV=0;
            map<int, int> statistics;
            for (int i = 0; i<frame.rows; i++)  {
                for (int j = 0; j<frame.cols; j++) {
                    float diffX = j - center.x;
                    float diffY = i - center.y;
                    float distance = sqrt (diffX * diffX + diffY * diffY);
                    if (distance < radius) {
                        cv::Vec3b hsvValue = hsv.at<cv::Vec3b>(i, j);
                        int H = hsvValue.val[0]; //hue
                        int S = hsvValue.val[1]; //saturation
                        int V = hsvValue.val[2]; //value
                        
                        int key = (H << 16) | (S << 8) | V;
                        int total = (statistics.count(key) == 1) ? statistics[key] : 0;
                        statistics[key]=total + 1;
                    }
                }
            }
            vector<int> sorted_values;
            map<int, int>::iterator it1 = statistics.begin();
            while(it1 != statistics.end()) {
                //int key = it1->first;
                int value = it1->second;
                sorted_values.push_back(value);
                it1++;
            }
            sort(sorted_values.begin(), sorted_values.end());
            int min_value = sorted_values[sorted_values.size() / 3];//discard 1/3 pixels
            
            map<int, int>::iterator it2 = statistics.begin();
            bool first = true;
            while (it2 != statistics.end()) {
                int key = it2->first;
                int value = it2->second;
                int H = (key >> 16) & 0xff;
                int S = (key >> 8) & 0xff;
                int V = key & 0xff;
                if (value >= min_value) {
                    if (first) {
                        _minH = _maxH = H;
                        _minS = _maxS = S;
                        _minV = _maxV = V;
                        first = false;
                    }
                    else {
                        if (H < _minH)
                            _minH = H;
                        else if (H > _maxH) {
                            _maxH = H;
                        }
                        if (S < _minS)
                            _minS = S;
                        else if (S > _maxS) {
                            _maxS = S;
                        }
                        if (V < _minV)
                            _minV = V;
                        else if (V > _maxV) {
                            _maxV = V;
                        }
                    }
                }
                it2++;
            }
            if (debug)
                cout << "H=" << _minH << "-" << _maxH << ",S=" << _minS << "-" << _maxS << ",V=" << _minV << "-" << _maxV << endl;
            m_config->get_active_config()->minH=_minH;
            m_config->get_active_config()->maxH=_maxH;
            m_config->get_active_config()->minS=_minS;
            m_config->get_active_config()->maxS=_maxS;
            m_config->get_active_config()->minV=_minV;
            m_config->get_active_config()->maxV=_maxV;
            found = true;
        }
        if (debug) {
            cout << "Total circles: " << circles.size() << endl;
            for( size_t i = 0; i < circles.size(); i++ ) {
                cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);
                // draw the circle center
                cv::circle( frame, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
                // draw the circle outline
                cv::circle( frame, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
            }
            //save image 
            cv::imwrite("background2.jpg",frame);
        }
    }
    Led::turn_off_led(using_pin);
    if (debug) {
        cout << "Done with calibration" << endl;
    }
    Led::buzzle(!found);
    return found;
}
