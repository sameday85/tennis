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
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <sys/time.h>

#include <raspicam/raspicam_cv.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/photo.hpp"

#include <wiringPi.h>
#include <softPwm.h>
#include <pthread.h>
#include <mutex> 
#include <math.h>
#include <map>

#define CONFIG_FILE			"/home/pi/.tennis.ini"
//raspistill outputs 3280x2464
#define CAMERA_FRAME_WIDTH	(320*3)
#define CAMERA_FRAME_HEIGHT (240*3)
//Motor control
#define MOTORS_LEFT			0
#define MOTORS_RIGHT		1
#define MOTOR_COLLECTOR		4
//directions
#define DIR_FORWARD			0
#define DIR_BACKWARD		1
//pins for motors
#define PIN_LMOTORS_1		4
#define PIN_LMOTORS_2		1
#define PIN_RMOTORS_1		6
#define PIN_RMOTORS_2		5
#define PIN_COLLECTOR_1		26
#define PIN_COLLECTOR_2		27

#define PIN_PWM_LEFT		10
#define PIN_PWM_RIGHT		11
#define PWM_MAX				100

#define MOVING_SPEED		12 //was 15
#define TURNING_SPEED_MIN   10
#define TURNING_SPEED_MAX   30
#define ROTATING_SPEED		40 //was 42

//ultra sound sensor
#define PIN_TRIG_FRONT		15
#define PIN_ECHO_FRONT		16
 
#define PIN_TRIG_REAR		28
#define PIN_ECHO_REAR		29

#define PIN_BTN1			0
#define PIN_BTN2            12

#define PIN_LED_RED			2
#define PIN_LED_GREEN		13
#define PIN_LED_BLUE		14

#define PIN_BUZZLE			3

//time limitation for turning 90 degree
#define MAX_TURNING_90_MS		18000

#define WAIT_BALL_OUT_OF_SCENE_MS	1500 //after the ball is out of scene, wait for this time, then start the collector
#define WAIT_BALL_PICKUP_MS	        2000 //the time collector is running
#define BACK_AFTER_PICKUP_MS		2000
#define FRONT_DISTANCE_DANGEROUS	20 //cm
#define REAR_DISTANCE_DANGEROUS		10 //cm

#define PIXEL_DISTANCE_PICK_FAR		180 //far point, at which the ball can be picked up
#define PICKUP_ANGLE_FAR			45  //was 45
#define PIXEL_DISTANCE_PICK_NEAR	120 //near point, at which the ball can be picked up
#define PICKUP_ANGLE_NEAR			65  //

#define PERFECT_ANGLE			6
#define GOOD_ANGLE				30 //if the ball is far away

//the car is picking up balls
#define STATE_STARTED			0
#define STATE_INIT_PICKING		1
#define STATE_PICKING_UP		2
//interruptions
#define INT_NONE				0
#define INT_FRONT_OBSTACLE		1

#define INT_REAR_OBSTACLE		2
#define INT_NO_MORE_BALLS		3
//car movement states
#define CAR_STATE_STOPPED					0
#define CAR_STATE_MOVING_FORWARD			1 //need to check for obstacle if the state is an odd number
#define CAR_STATE_MOVING_BACKWARD			2
#define CAR_STATE_TURNING_LEFT_FORWARD		3 //two motors running in full speed, other two running in half speed, car is moving forward
#define CAR_STATE_TURNING_RIGHT_FORWARD		5
#define CAR_STATE_TURNING_LEFT_BACKWARD		4 //two motors running in full speed, other two running in half speed, car is moving forward
#define CAR_STATE_TURNING_RIGHT_BACKWARD	6
#define CAR_STATE_ROTATING_LEFT				21
#define CAR_STATE_ROTATING_RIGHT			23

#define COLLECTOR_STATE_STOPPED			0
#define COLLECTOR_STATE_RUNNING			1

#define TURNING_DIRECTION_UNKNOWN			0
#define TURNING_DIRECTION_CLOCKWISE			1
#define TURNING_DIRECTION_COUNTERCLOCKWISE	2

//user actions
#define UA_NONE				0
#define UA_PAUSE			1
#define UA_CALIBRATE		2
#define UA_TEST_MOTOR		3
#define UA_TEST_COLLECTOR	4
#define UA_TEST_CAMERA		5
#define UA_TEST_SELF		6
#define UA_WAITING			9
#define UA_DEBUG			10
#define UA_INC_SPEED		11
#define UA_DEC_SPEED	    12
#define UA_PRE_CALIBRATE	13
#define UA_DONE				100

#define VENUE_INDOOR		0 //for debugging
#define VENUE_OUTDOOR		1 //tennis court

//two buttons
#define BTN_01              1
#define BTN_02              2

using namespace std; 

int g_user_action;
bool g_is_led_on;
bool debug;

int speed_base;
//the camera is not facing front straightly, add this value the calculated ball's angle
int camera_deviation; 
//inRange
int minH, maxH, minS, maxS, minV, maxV;
//Blur
int erosion_size, dilation_size;
//Canny
int canny_thresh;
int min_area; //min area of the contour
//
int frames_per_second, frame_time_ms;
int scene_seq_consumed;

//											                   0
typedef struct _Scene {                //                      |
	int balls; //total number of balls                         |
	int angle; //The angle of the nearest ball 270(-90) -------o---------90
	int distance; //the nearest ball distance in pixels
	int balls_at_left, balls_at_right; //balls at the left or right sections in the scene
	int nearest_ball_at_left, nearest_ball_at_right;
    int min_angle;//the ball close to the middle of the scene
	unsigned long seq; //sequence number

	long front_obstacle, rear_obstacle; //distance in cm
} Scene;

typedef struct _RobotCtx {
	Scene scene;
	int venue;	//see macros VENUE_XXX
	int  state; //see macros STATE_XXX
	int  interruption; //see macros INT_XXX
	
	Scene last_scene_w_balls;
	int balls_collected;
	int consecutive_collected;
	int last_turn_direction;
} RobotCtx;

Scene g_scene; //the current scene
RobotCtx g_context;
int g_car_state, g_collector_state;
bool g_turning_360;

raspicam::RaspiCam_Cv Camera;

//the low byte is the button number (BTN_1 or BTN_2)
//high byte is reserved
vector<int> btn_events;
std::mutex mtx_btn_event; //mutex for accessing above button event vector

int abs(int value) {
	if (value < 0)
		value = 0 - value;
	return value;
}

void delay_ms(int x) {
	usleep(x * 1000);
}
//current system time in milliseconds
long current_time_ms() {
	struct timeval start;
	gettimeofday(&start, NULL);

	return (long)(start.tv_usec / 1000 + start.tv_sec * 1000);
}

//stop the program when ctr+c is received
void handle_signal(int signal) {
    // Find out which signal we're handling
    switch (signal) {
		case SIGINT://key 3
			g_user_action=UA_DONE;
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

//load settings from the configuration file and initialize other global settings
//the configuration file is located at ~/.tennis.ini, see the macro CONFIG_FILE
//right now only the tennis ball hsv color range are configurable from the configuraion file.
void load_config() {
	debug = true;
	g_user_action=UA_NONE;
	speed_base = 0;
	camera_deviation=0;
	//default values
	frames_per_second = 10;
	
	minH = 60;  maxH = 90; //0-180
	minS = 170; maxS = 255; //0-255
	minV = 70;  maxV = 155; //0-255

	erosion_size = 3;
	dilation_size= 10;
	
	canny_thresh = 100;
	min_area = 4;
	
	frame_time_ms = 1000 / frames_per_second;
	scene_seq_consumed = 0;

	map<string, string> ans;
    ifstream input(CONFIG_FILE, ifstream::in); //The input stream
    string line;
    while (input) {
        getline(input, line, '\n');
        if (line.find_first_of("#") == 0)
			continue;
        string::size_type pos = line.find_first_of("=");
        if (pos != string::npos) {
			string key = line.substr(0, pos);
            string value = line.substr(pos+1);
            ans[key]=value;
        }
    }
    input.close();
    
    if (ans.size() <= 0)
		return;
		
	//inRange color filter
	if (ans.count("minH") == 1) {
		minH = stoi(ans["minH"]);
	}
	if (ans.count("maxH") == 1) {
		maxH = stoi(ans["maxH"]);
	}
	if (ans.count("minS") == 1) {
		minS = stoi(ans["minS"]);
	}
	if (ans.count("maxS") == 1) {
		maxS = stoi(ans["maxS"]);
	}
	if (ans.count("minV") == 1) {
		minV = stoi(ans["minV"]);
	}
	if (ans.count("maxV") == 1) {
		maxV = stoi(ans["maxV"]);
	}
}	

//save the tennis ball color range to the configuration file
void save_config() {
	std::ofstream out(CONFIG_FILE);
    out << "[OpenCV]" << endl;
    out << "minH=" << minH << endl;
    out << "maxH=" << maxH << endl;
    out << "minS=" << minS << endl;
    out << "maxS=" << maxS << endl;
    out << "minV=" << minV << endl;
    out << "maxV=" << maxV << endl;
    out.close();
}

//start the given motor. The motor may be one of the robot car motors or the collector (which is a heavy duty motor).
//@param motor - the motor number
//@param dir - the direction: forward or backward
void start_motor(int motor, int dir) {
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
void stop_motor(int motor) {
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

//set the motor speed for the given car state
void set_speed(int desired_state) {
	switch (desired_state) {
		case CAR_STATE_MOVING_FORWARD:
		case CAR_STATE_MOVING_BACKWARD:
			softPwmWrite (PIN_PWM_LEFT,  MOVING_SPEED+speed_base);
			softPwmWrite (PIN_PWM_RIGHT, MOVING_SPEED+speed_base);
			break;
		case CAR_STATE_TURNING_LEFT_FORWARD:
		case CAR_STATE_TURNING_RIGHT_BACKWARD: //car header direction change
			softPwmWrite (PIN_PWM_LEFT,  TURNING_SPEED_MIN+speed_base); 
			softPwmWrite (PIN_PWM_RIGHT, TURNING_SPEED_MAX+speed_base);
			break;
		case CAR_STATE_TURNING_RIGHT_FORWARD:
		case CAR_STATE_TURNING_LEFT_BACKWARD: //car header direction change
			softPwmWrite (PIN_PWM_LEFT,  TURNING_SPEED_MAX+speed_base);
			softPwmWrite (PIN_PWM_RIGHT, TURNING_SPEED_MIN+speed_base);
			break;
		case CAR_STATE_ROTATING_LEFT:
		case CAR_STATE_ROTATING_RIGHT:
			softPwmWrite (PIN_PWM_LEFT,  ROTATING_SPEED+speed_base);
			softPwmWrite (PIN_PWM_RIGHT, ROTATING_SPEED+speed_base);
			break;
	}
}

//set the motor speeds and start motors to  move forward
void move_car_forward() {
	if (g_car_state==CAR_STATE_MOVING_FORWARD)
		return;
	if (debug)
		cout << "%%%%%Moving forward" << endl;
	set_speed(CAR_STATE_MOVING_FORWARD);
	start_motor(MOTORS_LEFT, DIR_FORWARD);
	start_motor(MOTORS_RIGHT,DIR_FORWARD);
	g_car_state=CAR_STATE_MOVING_FORWARD;
}

//set the motor speeds and start motors to  move backward
void move_car_backward(){
	if (g_car_state==CAR_STATE_MOVING_BACKWARD)
		return;	
	if (debug)
		cout << "%%%%%Moving backward" << endl;
	set_speed(CAR_STATE_MOVING_BACKWARD);
	start_motor(MOTORS_LEFT, DIR_BACKWARD);
	start_motor(MOTORS_RIGHT,DIR_BACKWARD);
	g_car_state=CAR_STATE_MOVING_BACKWARD;
}

//set the motor speeds and turn the car to the left
//in this case right motors will be a little bit fast than left motors
void turn_car_left_forward() {
	if (g_car_state==CAR_STATE_TURNING_LEFT_FORWARD)
		return;
	if (debug)
		cout << "%%%%%Turning left forward" << endl;
	set_speed(CAR_STATE_TURNING_LEFT_FORWARD); 
	start_motor(MOTORS_LEFT, DIR_FORWARD);
	start_motor(MOTORS_RIGHT,DIR_FORWARD);
	g_car_state=CAR_STATE_TURNING_LEFT_FORWARD;
}

//set the motor speeds and turn the car to the right
//in this case left motors will be a little bit fast than right motors
void turn_car_right_forward() {
	if (g_car_state==CAR_STATE_TURNING_RIGHT_FORWARD)
		return;
	if (debug)
		cout << "%%%%%Turning right forward" << endl;
	set_speed(CAR_STATE_TURNING_RIGHT_FORWARD);
	start_motor(MOTORS_LEFT, DIR_FORWARD);
	start_motor(MOTORS_RIGHT,DIR_FORWARD);
	g_car_state=CAR_STATE_TURNING_RIGHT_FORWARD;
}

//set the motor speeds and move the car backward. the car will turn a little bit to the left
void turn_car_left_backward() {
	if (g_car_state==CAR_STATE_TURNING_LEFT_BACKWARD)
		return;
	if (debug)
		cout << "%%%%%Turning left backward" << endl;
	set_speed(CAR_STATE_TURNING_LEFT_BACKWARD); 
	start_motor(MOTORS_LEFT, DIR_BACKWARD);
	start_motor(MOTORS_RIGHT,DIR_BACKWARD);
	g_car_state=CAR_STATE_TURNING_LEFT_BACKWARD;
}

//set the motor speeds and move the car backward. the car will turn a little bit to the right
void turn_car_right_backward() {
	if (g_car_state==CAR_STATE_TURNING_RIGHT_BACKWARD)
		return;
	if (debug)
		cout << "%%%%%Turning right backward" << endl;
	set_speed(CAR_STATE_TURNING_RIGHT_BACKWARD);
	start_motor(MOTORS_LEFT, DIR_BACKWARD);
	start_motor(MOTORS_RIGHT,DIR_BACKWARD);
	g_car_state=CAR_STATE_TURNING_RIGHT_BACKWARD;
}

//turn the car to the left at its current position
void rotate_car_left() {
	if (g_car_state==CAR_STATE_ROTATING_LEFT)
		return;
	if (debug)
		cout << "%%%%%Rotating left" << endl;
	set_speed(CAR_STATE_ROTATING_LEFT);
	start_motor(MOTORS_LEFT, DIR_BACKWARD);
	start_motor(MOTORS_RIGHT,DIR_FORWARD);
	g_car_state=CAR_STATE_ROTATING_LEFT;
}

//turn the car to the right at its current position
void rotate_car_right() {
	if (g_car_state==CAR_STATE_ROTATING_RIGHT)
		return;
	if (debug)
		cout << "%%%%%Rotating right" << endl;
	set_speed(CAR_STATE_ROTATING_RIGHT);
	start_motor(MOTORS_LEFT, DIR_FORWARD);
	start_motor(MOTORS_RIGHT,DIR_BACKWARD);
	g_car_state=CAR_STATE_ROTATING_RIGHT;
}

//rotate the car clockwise or counterclosewise
//@param direction: TURNING_DIRECTION_CLOCKWISE or TURNING_DIRECTION_COUNTERCLOCKWISE
void rotate_car(int direction) {
	if (direction == TURNING_DIRECTION_CLOCKWISE)
		rotate_car_right();
	else if (direction == TURNING_DIRECTION_COUNTERCLOCKWISE)
		rotate_car_left();
}

//stop all car motors
void stop_car() {
	if (g_car_state==CAR_STATE_STOPPED)
		return;
	if (debug)
		cout << "%%%%%Stopped" << endl;
	stop_motor(MOTORS_LEFT);
	stop_motor(MOTORS_RIGHT);
	g_car_state=CAR_STATE_STOPPED;
}

//turning the car slow by moving the car back and forth with one side motors running a little bit fast and other side motor running a little bit slow. 
//will stop moving once the global variable g_turning_360 is set to false
//this function will be running in a background.
void* _turn_car_360(void *arg) {
	g_turning_360 = true;
	int direction =*((int*)arg);
	bool clockwise = (direction == TURNING_DIRECTION_CLOCKWISE);
	int desired_car_states[2];
	if (clockwise) {
		desired_car_states[0]=CAR_STATE_TURNING_RIGHT_FORWARD;
		desired_car_states[1]=CAR_STATE_TURNING_RIGHT_BACKWARD;
	}
	else {
		desired_car_states[0]=CAR_STATE_TURNING_LEFT_FORWARD;
		desired_car_states[1]=CAR_STATE_TURNING_LEFT_BACKWARD;
	}
	int duration_ms[2], step=500;
	duration_ms[0]=1500; //forward
	duration_ms[1]=1000; //backward
	for (int i = 0; i < 20 && g_turning_360; ++i) {
		switch (desired_car_states[i & 1]) {
			case CAR_STATE_TURNING_LEFT_FORWARD:
			turn_car_left_forward();
			break;
			case CAR_STATE_TURNING_LEFT_BACKWARD:
			turn_car_left_backward();
			break;
			case CAR_STATE_TURNING_RIGHT_FORWARD:
			turn_car_right_forward();
			break;
			case CAR_STATE_TURNING_RIGHT_BACKWARD:
			turn_car_right_backward();
			break;
		}
		int state_ms = duration_ms[i&1];
		int car_state= g_car_state;
		for (int j = 0; (j < state_ms / step) && g_turning_360; ++j) {
			delay_ms(step);
		}
	}
	if (debug)
		cout << "Exited from turning car thread ...." << endl;
}

//turn the car slowly. this will be done by starting a new background thread to run the above _turn_car_360 function
//@param _direction: closewise or anticlockwise
void turn_car_360(int _direction) {
	static int direction;
	direction = _direction;
	pthread_t thread_turning_car;
	pthread_create(&thread_turning_car, NULL, _turn_car_360, &direction);
}

//determine what direction we should turn the car
//@param recovering - true if we just lost tracking a ball and wanted to get it back
int choose_turning_driection(RobotCtx *ctx, bool recovering) {
	int direction = TURNING_DIRECTION_COUNTERCLOCKWISE;
	if (recovering) {
		//we lost the ball, try to get it back
		if (ctx->last_scene_w_balls.angle <= 0) {
			if (debug)
				cout << "Decision 1" << endl;
		}
		else {
			if (debug)
				cout << "Decision 2" << endl;
			direction = TURNING_DIRECTION_CLOCKWISE;
		}
	}
	else {
		//after we picked up one ball
		if (ctx->last_scene_w_balls.balls_at_left > 0 &&  ctx->last_scene_w_balls.balls_at_right > 0) {
			if (ctx->last_scene_w_balls.nearest_ball_at_left < ctx->last_scene_w_balls.nearest_ball_at_right) {
				if (debug)
					cout << "Decision 3" << endl;
				direction = TURNING_DIRECTION_CLOCKWISE;
			}
			else if (ctx->last_scene_w_balls.nearest_ball_at_left > ctx->last_scene_w_balls.nearest_ball_at_right) {
				if (debug)
					cout << "Decision 4" << endl;
			}
			else {
				if (debug)
					cout << "Decision 5" << endl;
				direction=ctx->last_turn_direction;
			}
		}
		else if (ctx->last_scene_w_balls.balls_at_left > 0) {
			if (debug)
				cout << "Decision 6" << endl;
		}
		else if (ctx->last_scene_w_balls.balls_at_right > 0) {
			if (debug)
				cout << "Decision 7" << endl;
			direction = TURNING_DIRECTION_CLOCKWISE;
		}
		else if (ctx->last_scene_w_balls.angle <= 0) {
			if (debug)
				cout << "Decision 8, no known balls left" << endl;
		}
		else {
			if (debug)
				cout << "Decision 9, no known balls left" << endl;
			direction=ctx->last_turn_direction;
		}
	}
	if (direction == TURNING_DIRECTION_UNKNOWN)
		direction = TURNING_DIRECTION_CLOCKWISE;
	return direction;
}
//workaround the front or rear obstacle. for rear obstacle do nothing as the car alredy stopped.
void workaround_obstacle(RobotCtx *ctx) {
	if (ctx->interruption == INT_FRONT_OBSTACLE) {
		move_car_backward();
		delay_ms(2000);
		if (g_user_action == UA_DONE)
			return;
		int direction = choose_turning_driection(ctx, false);
		ctx->last_turn_direction = direction;
		rotate_car(direction);
		delay_ms(1000);
	}
	else if (ctx->interruption == INT_REAR_OBSTACLE) {
		//car already stopped, do nothing here
	}
}
//turn the motor on
void start_collector() {
	if (g_collector_state==COLLECTOR_STATE_RUNNING)
		return;
	start_motor(MOTOR_COLLECTOR, DIR_FORWARD);
	g_collector_state=COLLECTOR_STATE_RUNNING;
}
//turn the motor off
void stop_collector() {
	if (g_collector_state==COLLECTOR_STATE_STOPPED)
		return;
	stop_motor(MOTOR_COLLECTOR);
	g_collector_state=COLLECTOR_STATE_STOPPED;
}

void turn_on_led(int pin) {
    digitalWrite(pin, HIGH);
}

void turn_off_led(int pin) {
    digitalWrite(pin, LOW);
}

void turn_on_red_led(){
	if (g_is_led_on)
		return;
	digitalWrite(PIN_LED_RED, HIGH);
	g_is_led_on=true;
}

void turn_off_red_led() {
	if (!g_is_led_on)
		return;
	digitalWrite(PIN_LED_RED, LOW);
	g_is_led_on=false;
}
//make a long or shot buzzle
void buzzle (bool long_time) {
	digitalWrite(PIN_BUZZLE, HIGH);
	delay_ms(long_time ? 2000 : 300);
	digitalWrite(PIN_BUZZLE, LOW);
}
//perform self test. this will test the robot car motors, collector, led  and buzzle.
//it is better to lift the car up when run this test.
void self_test() {
	buzzle (true);
	turn_on_red_led();
	delay_ms(1000);
	turn_off_red_led();
	turn_on_led(PIN_LED_GREEN);
	delay_ms(1000);
	turn_off_led(PIN_LED_GREEN);
	turn_on_led(PIN_LED_BLUE);
	delay_ms(1000);
	turn_off_led(PIN_LED_BLUE);
	
	buzzle (false);
	
	move_car_forward();
	delay_ms(2000);
	stop_car();
	delay_ms(1000);
	if (g_user_action == UA_DONE)
		return;

	move_car_backward();
	delay_ms(2000);
	stop_car();
	delay_ms(1000);
	if (g_user_action == UA_DONE)
		return;

	turn_car_left_forward();
	delay_ms(1000);
	stop_car();
	delay_ms(1000);
	if (g_user_action == UA_DONE)
		return;
	
	turn_car_right_forward();
	delay_ms(1000);
	stop_car();
	if (g_user_action == UA_DONE)
		return;


	delay_ms(1000);
	start_collector();
	delay_ms(1000);
	stop_collector();
}
//recognize one ball and get its hsv color range. the idea is to take a background picture first(the led is stead on, frame 1), 
//then put one ball in the background (the led is flashing) and get another picture(after the led stop flashing, frame 2). 
//substract frame1 from frame 2 to get the ball picture only with all other areas as black. pixels will then be retrieved
//from the ball picture(a cycle) and min/max hsv values will be calculated and saved to the configuration file.
bool calibrate() {
	turn_on_red_led();
	delay_ms(2000);
    buzzle(false);
    
	//take the background image
	cv::Mat bg;
	for (int i = 0; i < 30; ++i) {
		Camera.grab();
		Camera.retrieve (bg);
	}
	if (debug)
		cv::imwrite("background1.jpg",bg);
	
	int trying = 0;
	cv::Mat frame;
	bool found = false;
	while (!found && (trying <= 10) && (g_user_action != UA_DONE)) { //at most ten seconds
		//flashing the led
		++trying;
		delay_ms(1000);
		if (trying <= 5) {  //time for moving hands away
			if ((trying & 1) == 0) {
				turn_off_red_led();
			}
			else {
				turn_on_red_led();
			}
			continue;
		}
		else {
			turn_on_red_led(); //stays on
		}

		Camera.grab();
		Camera.retrieve (frame);
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
			int _minH, _minS, _minV, _maxH, _maxS, _maxV;
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
				int key = it1->first;
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
			minH=_minH;
			maxH=_maxH;
			minS=_minS;
			maxS=_maxS;
			minV=_minV;
			maxV=_maxV;
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
	turn_on_red_led(); //as we are going back to pause state
	if (debug) {
		cout << "Done with calibration" << endl;
	}
	buzzle(!found);
	return found;
}
//measure the front or rear obstacle distance in cm
long measure_distance(int pin_trig, int pin_echo)
{
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

//distance in cm
long measure_front_distance(void)
{
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

//distance in cm
long measure_rear_distance(void) {
	return measure_distance(PIN_TRIG_REAR, PIN_ECHO_REAR);
}

//Process button events, running in a background thread
//button one is for changing the value
//button two is for selecting options
//options:
//red on    - pause/resume
//green on  - inc speed
//blue on   - dec speed
//green and blue on - calibrate
void* btn_event_handle(void *arg) {
    int wait_ms=1000;
    while (g_user_action != UA_DONE) {
        delay_ms(wait_ms);
        int event = 0;
        mtx_btn_event.lock();
        if (btn_events.size() > 0) {
            event = btn_events.at(0);
            btn_events.erase(btn_events.begin());
        }
        mtx_btn_event.unlock();
        if (event != 0) {
            if (debug)
                cout << "Btn event " << event <<", g state=" << g_user_action << endl;
            if ((event & 0xff) == BTN_01) {
                switch (g_user_action) {
                    case UA_NONE:
                        g_user_action = UA_PAUSE;
                        break;
                    case UA_PAUSE:
                        g_user_action = UA_NONE;
                        break;
                    case UA_INC_SPEED:
                        speed_base += 5;
                        buzzle(false);
                        break;
                    case UA_DEC_SPEED:
                        buzzle(false);
                        if (speed_base >= 5)
                            speed_base -= 5;
                        else
                            speed_base = 0;
                        break;
                    case UA_PRE_CALIBRATE:
                        g_user_action=UA_CALIBRATE;
                        turn_off_led(PIN_LED_GREEN);
                        turn_off_led(PIN_LED_BLUE);
                        //red led will be turned on by the calibrate function
                        break;
                }
            }
            else if ((event & 0xff) == BTN_02) {
                //navigate through pause, inc speed, dec speed and pre_calibrate
                switch (g_user_action) {
                    case UA_PAUSE:
                        g_user_action = UA_INC_SPEED;
                        turn_off_red_led();
                        turn_on_led(PIN_LED_GREEN);
                        turn_off_led(PIN_LED_BLUE);
                        break;
                    case UA_INC_SPEED:
                        g_user_action = UA_DEC_SPEED;
                        turn_off_red_led();
                        turn_off_led(PIN_LED_GREEN);
                        turn_on_led(PIN_LED_BLUE);
                        break;
                    case UA_DEC_SPEED:
                        g_user_action=UA_PRE_CALIBRATE;
                        turn_off_red_led();
                        turn_on_led(PIN_LED_GREEN);
                        turn_on_led(PIN_LED_BLUE);
                        break;
                    case UA_PRE_CALIBRATE:
                        g_user_action=UA_PAUSE;
                        turn_on_red_led();
                        turn_off_led(PIN_LED_GREEN);
                        turn_off_led(PIN_LED_BLUE);
                        break;
                }
            }
        }
    }
}

//Observing obstacles, stop the car if needed.  the interruption flag in the context will be set if an obstracle is very close.
//this function will be running in a background thread
void* observor(void *arg) {
	while (g_user_action != UA_DONE) {
		if ((g_car_state & 1) == 1) {
			if (g_scene.seq > 0 && g_scene.front_obstacle <= FRONT_DISTANCE_DANGEROUS) {
				if (debug)
					cout << "!!!!!!!!!!!!!!Danger front, stop now: " << g_scene.front_obstacle << endl;
				stop_car();
				g_context.interruption = INT_FRONT_OBSTACLE;
			}
		}
		else if (g_car_state != CAR_STATE_STOPPED) { //moving backwards
			if (g_scene.seq > 0 && g_scene.rear_obstacle <= REAR_DISTANCE_DANGEROUS) {
				if (debug)
					cout << "!!!!!!!!!!!!!!Danger rear, stop now: " << g_scene.rear_obstacle << endl;
				stop_car();
				g_context.interruption = INT_REAR_OBSTACLE;
			}
		}
		if (digitalRead (PIN_BTN1) == HIGH) {
			delay_ms(100);
			if (digitalRead (PIN_BTN1) == HIGH) {
                mtx_btn_event.lock();
                btn_events.push_back(BTN_01);
                mtx_btn_event.unlock();
			}
		}
		if (digitalRead (PIN_BTN2) == HIGH) {
			delay_ms(100);
			if (digitalRead (PIN_BTN2) == HIGH) {
                mtx_btn_event.lock();
                btn_events.push_back(BTN_02);
                mtx_btn_event.unlock();
			}
		}
		delay_ms(frame_time_ms); //obstacle distance is updated in every frame
	}
}
//capturing frames and analyse each frame to recognize balls, find the nearest ball and get its distance and angle. other information such as
//balls at the left of the nearest ball, balls at the right of the nearest ball are also available and saved to global variable g_scene.
void* sensor(void *arg) {
	memset (&g_scene, 0, sizeof (g_scene));
	bool verbose = debug && (g_user_action == UA_TEST_CAMERA);
	cv::Mat frame;
	long frame_start;
	while (g_user_action != UA_DONE) {
		frame_start = current_time_ms();
		if (g_user_action == UA_NONE || g_user_action == UA_TEST_CAMERA || g_user_action == UA_TEST_SELF) {
			Camera.grab();
			Camera.retrieve (frame);

			cv::Mat hsv;
			cv::cvtColor (frame, hsv, CV_BGR2HSV);
			 //gray color
			cv::Mat mask = cv::Mat(frame.rows, frame.cols, CV_8UC1);
			cv::inRange(hsv, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), mask);
			if (verbose)
				cv::imwrite("step1.jpg",mask);    

			//https://docs.opencv.org/3.4.2/db/df6/tutorial_erosion_dilatation.html
			cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
			cv::erode(mask, mask, element1 );  
			if (verbose)
				cv::imwrite("step2.jpg",mask);       
			
			cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),cv::Point( dilation_size, dilation_size ) );
			cv::dilate( mask, mask, element2 );
			if (verbose)
				cv::imwrite("step3.jpg",mask);            
				
			//find contours
			vector<vector<cv::Point> > contours;
			vector<cv::Vec4i> hierarchy;
			cv::Mat canny_output;
			
			cv::Canny(mask, canny_output, canny_thresh, canny_thresh*2, 3 ); /// Detect edges using canny
			cv::findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );/// Find contours

			/// Draw contours
            int half_width = frame.cols >> 1;
			int total = 0;
			long min_distance = 0;
			int min_angle=180, ball_angle = 180, ball_distance_y = 0, center_x = 0;
			vector<cv::Point> all_positions;
			for( int i = 0; i < contours.size(); i++ ) {
				int area = cv::contourArea(contours[i]);
				if (area < min_area)
					continue;

				cv::Moments mnt =  cv::moments( contours[i], false );
				cv::Point center(mnt.m10/mnt.m00 , mnt.m01/mnt.m00);
				
				int diff_x = center.x - half_width;
				int diff_y = frame.rows - center.y;
				if (diff_y <= 0)
					continue;
					
				all_positions.push_back(center);
				long distance = (long)diff_x * diff_x + (long)diff_y * diff_y;
                int angle = (int)(atan(1.0 * diff_x / diff_y) * 180.0 / 3.14) + camera_deviation;
                if (abs(angle) < abs(min_angle))
                    min_angle = angle;
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
			for (int i = 0; i < all_positions.size(); ++i) {
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
			g_scene.balls = total;
			g_scene.angle = ball_angle;
			g_scene.distance=ball_distance_y;
			g_scene.balls_at_left=balls_at_left;
			g_scene.nearest_ball_at_left=frame.rows - nearest_ball_at_left;
			g_scene.balls_at_right=balls_at_right;
			g_scene.nearest_ball_at_right=frame.rows - nearest_ball_at_right;
            g_scene.min_angle=min_angle;
			g_scene.front_obstacle=measure_front_distance();
			g_scene.rear_obstacle=measure_rear_distance();
			g_scene.seq++;
			if (debug) {
				cout << "#" << g_scene.seq << ": " << g_scene.balls << ",target ball angle " << g_scene.angle << ",dist " << g_scene.distance  << ",all balls min angle " << g_scene.min_angle<< ",obstacles(F/R) " << g_scene.front_obstacle << "/" << g_scene.rear_obstacle << ",balls(L/R) " <<  balls_at_left << "/" << balls_at_right << endl;
			}

			//save image 
			if (verbose) {
				char szFileName[255];
				sprintf(szFileName, "frame%03ld.jpg", g_scene.seq);
				cv::imwrite(szFileName, frame);
			}
		}//paused
		long left =  frame_time_ms - (current_time_ms() - frame_start);
		if (left > 0)
			delay_ms(left);
	}
}

//get one scene
//@returns true if one updated scene is available and retrieved, otherwise false.
bool get_stable_scene(RobotCtx *ctx) {
	if (ctx->scene.balls > 1) {
		memcpy (&ctx->last_scene_w_balls, &ctx->scene, sizeof (Scene));
	}
	Scene *output = &ctx->scene;
	long last_seq = scene_seq_consumed;
	while (scene_seq_consumed == g_scene.seq && (g_user_action == UA_NONE || g_user_action == UA_TEST_CAMERA)) {
		delay_ms(frame_time_ms >> 1);
	}
	if (scene_seq_consumed == g_scene.seq)
		return false;
	memcpy (output, &g_scene, sizeof (Scene));
	scene_seq_consumed = output->seq;
	/*if (debug) {
		long skipped = scene_seq_consumed - last_seq - 1;
		cout << "***Got frame: " << scene_seq_consumed << ", skipped: " << skipped << endl;
	}*/
	return true;
}

//check if the target ball is still in the right position
//@param angle - the ball angle
//@pram distance - the ball distance in pixels
//@param strict - true if the ball should be very close to the center so that it can be picked up.
bool is_covered_raw(int angle, int distance, bool strict) {
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
bool is_covered(Scene *current, bool strict) {
	if (current->balls <= 0)
		return false; //something wrong
	return is_covered_raw(current->angle, current->distance, strict);
}

//@return true if the ball is close enough and in allowed angle
bool is_ready_pickup(Scene *current) {
    if (current->balls <= 0)
        return false;
    return (current->distance <= PIXEL_DISTANCE_PICK_FAR) && is_covered(current, true);
}

//find one ball (turn the car around if needed) and adjust the car direction to make sure the ball is at 
//good positition so that it can be picked up later.
//@param recovering - true if we just lost the "nearest" ball and want to find it again.
//@returns true if at least one ball is found
bool targeting (RobotCtx *context, bool recovering) {
	if (debug)
		cout << "@@@Targeting, recovering?" << recovering << endl;
	bool found = false;
	if ((context->scene.balls <= 0) || abs(context->scene.angle) > GOOD_ANGLE) {
		//step 1, rotate the car until we see a ball
		long till_ms = current_time_ms() + MAX_TURNING_90_MS * 4;//for 360 degree
		int direction = TURNING_DIRECTION_UNKNOWN;
		if (context->scene.balls > 0) {
			direction = context->scene.angle > 0 ? TURNING_DIRECTION_CLOCKWISE : TURNING_DIRECTION_COUNTERCLOCKWISE;
		}
		else {
			direction = choose_turning_driection(context, recovering);
		}
		if ((context->scene.balls <= 0) && context->consecutive_collected > 0) {
			if (direction == TURNING_DIRECTION_CLOCKWISE) {
				turn_car_right_backward();
			}
			else {
				turn_car_left_backward();
			}
			delay_ms(BACK_AFTER_PICKUP_MS);
		}
		rotate_car(direction);
		found = false;
		while (!found && (context->interruption == INT_NONE) && (current_time_ms() < till_ms)) {
			delay_ms(frame_time_ms>>1);
			if (g_user_action == UA_DONE || g_user_action == UA_PAUSE || !get_stable_scene(context))
				break;
			if (context->scene.balls > 0) {
				if (abs(context->scene.angle) <= PERFECT_ANGLE) {
					found = true;
				}
				else if ((context->scene.angle < 0) && (direction == TURNING_DIRECTION_COUNTERCLOCKWISE))
					continue;
				else if ((context->scene.angle > 0) && (direction == TURNING_DIRECTION_CLOCKWISE))
					continue;
				else {
					found = true;
				}
			}
		}
		if (!found)
			return false;
	}
	if (found && is_ready_pickup(&context->scene) || abs(context->scene.angle) <= PERFECT_ANGLE)
		return found;
	{
		//step 2, move back and forth to target the ball
		int last_angle = context->scene.angle, this_angle=0;
		turn_car_360(last_angle > 0 ? TURNING_DIRECTION_CLOCKWISE : TURNING_DIRECTION_COUNTERCLOCKWISE);
		//reset again
		found = false;
		long till_ms = current_time_ms() + MAX_TURNING_90_MS * 4;
		while (!found && (context->interruption == INT_NONE) && (current_time_ms() < till_ms)) {
			delay_ms(frame_time_ms>>1);
			if (g_user_action == UA_DONE || g_user_action == UA_PAUSE || !get_stable_scene(context))
				break;
			if (context->scene.balls > 0) {
				this_angle = context->scene.angle;
                if (is_ready_pickup(&context->scene) || abs(context->scene.angle) <= PERFECT_ANGLE || (this_angle * last_angle < 0)) {//direction reversed
                    found = true;
                }
				last_angle = this_angle;
			}
		}
		g_turning_360 = false;
	}
	return found;
}

//when the car is moving towards the target ball, make sure the ball is still in good position.
//if the ball is a little off the path, adjust the car direction.
bool tracking(RobotCtx *ctx) {
	if (debug)
		cout << "@@@Tracking ball" << endl;
	
	int total_counter=0;
	int counter_left = 0, counter_right = 0;//how many times we saw the ball at left or right
	bool ready_to_pickup = false;
	//tracking the ball
	while (ctx->interruption == INT_NONE && g_user_action == UA_NONE && get_stable_scene(ctx)) {
		if (ctx->scene.balls > 0) {
			if (is_ready_pickup(&ctx->scene)) {
				ready_to_pickup = true;
				break;
			}
			else if (is_covered(&ctx->scene, false)) {//still within allowed range
				int ball_angle = ctx->scene.angle;
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
							turn_car_left_forward();
							turned = true;
						}
						else if (counter_left < counter_right) {
							turn_car_right_forward();
							turned = true;
						}
					}
					if (!turned) {
						move_car_forward();//full speed
					}
					total_counter = counter_left = counter_right = 0;
				}
			}
			else if (targeting(ctx, true)) {//the ball is out of path
				move_car_forward();
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
bool searching(RobotCtx *context) {
	if (debug)
		cout << "@@@Searching" << endl;
	
	if (!get_stable_scene (context))
		return false;

	bool found = false;
	if ((context->scene.balls > 0) && is_covered(&context->scene, true))
		found = true;
	else {
		//turn around to find if there are more balls
		found = targeting(context, false);
		if ((context->interruption == INT_NONE) && context->scene.balls <= 0)
			context->interruption = INT_NO_MORE_BALLS;
	}
	if (found)
		move_car_forward();
	return found;
}

//search to find one ball and pick it up
void picking_up(RobotCtx *ctx) {
	if (debug)
		cout << "@@@Picking up" << endl;
	bool picked_one = false;
	if (g_user_action == UA_NONE) {
		ctx->state = STATE_INIT_PICKING;
		if (searching(ctx)) {
			if (debug)
				cout << "**Found the ball to be picked up, angle " << ctx->scene.angle << endl;
			move_car_forward();
			ctx->state = STATE_PICKING_UP;
			if (tracking(ctx)) {
				if (debug)
					cout << "==============================>Ready to pickup" << endl;
                //wait for the ball to be out of scene while the car is moving forward
				move_car_forward();
                int last_distance = ctx->scene.distance;
                long wait_until = current_time_ms() + 3000; //set a time limitation
                get_stable_scene(ctx);
                while (ctx->scene.balls > 0 && ctx->scene.distance <= last_distance) {
                    if (current_time_ms() > wait_until)
                        break;
                    last_distance = ctx->scene.distance;
                    delay_ms(frame_time_ms);
                    get_stable_scene(ctx);
                }
				if (g_user_action == UA_NONE && ctx->interruption == INT_NONE) {//we are still moving
					start_collector();
					delay_ms(WAIT_BALL_OUT_OF_SCENE_MS);
                    stop_car();
                    delay_ms(WAIT_BALL_PICKUP_MS);
					++ctx->balls_collected;
					picked_one = true;
				}
				stop_collector();
			}
		}
	}
	if (picked_one)
		++ctx->consecutive_collected;
	else
		ctx->consecutive_collected = 0;
}

int main ( int argc,char **argv ) {
	load_config();
	hook_signal();
	wiringPiSetup();

	pinMode(PIN_LMOTORS_1, OUTPUT);
	pinMode(PIN_LMOTORS_2, OUTPUT);
	pinMode(PIN_RMOTORS_1, OUTPUT);
	pinMode(PIN_RMOTORS_2, OUTPUT);
	
	pinMode(PIN_COLLECTOR_1, OUTPUT);
	pinMode(PIN_COLLECTOR_2, OUTPUT);
	
	pinMode(PIN_ECHO_FRONT, INPUT);
	pinMode(PIN_TRIG_FRONT, OUTPUT);
	pinMode(PIN_ECHO_REAR, INPUT);
	pinMode(PIN_TRIG_REAR, OUTPUT);

	pinMode(PIN_BTN1,  INPUT);
    pinMode(PIN_BTN2,  INPUT);
	pinMode(PIN_LED_RED,    OUTPUT);
	pinMode(PIN_LED_GREEN,  OUTPUT);
    pinMode(PIN_LED_BLUE,   OUTPUT);
	pinMode(PIN_BUZZLE, OUTPUT);
	digitalWrite(PIN_BUZZLE, LOW);
	
	softPwmCreate (PIN_PWM_LEFT,  0, PWM_MAX) ;
	softPwmCreate (PIN_PWM_RIGHT, 0, PWM_MAX) ;
	//force to turn led off
	g_is_led_on = true;
	turn_off_red_led();
    turn_off_led(PIN_LED_GREEN);
    turn_off_led(PIN_LED_BLUE);
	//reset
	memset (&g_context, 0, sizeof (RobotCtx));
	for (int i = 1; i < argc; ++i) {
		if (strcmp (argv[i], "-boot") == 0) {
			delay_ms(10 * 1000);
			g_user_action=UA_PAUSE;
			g_context.venue=VENUE_OUTDOOR;
			debug = false;
		}
		else if (strcmp (argv[i], "-court") == 0) {
			g_context.venue=VENUE_OUTDOOR;
		}
		else if (strcmp (argv[i], "-motor") == 0) {
			g_user_action=UA_TEST_MOTOR;
		}
		else if (strcmp (argv[i], "-collector") == 0) {
			g_user_action=UA_TEST_COLLECTOR;
		}
		else if (strcmp (argv[i], "-camera") == 0) {
			g_user_action=UA_TEST_CAMERA;
		}
		else if (strcmp (argv[i], "-self") == 0) {
			g_user_action=UA_TEST_SELF;
		}
		else if (strcmp (argv[i], "-debug") == 0) {
			g_user_action=UA_DEBUG;
		}
		else if (strcmp (argv[i], "-pause") == 0) {
			g_user_action=UA_PAUSE;
		}
	}
	//force to stop
    g_car_state=CAR_STATE_MOVING_BACKWARD;
    stop_car();
    //force to stop
    g_collector_state=COLLECTOR_STATE_RUNNING;
    stop_collector();
    
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
	Camera.set( CV_CAP_PROP_FRAME_WIDTH,  CAMERA_FRAME_WIDTH );
	Camera.set( CV_CAP_PROP_FRAME_HEIGHT, CAMERA_FRAME_HEIGHT );
    Camera.set( CV_CAP_PROP_WHITE_BALANCE_RED_V, 0);//0-100
    Camera.set( CV_CAP_PROP_FPS, frames_per_second);

    if (!Camera.open())
		return 1;

	pthread_t thread_sensor;
	pthread_create(&thread_sensor, NULL, sensor, NULL);
	pthread_t thread_observor;
	pthread_create(&thread_observor, NULL, observor, NULL);
	pthread_t thread_btn_event;
	pthread_create(&thread_btn_event, NULL, btn_event_handle, NULL);

    //tap the button to start
	while (g_user_action != UA_DONE) {
		switch (g_user_action) {
			case UA_NONE:
				turn_off_red_led();
				picking_up(&g_context);
				switch (g_context.interruption) {
					case INT_FRONT_OBSTACLE:
					case INT_REAR_OBSTACLE:
						workaround_obstacle(&g_context);
						g_context.interruption = INT_NONE;
						break;
					case INT_NO_MORE_BALLS:
						if (g_user_action != UA_DONE) {
							g_user_action=UA_PAUSE; //auto pause
							g_context.interruption = INT_NONE;
							buzzle(true);
						}
						break;
					default:
						delay_ms(frame_time_ms);//go to pick up another ball
						break;
				}//inner switch
				break;
			case UA_PAUSE:
				stop_car();
				turn_on_red_led();
				delay_ms(frame_time_ms);
				break;
			case UA_CALIBRATE:
				if (calibrate()) {
					save_config();
				}
				if (g_user_action != UA_DONE) {
					g_user_action=UA_PAUSE;
				}
				break;
			case UA_TEST_MOTOR:
				turn_car_360(TURNING_DIRECTION_CLOCKWISE);
				g_user_action=UA_WAITING;
				break;
			case UA_TEST_COLLECTOR:
				start_collector();
				g_user_action=UA_WAITING;
				break;
			case UA_TEST_CAMERA:
				delay_ms(1000);
				break;
			case UA_TEST_SELF:
				self_test();
				delay_ms(1000);
				break;
			case UA_DEBUG:
				move_car_forward();
				delay_ms(4000);
				g_user_action=UA_DONE;
				break;
            default: //UA_INC_SPEED, UA_DEC_SPEED, UA_PRE_CALIBRATE, UA_WAITING
				delay_ms(1000);
				break;
		}//switch
	} //while
	g_turning_360=false;
    pthread_join(thread_sensor, NULL);
    pthread_join(thread_observor, NULL);
    pthread_join(thread_btn_event, NULL);
    Camera.release();
    turn_off_red_led();
    turn_off_led(PIN_LED_GREEN);
    turn_off_led(PIN_LED_BLUE);
    digitalWrite(PIN_BUZZLE, LOW);
    stop_car();
    stop_collector();
    
    save_config();
    
    return 0;
}
