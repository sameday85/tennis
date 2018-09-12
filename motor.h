#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <pthread.h>

#include "component.h"
#include "location.h"

//Motor control
#define MOTORS_LEFT         0
#define MOTORS_RIGHT        1
#define MOTOR_COLLECTOR     4

//directions
#define DIR_FORWARD         0
#define DIR_BACKWARD        1

#define PWM_MAX             100
//All speeds are between 0 and 100(PWM_MAX)
#define TARGET_CALIBRATION_SPEED     16 //cm per second, for calibrating the motor speed
#define MOVING_SPEED        14 //was 15
#define TURNING_SPEED_MIN   10
#define TURNING_SPEED_MAX   30
#define ROTATING_SPEED_FAST 58 //rotate the car fast, once find a ball slow down
#define ROTATING_SPEED_SLOW 34 //

//time limitation for turning 360 degree in fast speed
#define MAX_TURNING_360_MS       20000

//car movement states
#define CAR_STATE_STOPPED                   0
#define CAR_STATE_MOVING_FORWARD            1 //need to check for obstacle if the state is an odd number
#define CAR_STATE_MOVING_BACKWARD           2
#define CAR_STATE_TURNING_LEFT_FORWARD      3 //two motors running in full speed, other two running in half speed, car is moving forward
#define CAR_STATE_TURNING_RIGHT_FORWARD     5
#define CAR_STATE_TURNING_LEFT_BACKWARD     4 //two motors running in full speed, other two running in half speed, car is moving backward
#define CAR_STATE_TURNING_RIGHT_BACKWARD    6
#define CAR_STATE_ROTATING_LEFT_SLOW        21
#define CAR_STATE_ROTATING_RIGHT_SLOW       23
#define CAR_STATE_ROTATING_LEFT_FAST        25
#define CAR_STATE_ROTATING_RIGHT_FAST       27

#define COLLECTOR_STATE_STOPPED         0
#define COLLECTOR_STATE_RUNNING         1

#define TURNING_DIRECTION_UNKNOWN       0
#define TURNING_DIRECTION_RIGHT         1
#define TURNING_DIRECTION_LEFT          2

#define TARGET_CALIBRATION_SPEED     16 //cm per second, for calibrating the motor speed

//Managing the car's four motors and also the collector motor
class Motor : public Component {
    public:
    Motor(Config *p_config);
    virtual ~Motor();
    
    private:
    void start_motor(int motor, int dir);
    void stop_motor(int motor);
    void set_speed_left_right(int left_speed, int right_speed);
    void set_speed(int desired_state);
    static void* _rotate_car_bg(void *arg);
    void _rotate_car(int to_state);
    int measure_speed(Location *p_location);

    public:
    bool init();
    void stop();
    
    void move_car_forward();
    void move_car_backward();
    void turn_car_left_forward();
    void turn_car_right_forward();
    void turn_car_left_backward();
    void turn_car_right_backward();
    void rotate_car_fast(int _direction);
    void rotate_car_slow(int _direction);
    void stop_car();
    
    void start_collector();
    void stop_collector();
    bool calibrate(Location *p_location);
    int get_car_state();
    
    private:
    pthread_t the_thread;
    RobotConfig *active_config;
    
    int m_car_state, m_collector_state;
    int rotate_to_state;    
};

#endif
