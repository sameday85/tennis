#ifndef __PICKER_H__
#define __PICKER_H__

#include "vision.h"
#include "motor.h"
#include "location.h"
#include "observer.h"

#define VENUE_INDOOR        0 //for debugging
#define VENUE_OUTDOOR       1 //tennis court

#define WAIT_BALL_OUT_OF_SCENE_MS   1800 //after the ball is out of scene, wait for this time, then start the collector
#define WAIT_BALL_PICKUP_MS         2000 //the time collector is running
#define BACK_AFTER_PICKUP_MS        2000
#define FRONT_DISTANCE_DANGEROUS    8 //cm
#define REAR_DISTANCE_DANGEROUS     20 //cm

#define PIXEL_DISTANCE_PICK_FAR     180 //far point, at which the ball can be picked up
#define PICKUP_ANGLE_FAR            45  //was 45
#define PIXEL_DISTANCE_PICK_NEAR    120 //near point, at which the ball can be picked up
#define PICKUP_ANGLE_NEAR           65  //

#define PERFECT_ANGLE           6
#define GOOD_ANGLE              30 //if the ball is far away

typedef struct _RobotCtx {
    Scene scene;
    int venue;  //see macros VENUE_XXX
    
    Scene last_scene_w_balls;
    int balls_collected;
    int consecutive_collected;
    int last_turn_direction;
} RobotCtx;

//Main class to pick up balls
class Picker {
    public:
    Picker(Config *config);
    
    public:
    bool init();
    void run();
    void stop();
    void de_init();
    void set_user_action(int act);

    private:
    int choose_turning_driection(bool recovering);
    void workaround_obstacle();
    bool speed_calibrate();
    bool camera_calibrate();
    bool get_stable_scene();
    bool is_covered_raw(int angle, int distance, bool strict);
    bool is_covered(bool strict);
    bool is_ready_pickup();
    bool targeting (bool recovering);
    bool tracking();
    bool searching();
    void picking_up();
    
    private:
    bool debug; //as m_config->is_debug()
    int  m_interruption; //see macros INT_XXX
    int m_user_action;
    Config *m_config;
    Motor *m_motor;
    Vision *m_vision;
    Location *m_location;
    Observer *m_observer;
    RobotCtx m_context;
};


#endif