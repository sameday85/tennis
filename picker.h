#ifndef __PICKER_H__
#define __PICKER_H__

#include "vision.h"
#include "motor.h"
#include "location.h"
#include "observer.h"

#define VENUE_INDOOR        0 //for debugging
#define VENUE_OUTDOOR       1 //tennis court

#define FRONT_DISTANCE_DANGEROUS    8 //cm
#define REAR_DISTANCE_DANGEROUS     20 //cm

#define PIXEL_DISTANCE_PICK_FAR     180 //far point, at which the ball can be picked up
#define PICKUP_ANGLE_FAR            45  //was 45
#define PIXEL_DISTANCE_PICK_NEAR    120 //near point, at which the ball can be picked up
#define PICKUP_ANGLE_NEAR           65  //

#define PERFECT_ANGLE           6 //idea angle to pick up the ball
#define GOOD_ANGLE              30 //if the ball is far away
#define NARROW_ANGLE            40 //for determing the rightmost or leftmost ball

#define ALGORITHM_TBD               0
#define ALGORITHM_NEAREST_FIRST     1
#define ALGORITHM_RIGHTMOST_FIRST   2
#define ALGORITHM_LEFTMOST_FIRST    3

#define MAX_ALGORITHM_HISTORY       5

#define VISIBLE_DISTANCE_CM         100 //in cm

//If the ball is far, we think the car has enough time to make a small turn on its way to pick it up
#define PIXEL_DISTANCE_FAR          150

typedef struct _RobotCtx {
    Scene scene;
    Ball target_ball; //need a backup as sometimes the target ball disppears in one frame
    int algorithm_history[MAX_ALGORITHM_HISTORY];
    int algorithm_pos;//pointing to the first empty/available cell in above array
    int active_algorithm;
    int max_near_balls; //need to move back a little bit to pick up these near balls
    int venue;  //see macros VENUE_XXX
    int total_balls_collected;
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
    bool should_continue();
    int choose_turning_direction();
    void workaround_obstacle();
    bool get_stable_scene();
    void consume_scene();
    int get_2nd_last_algorithm();
    int get_last_algorithm();
    void push_algorithm(int algorithm);
    std::string get_algorithm_name();
    bool is_far();
    bool is_covered(bool strict);
    bool is_ready_pickup();
    bool tracking();
    bool searching();
    void picking_up();
    void after_pickup(bool picked_one, long delay);

    private:
    bool debug; //as m_config->is_debug()
    int m_interruption; //see macros INT_XXX
    int m_user_action;
    Config *m_config;
    Motor *m_motor;
    Vision *m_vision;
    Location *m_location;
    Observer *m_observer;
    RobotCtx m_context;
};


#endif
