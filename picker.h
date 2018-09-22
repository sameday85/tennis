#ifndef __PICKER_H__
#define __PICKER_H__

#include "vision.h"
#include "motor.h"
#include "location.h"
#include "observer.h"

#define VENUE_INDOOR        0 //for debugging
#define VENUE_OUTDOOR       1 //tennis court

#define SPEED_PIXELS_PER_MS         0.06 //pixels per milliseconds
#define FRONT_DISTANCE_DANGEROUS    8 //cm
#define REAR_DISTANCE_DANGEROUS     20 //cm

#define PIXEL_DISTANCE_PICK_FAR     180 //far point, at which the ball can be picked up
#define PICKUP_ANGLE_FAR            45  //was 45
#define PIXEL_DISTANCE_PICK_NEAR    120 //near point, at which the ball can be picked up
#define PICKUP_ANGLE_NEAR           65  //

#define PERFECT_ANGLE           6 //idea angle to pick up the ball
#define GOOD_ANGLE              30 //if the ball is far away

//If the ball is far, we think the car has enough time to make a small turn on its way to pick it up
#define PIXEL_DISTANCE_FAR          250 //the ball is far if its pixel distance is great than this value

typedef struct _RobotCtx {
    Scene scene;
    int venue;  //see macros VENUE_XXX
    
    Scene last_scene_w_balls;
    int balls_collected;
    int last_turn_direction;
    int hint_direction; //Recommended turning direction for picking up more balls
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
    int choose_turning_driection();
    void workaround_obstacle();
    void take_snapshot();
    bool get_stable_scene();
    bool is_far();
    bool is_covered(bool strict);
    bool is_ready_pickup();
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
