#ifndef __OBSERVER_H__
#define __OBSERVER_H__

#include <raspicam/raspicam_cv.h>
#include <pthread.h>

#include "pins.h"
#include "component.h"
#include "motor.h"
#include "location.h"

#define FRONT_DISTANCE_DANGEROUS    8 //cm
#define REAR_DISTANCE_DANGEROUS     20 //cm

//interruptions
#define INT_NONE                0
#define INT_FRONT_OBSTACLE      1
#define INT_REAR_OBSTACLE       2
#define INT_NO_MORE_BALLS       3

//user actions
#define UA_NONE                     0
#define UA_PAUSE                    1
#define UA_CAMERA_CALIBRATION       2
#define UA_WAITING                  3
#define UA_DEBUG                    10
#define UA_MOTOR_CALIBRATION        11
#define UA_PRE_MOTOR_CALIBRATION    12
#define UA_PRE_CAMERA_CALIBRATION   13
#define UA_NAV2_PAUSE               14
#define UA_NAV2_RESUME              15
#define UA_DONE                     100

//two buttons
#define BTN_01              1
#define BTN_02              2

//Monitor obstracles and button status
class Observer : public Component {
    public:
    Observer(Config *config, Motor *motor, Location *location, int *flag_interruption, int *user_action);
    virtual ~Observer();
    
    void start();
    bool init();
    void stop();

    private:
    pthread_t the_thread; 
    Motor *m_motor;
    Location *m_location;
    int *p_interruption;
    int *p_user_action; 
    
    private:
    static void* _monitor(void *arg);
    void* btn_event_handle(int event);
};


#endif
