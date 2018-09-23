#ifndef __VISION_H__
#define __VISION_H__

#include <raspicam/raspicam_cv.h>
#include <pthread.h>
#include <mutex>  
#include "component.h"

#define CAMERA_FRAME_WIDTH  (320*3)
#define CAMERA_FRAME_HEIGHT (240*3)
/*
 Pixels     Acutal Distance(cm)
 158            031 (1ft)
 305            061 (2ft)
 426            353 
 Tennis Court: 23.77m x 8.23m
 The nearest distance the camera can see is 8cm
 */
#define MAX_BALLS_AT_VENUE          50
 
#define BALL_SIDE_CENTER             0
#define BALL_SIDE_LEFT               1
#define BALL_SIDE_RIGHT              2

//ball information
typedef struct _Ball {
    int x, y; //position, the bottom center(camera location) coordination is (0,0)
    int angle; //at camera's left - negative values(0 - -90); at camera's right- positive values(0-90)
    int side; //at the nearest ball's left(BALL_SIDE_LEFT) or right(BALL_SIDE_RIGHT) or it is the nearest ball itself(BALL_SIDE_CENTER)
    int area; //opencv contour area
} Ball;

//scene information
typedef struct _Scene {
    //with the first one is always the nearest one
    Ball all_balls[MAX_BALLS_AT_VENUE];
    int total_balls, center_balls, left_balls, right_balls;
    unsigned long seq; //sequence number
} Scene;

//capturing images and recognizing balls in the image
class Vision : public Component {
    private:
    pthread_t the_thread; 
    bool camera_ready;
    unsigned long scene_seq_consumed;
    std::mutex *p_mtx; //for accessing/updating the scene

    raspicam::RaspiCam_Cv *m_camera;
    Scene m_scene;

    private:
    static void* sensor(void *arg);

    public:
    Vision(Config *config);
    virtual ~Vision();

    bool get_stable_scene(Scene *p_scene);
    bool calibrate();

    bool init();
    void start();
    void stop();
};

#endif
