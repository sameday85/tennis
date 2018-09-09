#ifndef __VISION_H__
#define __VISION_H__

#include <raspicam/raspicam_cv.h>
#include <pthread.h>

#include "component.h"

#define CAMERA_FRAME_WIDTH  (320*3)
#define CAMERA_FRAME_HEIGHT (240*3)

//                                                             0
typedef struct _Scene {                //                      |
    int balls; //total number of balls                         |
    int angle; //The angle of the nearest ball 270(-90) -------o---------90
    int distance; //the nearest ball distance in pixels
    int balls_at_left, balls_at_right; //balls at the left or right sections in the scene
    int nearest_ball_at_left, nearest_ball_at_right;
    unsigned long seq; //sequence number
} Scene;

//capturing images and recognizing balls in the image
class Vision : public Component {
    private:
    pthread_t the_thread; 
    bool camera_ready;
    unsigned long scene_seq_consumed;
    
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
