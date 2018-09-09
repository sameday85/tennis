/*
 * the tennis picker configuration
*/
#ifndef __CONFIG_H__
#define __CONFIG_H__

//the configuration file path
#define CONFIG_FILE         "/home/pi/.tennis.ini"

//some configurations will be saved/loaded from the configuration file
typedef struct _RobotConfig{
    //for ball recognization
    int minH, maxH, minS, maxS, minV, maxV;//inRange
    int erosion_size, dilation_size;//Blur
    int canny_thresh;//Canny
    int min_area; //min area of the contour

    int speed_base;//add this to the SPEED macros
} RobotConfig;

//Indoor and outdoor configuration management
class Config {
    public:
    Config();
    void activate_indoor_config();
    void activate_outdoor_config();
    RobotConfig *get_active_config();
    
    void load_config();
    void save_config();
    void set_debug(bool d);
    bool is_debug();
    int get_frames_per_second();
    int get_frame_time_ms();

    private:
    int frames_per_second, frame_time_ms; //image capturing
    bool m_debug;
    RobotConfig indoor_config, outdoor_config, *active_config;
};

#endif
