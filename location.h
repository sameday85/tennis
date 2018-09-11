#ifndef __LOCATION_H__
#define __LOCATION_H__

#include "component.h"

#define SAMPLE_SIZE     4 //4 frames per second, Observer is measuring distance twice in one frame, here is for half second

//Car location management
class Location : public Component {
    public:
    Location(Config *config);
    virtual ~Location();
    
    private:
    int front_idx, rear_idx;
    long front[SAMPLE_SIZE], rear[SAMPLE_SIZE];//distances in cm
    long measure_distance(int pin_trig, int pin_echo);
    long calculate_average(long *distances);
    
    public:
    bool init();
    void start();
    long measure_front_distance(void);
    long measure_rear_distance(void);
};


#endif
