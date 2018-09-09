#ifndef __LOCATION_H__
#define __LOCATION_H__

#include "component.h"

//Car location management
class Location : public Component {
    public:
    Location(Config *config);
    virtual ~Location();
    
    private:
    long measure_distance(int pin_trig, int pin_echo);

    public:
    bool init();
    long measure_front_distance(void);
    long measure_rear_distance(void);
    long measure_front_distance_ex(void);
};


#endif
