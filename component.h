//base class for vision & location sensors
#include "config.h"

#ifndef __COMPONENT_H__
#define __COMPONENT_H__

class Component {
    protected:
    bool m_paused, m_done, debug;
    Config *m_config;

    public:
    Component(Config *config);
    
    public:
    virtual bool init();
    virtual void start();
    virtual void pause(bool b);
    virtual void stop();
    
    Config *get_config();
    bool is_paused();
    bool is_done();
};

#endif
