
#ifndef __LED_H__
#define __LED_H__

class Led {
    public:

    static void turn_on_led(int pin);
    static void turn_off_led(int pin);
    static void turn_on_red_led();
    static void turn_off_red_led();
    static void turn_off_all_leds();
    
    static void buzzle (bool long_time);
};

#endif
