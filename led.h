
#ifndef __LED_H__
#define __LED_H__

#define LED_STATE_RED_ON            0x01
#define LED_STATE_GREEN_ON          0x02
#define LED_STATE_BLUE_ON           0x04

//Led GPIOs will be initialized by Observer
class Led {
    public:

    static void turn_on_led(int pin);
    static void turn_off_led(int pin);
    static void turn_on_red_led();
    static void turn_off_red_led();
    static void set_led_state(int state);

    static void buzzle (bool long_time);
};

#endif
