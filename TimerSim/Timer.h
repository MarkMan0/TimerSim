#pragma once

#include <functional>
#include <limits>
#include <cstdint>

class Timer {

public:
    using callback_t = void (*)(void);
    using counter_t = uint32_t;

    Timer();
    Timer(callback_t);
    void tick();

    void setPrescale(unsigned int);
    void setThreshold(counter_t);

    void enable(uint32_t freq = 0);
    void disable();

    unsigned int getPrescale() const;
    counter_t getThreshold() const;
    counter_t getCounter() const;

    void set_callback(callback_t cb);

    void reset();

    friend void timer_start(Timer& timer, uint32_t freq);

private:
    callback_t callback_;

    bool enabled_ = false;
    bool initialized_ = false;

    counter_t threshold_ = std::numeric_limits<counter_t>::max() - 1;
    counter_t counter_ = 0;
    counter_t prev_counter_ = 0;
    unsigned int prescale_val_ = 0;;
    unsigned int prescale_cnt_ = 0;

};

void timer_start(Timer &timer, uint32_t freq);

extern Timer stepper_timer;