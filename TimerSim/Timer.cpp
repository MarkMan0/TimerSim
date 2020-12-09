#include "Timer.h"
#include "stepper.h"

Timer::Timer() {
    callback_ = []() { Stepper::isr(); };
}

Timer::Timer(callback_t cb) : callback_(cb) {

}

void Timer::tick() {
    ++prescale_cnt_;
    if (prescale_cnt_ >= prescale_val_) {
        prev_counter_ = counter_;
        ++counter_;
        prescale_cnt_ = 0;
    }

    if (counter_ >= threshold_ || counter_ < prev_counter_) {
        counter_ = 0;
        callback_();
    }
}

void Timer::setPrescale(unsigned int v) {
    prescale_val_ = v;
}

void Timer::setThreshold(counter_t v) {
    threshold_ = v;
}

void Timer::enable(uint32_t freq)
{
    enabled_ = true;
}
void Timer::disable()
{
    enabled_ = false;
}

unsigned int Timer::getPrescale() const {
    return prescale_val_;
}

Timer::counter_t Timer::getThreshold() const {
    return threshold_;
}

Timer::counter_t Timer::getCounter() const {
    return counter_;
}

void Timer::set_callback(callback_t cb)
{
    callback_ = cb;
}

void Timer::reset() {
    counter_ = 0;
    prescale_cnt_ = 0;
}

Timer stepper_timer;

void timer_start(Timer& timer, uint32_t freq) {

    if (!timer.initialized_) {
        constexpr uint32_t f_cpu = 8000000; // TOOD: check in runtime
        constexpr uint32_t timer_rate = f_cpu / 2, stepper_timer_rate = 2000000;
        constexpr uint32_t psc = timer_rate / stepper_timer_rate;
        timer.setPrescale(psc);
        timer.setThreshold(std::min(static_cast<uint32_t>(timer_rate/ psc), static_cast < uint32_t>(0xFFFFFF)));
        timer.enable();
   }
}
