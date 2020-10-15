#include "Timer.h"

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
		callback_();
		counter_ = 0;
	}
}

void Timer::setPrescale(unsigned int v) {
	prescale_val_ = v;
}

void Timer::setThreshold(counter_t v) {
	threshold_ = v;
}

unsigned int Timer::getPrescale() const
{
	return prescale_val_;
}

Timer::counter_t Timer::getThreshold() const
{
	return threshold_;
}

Timer::counter_t Timer::getCounter() const
{
	return counter_;
}

void Timer::reset()
{
	counter_ = 0;
	prescale_cnt_ = 0;
}