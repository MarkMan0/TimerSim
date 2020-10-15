#pragma once

#include <functional>
#include <limits>
#include <cstdint>

class Timer {

public:
	using callback_t = void (*)(void);
	using counter_t = uint32_t;

	Timer(callback_t);
	void tick();

	void setPrescale(unsigned int);
	void setThreshold(counter_t);

	unsigned int getPrescale() const;
	counter_t getThreshold() const;
	counter_t getCounter() const;

	void reset();


private:
	callback_t callback_;


	counter_t threshold_ = std::numeric_limits<counter_t>::max() - 1;
	counter_t counter_ = 0;
	counter_t prev_counter_ = 0;
	unsigned int prescale_val_ = 0;;
	unsigned int prescale_cnt_ = 0;

};

