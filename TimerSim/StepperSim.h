#pragma once
#include <cstdint>

class StepperSim
{
public:
    void apply_step();
    void set_dir(int);
    void set_enabled(bool);
    double get_position() const;
    int64_t get_steps() const;

private:
    int64_t stepCnt = 0;
    int dir = 1;
    bool enabled = true;
    double mm_per_step = 1.0 / 100;
};


extern StepperSim stepperX, stepperY, stepperZ, stepperE0;
