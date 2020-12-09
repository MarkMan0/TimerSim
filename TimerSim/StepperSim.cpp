#include "StepperSim.h"
#include <stdexcept>

void StepperSim::apply_step()
{
    stepCnt += dir;
}

void StepperSim::set_dir(int dir2)
{

    if (dir2 > 0) {
        dir = -1;
    }
    else {
        dir = 1;
    }
}

void StepperSim::set_enabled(bool b)
{
    enabled = b;
}

double StepperSim::get_position() const
{
    return mm_per_step * stepCnt;
}

int64_t StepperSim::get_steps() const
{
    return stepCnt;
}


StepperSim stepperX, stepperY, stepperZ, stepperE0;