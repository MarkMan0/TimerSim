
#include <iostream>
#include "RingBuffer.h"

#include "planner.h"
#include "motion.h"
#include "stepper.h"

#include "Timer.h"
#include "StepperSim.h"

#include <vector>
#include <utility>
#include <fstream>
#include <string>

int main() {

    // set up planner
    for (int i = 0; i < 4; ++i) {
        planner.settings.max_acceleration_mm_per_s2[i] = 100;
        planner.settings.axis_steps_per_mm[i] = 100;
        planner.settings.max_feedrate_mm_s[i] = 100;
        planner.settings.retract_acceleration = 100;
        planner.settings.acceleration = 100;
        planner.settings.travel_acceleration = 100;
        planner.set_max_jerk(X_AXIS, 10);
        planner.set_max_jerk(Y_AXIS, 10);
        planner.set_max_jerk(Z_AXIS, 10);
        planner.set_max_jerk(E0_AXIS, 10);
        planner.reset_acceleration_rates();
        planner.steps_to_mm[i] = 1.0 / 100;
    }

    stepper.init();
    planner.init();

    do_blocking_move_to(200, 0, 0, 50);
    do_blocking_move_to(100, 0, 0, 40);
    bool b = true;
    std::ofstream myfile("output.txt", std::ios::out);
    if (!myfile.is_open()) {
        std::cout << "Couldn't open file" << std::endl;
        return 0;
    }
    double time = 0;
    int64_t lastX = -1;
    while (planner.has_blocks_queued() || planner.cleaning_buffer_counter)
    {
        stepper_timer.tick();
        time += 1.0 / STEPPER_TIMER_RATE;
        if (stepperX.get_steps() != lastX) {
            lastX = stepperX.get_steps();
            myfile << std::to_string(time) << ", " << std::to_string(stepperX.get_position()) << "\n";
        }
        if (stepperX.get_position() == 10) {
            std::cout << std::endl;
            b = false;
        }
    }

    return 0;
}