#pragma once

#include "RingBuffer.h"
#include "utils.h"
#include "MarlinTypes.h"

namespace planner {

    using PointStep = utils::BasePoint<long>;

    enum BlockFlag : char {
        BLOCK_FLAG_RECALCULATE = (1 << 0),
        BLOCK_FLAG_NOMINAL_LENGTH = (1 << 1),
        BLOCK_FLAG_CONTINUED = (1 << 2),
        BLOCK_FLAG_SYNC_POSITION = (1 << 3)
    };

    struct block_t {
        uint8_t flag;
        float nominal_speed_sqr,                  // The nominal speed for this block in (mm/sec)^2
            entry_speed_sqr,                    // Entry speed at previous-current junction in (mm/sec)^2
            max_entry_speed_sqr,                // Maximum allowable junction entry speed in (mm/sec)^2
            millimeters,                        // The total travel of this block in mm
            acceleration;                       // acceleration mm/sec^2

        union {
            marlin::abce_ulong_t steps;                     // Step count along each axis
            marlin::abce_long_t position;                   // New position to force when this sync block is executed
        };
        uint32_t step_event_count;                // The number of step events required to complete this block

        static constexpr uint8_t extruder = 0;
        uint32_t accelerate_until,                // The index of the step event on which to stop acceleration
            decelerate_after;                // The index of the step event on which to start decelerating

        uint32_t acceleration_rate;             // The acceleration rate used for acceleration calculation
        uint8_t direction_bits;

        uint32_t nominal_rate,                    // The nominal step rate for this block in step_events/sec
            initial_rate,                    // The jerk-adjusted step rate at start of block
            final_rate,                      // The minimal rate at exit
            acceleration_steps_per_s2;       // acceleration steps/sec^2

    };

    struct pos_t {
        utils::Point target_;
        double velocity_ = 0;
    };


    class MotionPlanner {

    public:
        void enqueue(const pos_t& p);
        void bufferSegment(const pos_t& p);
        void bufferSteps(const pos_t&, const marlin::abce_long_t&, const marlin::xyze_float_t&);
        void populateBlock(block_t&, const pos_t&, const marlin::abce_long_t&, const marlin::xyze_float_t&);
        void recalculate();

    private:
        double max_accel_[3] = { 500, 500, 500 };
        double max_jerk_[3] = { 10, 10, 10 };
        
        RingBuffer<block_t, 16> buffer_;
        marlin::xyze_long_t position;
        long steps_per_mm_[3] = { 100, 100, 100 };
        PointStep pointToStep(const utils::Point&) const;
        float previous_nominal_speed_sqr_ = 0;
        marlin::xyze_float_t previous_speed_;
    };

};
