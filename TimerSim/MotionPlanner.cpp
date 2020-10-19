#include "MotionPlanner.h"
#include <math.h>

void planner::MotionPlanner::enqueue(const pos_t& p) {
    bufferSegment(p);
}

void planner::MotionPlanner::bufferSegment(const pos_t& p) {

    const marlin::abce_long_t target = {
    int32_t(lround(p.target_.x_ * steps_per_mm_[0])),
    int32_t(lround(p.target_.y_ * steps_per_mm_[1])),
    int32_t(lround(p.target_.z_ * steps_per_mm_[2])),
    int32_t(0)
    };

    const marlin::xyze_pos_t target_float = { p.target_.x_, p.target_.y_, p.target_.z_, 0 };

    bufferSteps(p, target, target_float);

}

void planner::MotionPlanner::bufferSteps(const pos_t& p, const marlin::abce_long_t& target, const marlin::xyze_float_t& target_float) {

    block_t block;

    populateBlock(block, p, target, target_float);

    recalculate();

}


static float max_allowable_speed_sqr(const float& accel, const float& target_velocity_sqr, const float& distance) {
    return target_velocity_sqr - 2 * accel * distance;
}


void planner::MotionPlanner::populateBlock(block_t& block, const pos_t& p, const marlin::abce_long_t& target, const marlin::xyze_float_t& target_float) {

    const int32_t da = target.a - position.a,
        db = target.b - position.b,
        dc = target.c - position.c;

    uint8_t dm = 0;
    if (da < 0) {
        dm |= (1 << 0);
    }
    if (db < 0) {
        dm |= (1 << 1);
    }
    if (dc < 0) {
        dm |= (1 << 2);
    }

    block.flag = 0;
    block.direction_bits = dm;

    block.steps.set(abs(da), abs(db), abs(dc));

    marlin::abce_float_t steps_dist_mm;

    steps_dist_mm.a = da * 1 / steps_per_mm_[0];
    steps_dist_mm.b = db * 1 / steps_per_mm_[1];
    steps_dist_mm.c = dc * 1 / steps_per_mm_[2];

    block.millimeters = sqrt(utils::pow2(steps_dist_mm.x) + utils::pow2(steps_dist_mm.y) + utils::pow2(steps_dist_mm.z));

    block.step_event_count = utils::max_(block.steps.a, block.steps.b, block.steps.c, 0);

    const double inverse_millimeters = 1.0f / block.millimeters;  // Inverse millimeters to remove multiple divides

    double fr_mm_s = p.velocity_;
    float inverse_secs = fr_mm_s * inverse_millimeters;

    block.nominal_speed_sqr = utils::pow2(block.millimeters * inverse_secs);   // (mm/sec)^2 Always > 0
    block.nominal_rate = ceil(block.step_event_count * inverse_secs); // (step/sec) Always > 0

    marlin::xyze_float_t current_speed;
    for (int i = 0; i < 3; ++i) {
        current_speed[i] = steps_dist_mm[i] * inverse_secs;
    }


    const float steps_per_mm = block.step_event_count * inverse_millimeters;

    float accel = 500;  //default

    auto limit_accel = [this, &block, &accel](int axis) {
        if (block.steps[axis] && max_accel_[axis] < accel) {
            const float accel_steps_per_s2 = max_accel_[axis] * steps_per_mm_[axis];
            const float comp = accel_steps_per_s2 * (float)block.step_event_count;
            if ((float)accel * (float)block.steps[axis] > comp) accel = comp / (float)block.steps[axis];
        }
    };

    limit_accel(0);
    limit_accel(1);
    limit_accel(2);

    block.acceleration_steps_per_s2 = accel;
    block.acceleration = accel / steps_per_mm;

    float vmax_junction_sqr;

    float nominal_speed = sqrt(block.nominal_speed_sqr);

    static float previous_safe_speed;

    // Start with a safe speed (from which the machine may halt to stop immediately).
    float safe_speed = nominal_speed;

    uint8_t limited = 0;

    for (int i = 0; i < 3; ++i) {

        const float jerk = abs(current_speed[i]),   // cs : Starting from zero, change in speed for this axis
            maxj = (max_jerk_[i]); // mj : The max jerk setting for this axis
        if (jerk > maxj) {                          // cs > mj : New current speed too fast?
            if (limited) {                            // limited already?
                const float mjerk = nominal_speed * maxj; // ns*mj
                if (jerk * safe_speed > mjerk) safe_speed = mjerk / jerk; // ns*mj/cs
            } else {
                safe_speed *= maxj / jerk;              // Initial limit: ns*mj/cs
                ++limited;                              // Initially limited
            }
        }
    }


    float vmax_junction;
    bool moves_queued = buffer_.hasItems();
    if (moves_queued && !(previous_nominal_speed_sqr_ < 0.00001f)) {

        float v_factor = 1;
        limited = 0;

        float previous_nominal_speed = sqrt(previous_nominal_speed_sqr_);
        vmax_junction = utils::min_(nominal_speed, previous_nominal_speed);

        const float smaller_speed_factor = vmax_junction / previous_nominal_speed;

        for (int axis = 0; axis < 3; ++axis) {
            float v_exit = previous_speed_[axis] * smaller_speed_factor,
                v_entry = current_speed[axis];
            if (limited) {
                v_exit *= v_factor;
                v_entry *= v_factor;
            }

            // Calculate jerk depending on whether the axis is coasting in the same direction or reversing.
            const float jerk = (v_exit > v_entry)
                ? //                                  coasting             axis reversal
                ((v_entry > 0 || v_exit < 0) ? (v_exit - v_entry) : utils::max_(v_exit, -v_entry))
                : // v_exit <= v_entry                coasting             axis reversal
                ((v_entry < 0 || v_exit > 0) ? (v_entry - v_exit) : utils::max_(-v_exit, v_entry));

            const float maxj = (max_jerk_[axis]);

            if (jerk > maxj) {
                v_factor *= maxj / jerk;
                ++limited;
            }
        }

        if (limited) vmax_junction *= v_factor;
        // Now the transition velocity is known, which maximizes the shared exit / entry velocity while
        // respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve faster prints.
        const float vmax_junction_threshold = vmax_junction * 0.99f;
        if (previous_safe_speed > vmax_junction_threshold && safe_speed > vmax_junction_threshold) {
            vmax_junction = safe_speed;
        }
    } else {
        vmax_junction = safe_speed;
    }
    previous_safe_speed = safe_speed;
    vmax_junction_sqr = utils::pow2(vmax_junction);

    block.max_entry_speed_sqr = vmax_junction_sqr;

    const float v_allowable_sqr = max_allowable_speed_sqr(-block.acceleration, utils::pow2(float(0.05)), block.millimeters);

    block.entry_speed_sqr = utils::pow2(float(0.05)) ;

    block.flag |= block.nominal_speed_sqr <= v_allowable_sqr ? BLOCK_FLAG_RECALCULATE | BLOCK_FLAG_NOMINAL_LENGTH : BLOCK_FLAG_RECALCULATE;

    previous_speed_ = current_speed;
    previous_nominal_speed_sqr_ = block.nominal_speed_sqr;

    // Update the position
    position = target;
}

planner::PointStep planner::MotionPlanner::pointToStep(const utils::Point& p) const {

    PointStep steps;
    steps.x_ = steps_per_mm_[0] * p.x_;
    steps.y_ = steps_per_mm_[1] * p.y_;
    steps.z_ = steps_per_mm_[2] * p.z_;

    return steps;
}
