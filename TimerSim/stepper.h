/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
 * Derived from Grbl
 *
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "macros.h"
#include "stepper.h"
#include "types.h"
#include "planner.h"

// Disable multiple steps per ISR
//#define DISABLE_MULTI_STEPPING

//
// Estimate the amount of time the Stepper ISR will take to execute
//

/**
 * The method of calculating these cycle-constants is unclear.
 * Most of them are no longer used directly for pulse timing, and exist
 * only to estimate a maximum step rate based on the user's configuration.
 * As 32-bit processors continue to diverge, maintaining cycle counts
 * will become increasingly difficult and error-prone.
 */

  /**
   * Duration of START_TIMED_PULSE
   *
   * ...as measured on an LPC1768 with a scope and converted to cycles.
   * Not applicable to other 32-bit processors, but as long as others
   * take longer, pulses will be longer. For example the SKR Pro
   * (stm32f407zgt6) requires ~60 cyles.
   */
  #define TIMER_READ_ADD_AND_STORE_CYCLES 34UL

  // The base ISR takes 792 cycles
  #define ISR_BASE_CYCLES  792UL


  #define ISR_LA_BASE_CYCLES 0UL
#define STEPPER_TIMER_RATE 2000000

  // S curve interpolation adds 40 cycles]
  #define ISR_S_CURVE_CYCLES 40UL


  // Stepper Loop base cycles
  #define ISR_LOOP_BASE_CYCLES 4UL

  // To start the step pulse, in the worst case takes
  #define ISR_START_STEPPER_CYCLES 13UL

  // And each stepper (start + stop pulse) takes in worst case
  #define ISR_STEPPER_CYCLES 16UL

#define ISR_X_STEPPER_CYCLES       ISR_STEPPER_CYCLES
#define ISR_Y_STEPPER_CYCLES       ISR_STEPPER_CYCLES
#define ISR_Z_STEPPER_CYCLES       ISR_STEPPER_CYCLES

// E is always interpolated, even for mixing extruders
#define ISR_E_STEPPER_CYCLES         ISR_STEPPER_CYCLES

#define ISR_MIXING_STEPPER_CYCLES  0UL

// And the total minimum loop time, not including the base
#define MIN_ISR_LOOP_CYCLES (ISR_X_STEPPER_CYCLES + ISR_Y_STEPPER_CYCLES + ISR_Z_STEPPER_CYCLES + ISR_E_STEPPER_CYCLES + ISR_MIXING_STEPPER_CYCLES)

// Calculate the minimum MPU cycles needed per pulse to enforce, limited to the max stepper rate

#define _MIN_STEPPER_PULSE_CYCLES(N) _MAX(uint32_t((F_CPU) / (MAXIMUM_STEPPER_RATE)), ((F_CPU) / 500000UL) * (N))

#define MIN_STEPPER_PULSE_CYCLES _MIN_STEPPER_PULSE_CYCLES(1UL)

constexpr uint32_t _MIN_PULSE_HIGH_NS = 500000000UL / MAXIMUM_STEPPER_RATE;
constexpr uint32_t _MIN_PULSE_LOW_NS = _MIN_PULSE_HIGH_NS;


// But the user could be enforcing a minimum time, so the loop time is
#define ISR_LOOP_CYCLES (ISR_LOOP_BASE_CYCLES + _MAX(MIN_STEPPER_PULSE_CYCLES, MIN_ISR_LOOP_CYCLES))

// If linear advance is enabled, then it is handled separately
#define ISR_LA_LOOP_CYCLES 0UL

// Now estimate the total ISR execution time in cycles given a step per ISR multiplier
#define ISR_EXECUTION_CYCLES(R) (((ISR_BASE_CYCLES + ISR_S_CURVE_CYCLES + (ISR_LOOP_CYCLES) * (R) + ISR_LA_BASE_CYCLES + ISR_LA_LOOP_CYCLES)) / (R))

// The maximum allowable stepping frequency when doing x128-x1 stepping (in Hz)
#define MAX_STEP_ISR_FREQUENCY_128X ((F_CPU) / ISR_EXECUTION_CYCLES(128))
#define MAX_STEP_ISR_FREQUENCY_64X  ((F_CPU) / ISR_EXECUTION_CYCLES(64))
#define MAX_STEP_ISR_FREQUENCY_32X  ((F_CPU) / ISR_EXECUTION_CYCLES(32))
#define MAX_STEP_ISR_FREQUENCY_16X  ((F_CPU) / ISR_EXECUTION_CYCLES(16))
#define MAX_STEP_ISR_FREQUENCY_8X   ((F_CPU) / ISR_EXECUTION_CYCLES(8))
#define MAX_STEP_ISR_FREQUENCY_4X   ((F_CPU) / ISR_EXECUTION_CYCLES(4))
#define MAX_STEP_ISR_FREQUENCY_2X   ((F_CPU) / ISR_EXECUTION_CYCLES(2))
#define MAX_STEP_ISR_FREQUENCY_1X   ((F_CPU) / ISR_EXECUTION_CYCLES(1))

// The minimum allowable frequency for step smoothing will be 1/10 of the maximum nominal frequency (in Hz)
#define MIN_STEP_ISR_FREQUENCY MAX_STEP_ISR_FREQUENCY_1X

//
// Stepper class definition
//
class Stepper {

  public:


  private:

    static block_t* current_block;          // A pointer to the block currently being traced

    static uint8_t last_direction_bits,     // The next stepping-bits to be output
                   axis_did_move;           // Last Movement in the given direction is not null, as computed when the last movement was fetched from planner

    static bool abort_current_block;        // Signals to the stepper that current block should be aborted

    // Last-moved extruder, as set when the last movement was fetched from planner
     static constexpr uint8_t last_moved_extruder = 0;

    static uint32_t acceleration_time, deceleration_time; // time measured in Stepper Timer ticks
    static uint8_t steps_per_isr;         // Count of steps to perform per Stepper ISR call

      static constexpr uint8_t oversampling_factor = 0;

    // Delta error variables for the Bresenham line tracer
    static xyze_long_t delta_error;
    static xyze_ulong_t advance_dividend;
    static uint32_t advance_divisor,
                    step_events_completed,  // The number of step events executed in the current block
                    accelerate_until,       // The point from where we need to stop acceleration
                    decelerate_after,       // The point from where we need to start decelerating
                    step_event_count;       // The total event count for the current block

      static constexpr uint8_t stepper_extruder = 0;

      static int32_t bezier_A,     // A coefficient in Bézier speed curve
                     bezier_B,     // B coefficient in Bézier speed curve
                     bezier_C;     // C coefficient in Bézier speed curve
      static uint32_t bezier_F,    // F coefficient in Bézier speed curve
                      bezier_AV;   // AV coefficient in Bézier speed curve
      static bool bezier_2nd_half; // If Bézier curve has been initialized or not


    static int32_t ticks_nominal;
    //
    // Exact steps at which an endstop was triggered
    //
    static xyz_long_t endstops_trigsteps;

    //
    // Positions of stepper motors, in step units
    //
    static xyze_long_t count_position;

    //
    // Current direction of stepper motors (+1 or -1)
    //
    static xyze_int8_t count_direction;

  public:

    // Initialize stepper hardware
    static void init();

    // Interrupt Service Routine and phases

    // The stepper subsystem goes to sleep when it runs out of things to execute.
    // Call this to notify the subsystem that it is time to go to work.
    static inline void wake_up() { /*ENABLE_STEPPER_DRIVER_INTERRUPT();*/ }

    static inline bool is_awake() { return true; }

    static inline bool suspend() {
      const bool awake = is_awake();
      if (awake) /*DISABLE_STEPPER_DRIVER_INTERRUPT();*/ NOOP;
      return awake;
    }

    // The ISR scheduler
    static void isr();

    // The stepper pulse ISR phase
    static void pulse_phase_isr();

    // The stepper block processing ISR phase
    static uint32_t block_phase_isr();

    // Check if the given block is busy or not - Must not be called from ISR contexts
    static bool is_block_busy(const block_t* const block);

    // Get the position of a stepper, in steps
    static int32_t position(const AxisEnum axis);

    // Set the current position in steps
    static void set_position(const int32_t &a, const int32_t &b, const int32_t &c, const int32_t &e);
    static inline void set_position(const xyze_long_t &abce) { set_position(abce.a, abce.b, abce.c, abce.e); }
    static void set_axis_position(const AxisEnum a, const int32_t &v);

    // Report the positions of the steppers, in steps
    static void report_a_position(const xyz_long_t &pos);
    static void report_positions();

    // Quickly stop all steppers
    static void quick_stop() { abort_current_block = true; }

    // The direction of a single motor
    static void SET_STEP_DIR(AxisEnum ax);
    static bool motor_direction(const AxisEnum axis) { return TEST(last_direction_bits, axis); }

    // The last movement direction was not null on the specified axis. Note that motor direction is not necessarily the same.
    static bool axis_is_moving(const AxisEnum axis) { return TEST(axis_did_move, axis); }

    // The extruder associated to the last movement
    static uint8_t movement_extruder() {
      return (0);
    }

    // Handle a triggered endstop
    static void endstop_triggered(const AxisEnum axis);

    // Triggered position of an axis in steps
    static int32_t triggered_position(const AxisEnum axis);

    // Set direction bits for all steppers
    static void set_directions();

  private:

    // Set the current position in steps
    static void _set_position(const int32_t &a, const int32_t &b, const int32_t &c, const int32_t &e);
    static void _set_position(const abce_long_t &spos) { _set_position(spos.a, spos.b, spos.c, spos.e); }

    static uint32_t calc_timer_interval(uint32_t step_rate, uint8_t* loops) {
      uint32_t timer;

      // Scale the frequency, as requested by the caller
      step_rate <<= oversampling_factor;

      uint8_t multistep = 1;
        NOMORE(step_rate, uint32_t(MAX_STEP_ISR_FREQUENCY_1X));
      *loops = multistep;

        // In case of high-performance processor, it is able to calculate in real-time
        timer = uint32_t(STEPPER_TIMER_RATE) / step_rate;

      return timer;
    }

      static void _calc_bezier_curve_coeffs(const int32_t v0, const int32_t v1, const uint32_t av);
      static int32_t _eval_bezier_curve(const uint32_t curr_step);

};

extern Stepper stepper;
