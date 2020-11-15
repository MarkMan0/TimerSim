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
  * motion.h
  *
  * High-level motion commands to feed the planner
  * Some of these methods may migrate to the planner class.
  */

#include "macros.h"
#include "types.h"

  // Axis homed and known-position states
extern uint8_t axis_homed, axis_known_position;
constexpr uint8_t xyz_bits = _BV(X_AXIS) | _BV(Y_AXIS) | _BV(Z_AXIS);
inline bool no_axes_homed() {
    return !axis_homed;
}
inline bool all_axes_homed() {
    return (axis_homed & xyz_bits) == xyz_bits;
}
inline bool all_axes_known() {
    return (axis_known_position & xyz_bits) == xyz_bits;
}
inline void set_all_unhomed() {
    axis_homed = 0;
}
inline void set_all_unknown() {
    axis_known_position = 0;
}

inline bool homing_needed() {
    return !(all_axes_homed());
}

// Error margin to work around float imprecision
constexpr float fslop = 0.0001;

extern bool relative_mode;

extern xyze_pos_t current_position,  // High-level current tool position
destination;       // Destination for a move


// Scratch space for a cartesian result
extern xyz_pos_t cartes;



extern float xy_probe_feedrate_mm_s;
#define XY_PROBE_FEEDRATE_MM_S xy_probe_feedrate_mm_s


/**
 * Feed rates are often configured with mm/m
 * but the planner and stepper like mm/s units.
 */
extern const feedRate_t homing_feedrate_mm_s[XYZ];
inline feedRate_t homing_feedrate(const AxisEnum a) {
    return homing_feedrate_mm_s[a];
}
feedRate_t get_homing_bump_feedrate(const AxisEnum axis);

extern feedRate_t feedrate_mm_s;

/**
 * Feedrate scaling
 */
extern int16_t feedrate_percentage;

// The active extruder (tool). Set with T<extruder> command.
constexpr uint8_t active_extruder = 0;


inline float pgm_read_any(const float* p) {
    return (*p);
}
inline signed char pgm_read_any(const signed char* p) {
    return (*p);
}

extern const XYZval<float> base_min_pos;
extern const XYZval<float> base_max_pos;
extern const XYZval<float> base_home_pos;
extern const XYZval<float> base_min_pos;
extern const XYZval<float> max_length;
extern const XYZval<float> home_bump_mm;
extern const XYZval<signed char> home_dir;


constexpr xyz_pos_t hotend_offset[1] = { { 0 } };

typedef struct {
    xyz_pos_t min, max;
} axis_limits_t;
constexpr bool soft_endstops_enabled = false;


void get_cartesian_from_steppers();
void set_current_from_steppers_for_axis(const AxisEnum axis);

/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
void sync_plan_position();
void sync_plan_position_e();

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
void line_to_current_position(const feedRate_t& fr_mm_s = feedrate_mm_s);

#if EXTRUDERS
void unscaled_e_move(const float& length, const feedRate_t& fr_mm_s);
#endif

void prepare_line_to_destination();

void _internal_move_to_destination(const feedRate_t& fr_mm_s = 0.0f);

inline void prepare_internal_move_to_destination(const feedRate_t& fr_mm_s = 0.0f) {
    _internal_move_to_destination(fr_mm_s);
}


/**
 * Blocking movement and shorthand functions
 */
void do_blocking_move_to(const float rx, const float ry, const float rz, const feedRate_t& fr_mm_s = 0.0f);
void do_blocking_move_to(const xy_pos_t& raw, const feedRate_t& fr_mm_s = 0.0f);
void do_blocking_move_to(const xyz_pos_t& raw, const feedRate_t& fr_mm_s = 0.0f);
void do_blocking_move_to(const xyze_pos_t& raw, const feedRate_t& fr_mm_s = 0.0f);

void do_blocking_move_to_x(const float& rx, const feedRate_t& fr_mm_s = 0.0f);
void do_blocking_move_to_y(const float& ry, const feedRate_t& fr_mm_s = 0.0f);
void do_blocking_move_to_z(const float& rz, const feedRate_t& fr_mm_s = 0.0f);

void do_blocking_move_to_xy(const float& rx, const float& ry, const feedRate_t& fr_mm_s = 0.0f);
void do_blocking_move_to_xy(const xy_pos_t& raw, const feedRate_t& fr_mm_s = 0.0f);
inline void do_blocking_move_to_xy(const xyz_pos_t& raw, const feedRate_t& fr_mm_s = 0.0f) {
    do_blocking_move_to_xy(xy_pos_t(raw), fr_mm_s);
}
inline void do_blocking_move_to_xy(const xyze_pos_t& raw, const feedRate_t& fr_mm_s = 0.0f) {
    do_blocking_move_to_xy(xy_pos_t(raw), fr_mm_s);
}

void do_blocking_move_to_xy_z(const xy_pos_t& raw, const float& z, const feedRate_t& fr_mm_s = 0.0f);
inline void do_blocking_move_to_xy_z(const xyz_pos_t& raw, const float& z, const feedRate_t& fr_mm_s = 0.0f) {
    do_blocking_move_to_xy_z(xy_pos_t(raw), z, fr_mm_s);
}
inline void do_blocking_move_to_xy_z(const xyze_pos_t& raw, const float& z, const feedRate_t& fr_mm_s = 0.0f) {
    do_blocking_move_to_xy_z(xy_pos_t(raw), z, fr_mm_s);
}

void remember_feedrate_and_scaling();
void remember_feedrate_scaling_off();
void restore_feedrate_and_scaling();

//
// Homing
//

uint8_t axes_need_homing(uint8_t axis_bits = 0x07);
bool axis_unhomed_error(uint8_t axis_bits = 0x07);


#define MOTION_CONDITIONS IsRunning()

void set_axis_is_at_home(const AxisEnum axis);

void set_axis_not_trusted(const AxisEnum axis);

void homeaxis(const AxisEnum axis);

/**
 * Workspace offsets
 */

#define NATIVE_TO_LOGICAL(POS, AXIS) (POS)
#define LOGICAL_TO_NATIVE(POS, AXIS) (POS)
inline void toLogical(xy_pos_t&) { }
inline void toLogical(xyz_pos_t&) { }
inline void toLogical(xyze_pos_t&) { }
inline void toNative(xy_pos_t&) { }
inline void toNative(xyz_pos_t&) { }
inline void toNative(xyze_pos_t&) { }
#define LOGICAL_X_POSITION(POS) NATIVE_TO_LOGICAL(POS, X_AXIS)
#define LOGICAL_Y_POSITION(POS) NATIVE_TO_LOGICAL(POS, Y_AXIS)
#define LOGICAL_Z_POSITION(POS) NATIVE_TO_LOGICAL(POS, Z_AXIS)
#define RAW_X_POSITION(POS)     LOGICAL_TO_NATIVE(POS, X_AXIS)
#define RAW_Y_POSITION(POS)     LOGICAL_TO_NATIVE(POS, Y_AXIS)
#define RAW_Z_POSITION(POS)     LOGICAL_TO_NATIVE(POS, Z_AXIS)

/**
 * position_is_reachable family of functions
 */
 // Return true if the given position is within the machine bounds.
inline bool position_is_reachable(const float& rx, const float& ry) {
    return true;
}
inline bool position_is_reachable(const xy_pos_t& pos) {
    return position_is_reachable(pos.x, pos.y);
}


inline int x_home_dir(const uint8_t) {
    return -1;
}
