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

 /**
  * motion.cpp
  */



#include "motion.h"
#include "planner.h"

#include "types.h"

#include <cstdint>

const XYZval<float> base_min_pos = { 0, 0, 0 };
const XYZval<float> base_max_pos = { 300, 300, 400 };
const XYZval<float> base_home_pos = { 0, 0, 0 };
const XYZval<float> max_length = { 300, 300, 400 };
const XYZval<float> home_bump_mm = { 0, 0, 0 };
const XYZval<signed char> home_dir = { -1, -1, -1 };

float xy_probe_feedrate_mm_s = 60;


/**
 * axis_homed
 *   Flags that each linear axis was homed.
 *   XYZ on cartesian, ABC on delta, ABZ on SCARA.
 *
 * axis_known_position
 *   Flags that the position is known in each linear axis. Set when homed.
 *   Cleared whenever a stepper powers off, potentially losing its position.
 */
uint8_t axis_homed, axis_known_position; // = 0

// Relative Mode. Enable with G91, disable with G90.
bool relative_mode; // = false;

/**
 * Cartesian Current Position
 *   Used to track the native machine position as moves are queued.
 *   Used by 'line_to_current_position' to do a move after changing it.
 *   Used by 'sync_plan_position' to update 'planner.position'.
 */
xyze_pos_t current_position = { 0, 0, 0 };

/**
 * Cartesian Destination
 *   The destination for a move, filled in by G-code movement commands,
 *   and expected by functions like 'prepare_line_to_destination'.
 *   G-codes can set destination using 'get_destination_from_command'
 */
xyze_pos_t destination; // {0}



// The feedrate for the current move, often used as the default if
// no other feedrate is specified. Overridden for special moves.
// Set by the last G0 through G5 command's "F" parameter.
// Functions that override this for custom moves *must always* restore it!
feedRate_t feedrate_mm_s = MMM_TO_MMS(1500);
int16_t feedrate_percentage = 100;

// Homing feedrate is const progmem - compare to constexpr in the header
const feedRate_t homing_feedrate_mm_s[XYZ] = {
    60,60,10
};

// Cartesian conversion result goes here:
xyz_pos_t cartes;


/**
 * The workspace can be offset by some commands, or
 * these offsets may be omitted to save on computation.
 */
 /**
  * Output the current position to serial
  */

  /**
   * sync_plan_position
   *
   * Set the planner/stepper positions directly from current_position with
   * no kinematic translation. Used for homing axes and cartesian/core syncing.
   */
void sync_plan_position() {
    planner.set_position_mm(current_position);
}

void sync_plan_position_e() {
    planner.set_e_position_mm(current_position.e);
}

/**
 * Get the stepper positions in the cartes[] array.
 * Forward kinematics are applied for DELTA and SCARA.
 *
 * The result is in the current coordinate space with
 * leveling applied. The coordinates need to be run through
 * unapply_leveling to obtain the "ideal" coordinates
 * suitable for current_position, etc.
 */
void get_cartesian_from_steppers() {
    cartes.set(planner.get_axis_position_mm(X_AXIS), planner.get_axis_position_mm(Y_AXIS));
    cartes.z = planner.get_axis_position_mm(Z_AXIS);
}

/**
 * Set the current_position for an axis based on
 * the stepper positions, removing any leveling that
 * may have been applied.
 *
 * To prevent small shifts in axis position always call
 * sync_plan_position after updating axes with this.
 *
 * To keep hosts in sync, always call report_current_position
 * after updating the current_position.
 */
void set_current_from_steppers_for_axis(const AxisEnum axis) {
    get_cartesian_from_steppers();
    xyze_pos_t pos = cartes;
    pos.e = planner.get_axis_position_mm(E_AXIS);

    if (axis == ALL_AXES)
        current_position = pos;
    else
        current_position[axis] = pos[axis];
}

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
void line_to_current_position(const feedRate_t& fr_mm_s/*=feedrate_mm_s*/) {
    planner.buffer_line(current_position, fr_mm_s, active_extruder);
}

#if EXTRUDERS
void unscaled_e_move(const float& length, const feedRate_t& fr_mm_s) {
    current_position.e += length / planner.e_factor[active_extruder];
    line_to_current_position(fr_mm_s);
    planner.synchronize();
}
#endif


void _internal_move_to_destination(const feedRate_t& fr_mm_s/*=0.0f*/
) {
    const feedRate_t old_feedrate = feedrate_mm_s;
    if (fr_mm_s) feedrate_mm_s = fr_mm_s;

    const uint16_t old_pct = feedrate_percentage;
    feedrate_percentage = 100;

#if EXTRUDERS
    const float old_fac = planner.e_factor[active_extruder];
    planner.e_factor[active_extruder] = 1.0f;
#endif


    prepare_line_to_destination();

    feedrate_mm_s = old_feedrate;
    feedrate_percentage = old_pct;
#if EXTRUDERS
    planner.e_factor[active_extruder] = old_fac;
#endif
}

/**
 * Plan a move to (X, Y, Z) and set the current_position
 */
void do_blocking_move_to(const float rx, const float ry, const float rz, const feedRate_t& fr_mm_s/*=0.0*/) {

    const feedRate_t z_feedrate = fr_mm_s ? fr_mm_s : homing_feedrate(Z_AXIS),
        xy_feedrate = fr_mm_s ? fr_mm_s : feedRate_t(XY_PROBE_FEEDRATE_MM_S);



    // If Z needs to raise, do it before moving XY
    if (current_position.z < rz) {
        current_position.z = rz;
        line_to_current_position(z_feedrate);
}

    current_position.set(rx, ry);
    line_to_current_position(xy_feedrate);

    // If Z needs to lower, do it after moving XY
    if (current_position.z > rz) {
        current_position.z = rz;
        line_to_current_position(z_feedrate);
    }

    planner.synchronize();
}

void do_blocking_move_to(const xy_pos_t& raw, const feedRate_t& fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(raw.x, raw.y, current_position.z, fr_mm_s);
}
void do_blocking_move_to(const xyz_pos_t& raw, const feedRate_t& fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(raw.x, raw.y, raw.z, fr_mm_s);
}
void do_blocking_move_to(const xyze_pos_t& raw, const feedRate_t& fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(raw.x, raw.y, raw.z, fr_mm_s);
}

void do_blocking_move_to_x(const float& rx, const feedRate_t& fr_mm_s/*=0.0*/) {
    do_blocking_move_to(rx, current_position.y, current_position.z, fr_mm_s);
}
void do_blocking_move_to_y(const float& ry, const feedRate_t& fr_mm_s/*=0.0*/) {
    do_blocking_move_to(current_position.x, ry, current_position.z, fr_mm_s);
}
void do_blocking_move_to_z(const float& rz, const feedRate_t& fr_mm_s/*=0.0*/) {
    do_blocking_move_to_xy_z(current_position, rz, fr_mm_s);
}

void do_blocking_move_to_xy(const float& rx, const float& ry, const feedRate_t& fr_mm_s/*=0.0*/) {
    do_blocking_move_to(rx, ry, current_position.z, fr_mm_s);
}
void do_blocking_move_to_xy(const xy_pos_t& raw, const feedRate_t& fr_mm_s/*=0.0f*/) {
    do_blocking_move_to_xy(raw.x, raw.y, fr_mm_s);
}

void do_blocking_move_to_xy_z(const xy_pos_t& raw, const float& z, const feedRate_t& fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(raw.x, raw.y, z, fr_mm_s);
}

//
// Prepare to do endstop or probe moves with custom feedrates.
//  - Save / restore current feedrate and multiplier
//
static float saved_feedrate_mm_s;
static int16_t saved_feedrate_percentage;
void remember_feedrate_and_scaling() {
    saved_feedrate_mm_s = feedrate_mm_s;
    saved_feedrate_percentage = feedrate_percentage;
}
void remember_feedrate_scaling_off() {
    remember_feedrate_and_scaling();
    feedrate_percentage = 100;
}
void restore_feedrate_and_scaling() {
    feedrate_mm_s = saved_feedrate_mm_s;
    feedrate_percentage = saved_feedrate_percentage;
}


/**
 * Prepare a linear move in a Cartesian setup.
 *
 * When a mesh-based leveling system is active, moves are segmented
 * according to the configuration of the leveling system.
 *
 * Return true if 'current_position' was set to 'destination'
 */
inline bool line_to_destination_cartesian() {
    const float scaled_fr_mm_s = MMS_SCALED(feedrate_mm_s);

    planner.buffer_line(destination, scaled_fr_mm_s, active_extruder);
    return false; // caller will update current_position
}


/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, mesh moves, etc.
 *
 * Make sure current_position.e and destination.e are good
 * before calling or cold/lengthy extrusion may get missed.
 *
 * Before exit, current_position is set to destination.
 */
void prepare_line_to_destination() {
    
    if (line_to_destination_cartesian())
        return;

    current_position = destination;
    }

uint8_t axes_need_homing(uint8_t axis_bits/*=0x07*/) {

#define HOMED_FLAGS axis_homed

    // Clear test bits that are homed
    if (TEST(axis_bits, X_AXIS) && TEST(HOMED_FLAGS, X_AXIS)) CBI(axis_bits, X_AXIS);
    if (TEST(axis_bits, Y_AXIS) && TEST(HOMED_FLAGS, Y_AXIS)) CBI(axis_bits, Y_AXIS);
    if (TEST(axis_bits, Z_AXIS) && TEST(HOMED_FLAGS, Z_AXIS)) CBI(axis_bits, Z_AXIS);
    return axis_bits;
}

bool axis_unhomed_error(uint8_t axis_bits/*=0x07*/) {
    return false;
}

/**
 * Homing bump feedrate (mm/s)
 */
feedRate_t get_homing_bump_feedrate(const AxisEnum axis) {

    return homing_feedrate(axis);
    }


/**
 * Home an individual linear axis
 */
void do_homing_move(const AxisEnum axis, const float distance, const feedRate_t fr_mm_s = 0.0) {

    const feedRate_t real_fr_mm_s = fr_mm_s ? fr_mm_s : homing_feedrate(axis);


    // Only do some things when moving towards an endstop
    const int8_t axis_home_dir = home_dir[axis];
    const bool is_home_dir = (axis_home_dir > 0) == (distance > 0);




    abce_pos_t target = planner.get_axis_positions_mm();
    target[axis] = 0;
    planner.set_machine_position_mm(target);
    target[axis] = distance;


    // Set delta/cartesian axes directly
    planner.buffer_segment(target, real_fr_mm_s, active_extruder);

    planner.synchronize();

    if (is_home_dir) {
        //endstops.validate_homing_move();
    }
}

/**
 * Set an axis' current position to its home position (after homing).
 *
 * For Core and Cartesian robots this applies one-to-one when an
 * individual axis has been homed.
 *
 * DELTA should wait until all homing is done before setting the XYZ
 * current_position to home, because homing is a single operation.
 * In the case where the axis positions are already known and previously
 * homed, DELTA could home to X or Y individually by moving either one
 * to the center. However, homing Z always homes XY and Z.
 *
 * SCARA should wait until all XY homing is done before setting the XY
 * current_position to home, because neither X nor Y is at home until
 * both are at home. Z can however be homed individually.
 *
 * Callers must sync the planner position after calling this!
 */
void set_axis_is_at_home(const AxisEnum axis) {

    SBI(axis_known_position, axis);
    SBI(axis_homed, axis);

    current_position[axis] = base_home_pos[axis];

    /**
     * Z Probe Z Homing? Account for the probe's Z offset.
     */
    if (axis == Z_AXIS) {
        current_position.z -= 0;
    }
}

/**
 * Set an axis' to be unhomed.
 */
void set_axis_not_trusted(const AxisEnum axis) {

    CBI(axis_known_position, axis);
    CBI(axis_homed, axis);
}

/**
 * Home an individual "raw axis" to its endstop.
 * This applies to XYZ on Cartesian and Core robots, and
 * to the individual ABC steppers on DELTA and SCARA.
 *
 * At the end of the procedure the axis is marked as
 * homed and the current position of that axis is updated.
 * Kinematic robots should wait till all axes are homed
 * before updating the current position.
 */

void homeaxis(const AxisEnum axis) {
#define CAN_HOME_X true
#define CAN_HOME_Y true
#define CAN_HOME_Z true

    if (!CAN_HOME_X && !CAN_HOME_Y && !CAN_HOME_Z) return;


    const int axis_home_dir = (home_dir[axis]);


    do_homing_move(axis, 1.5f * max_length[axis] * axis_home_dir);



    set_axis_is_at_home(axis);
    sync_plan_position();

    destination[axis] = current_position[axis];

} // homeaxis()