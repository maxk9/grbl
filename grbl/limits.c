/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
  
#include "grbl.h"
#include "deps/LUFA/Drivers/Board/Board.h"

// Homing axis search distance multiplier. Computed by this value times the cycle travel.
#ifndef HOMING_AXIS_SEARCH_SCALAR
  #define HOMING_AXIS_SEARCH_SCALAR  1.5 // Must be > 1 to ensure limit switch will be engaged.
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR
  #define HOMING_AXIS_LOCATE_SCALAR  5.0 // Must be > 1 to ensure limit switch is cleared.
#endif

void limits_init() 
{
    LIMIT_DDR &= ~(LIMIT_MASK); // Set as input pins
    LIMIT_PORT |= (LIMIT_MASK);  // Enable internal pull-up resistors. Normal high operation.

    if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) {
        LIMIT_PCMSK |= LIMIT_MASK; // Enable specific pins of the Pin Change Interrupt
        PCICR |= (1 << LIMIT_INT); // Enable Pin Change Interrupt
    } else {
        limits_disable();
    }
}

// Disables hard limits.
void limits_disable()
{
  LIMIT_PCMSK &= ~LIMIT_MASK;  // Disable specific pins of the Pin Change Interrupt
  PCICR &= ~(1 << LIMIT_INT);  // Disable Pin Change Interrupt
}


// Returns limit state as a bit-wise uint8 variable. Each bit indicates an axis limit, where 
// triggered is 1 and not triggered is 0. Invert mask is applied. Axes are defined by their
// number in bit position, i.e. Z_AXIS is (1<<2) or bit 2, and Y_AXIS is (1<<1) or bit 1.
uint8_t limits_get_state()
{
  uint8_t limit_state = 0;
  if (!((LIMIT_PIN>>X_LIMIT_BIT)&1)) {
      limit_state = (1<<X_AXIS)|(1<<Y_AXIS); //set limit for X and Y
  }
  return limit_state;
}


// This is the Limit Pin Change Interrupt, which handles the hard limit feature. A bouncing 
// limit switch can cause a lot of problems, like false readings and multiple interrupt calls.
// If a switch is triggered at all, something bad has happened and treat it as such, regardless
// if a limit switch is being disengaged. It's impossible to reliably tell the state of a 
// bouncing pin without a debouncing method. A simple software debouncing feature may be enabled 
// through the config.h file, where an extra timer delays the limit pin read by several milli-
// seconds to help with, not fix, bouncing switches.
// NOTE: Do not attach an e-stop to the limit pins, because this interrupt is disabled during
// homing cycles and will not respond correctly. Upon user request or need, there may be a
// special pinout for an e-stop, but it is generally recommended to just directly connect
// your e-stop switch to the Arduino reset pin, since it is the most correct way to do this.
ISR(LIMIT_INT_vect) // DEFAULT: Limit pin change interrupt process.
{
    // Ignore limit switches if already in an alarm state or in-process of executing an alarm.
    // When in the alarm state, Grbl should have been reset or will force a reset, so any pending 
    // moves in the planner and serial buffers are all cleared and newly sent blocks will be 
    // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
    // limit setting if their limits are constantly triggering after a reset and move their axes.
    if (!((LIMIT_PIN>>X_LIMIT_BIT)&1)) {
        if (sys.state != STATE_ALARM) {
            {
                // Ignore limit switches if already in an alarm state or in-process of executing an alarm.
                // When in the alarm state, Grbl should have been reset or will force a reset, so any pending
                // moves in the planner and serial buffers are all cleared and newly sent blocks will be
                // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
                // limit setting if their limits are constantly triggering after a reset and move their axes.
                if (sys.state != STATE_ALARM) {
                    if (!(sys_rt_exec_alarm)) {
                        mc_reset(); // Initiate system kill.
                        bit_true_atomic(sys_rt_exec_alarm, (EXEC_ALARM_HARD_LIMIT|EXEC_CRITICAL_EVENT)); // Indicate hard limit critical event
                    }
                }
            }
        }
    }
}

extern USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface;
// Homes the specified cycle axes, sets the machine position, and performs a pull-off motion after
// completing. Homing is a special motion case, which involves rapid uncontrolled stops to locate
// the trigger point of the limit switches. The rapid stops are handled by a system level axis lock 
// mask, which prevents the stepper algorithm from executing step pulses. Homing motions typically 
// circumvent the processes for executing motions in normal operation.
// NOTE: Only the abort realtime command can interrupt this process.
// TODO: Move limit pin-specific calls to a general function for portability.
#define DIR_UP 1
#define DIR_DOWN 2

typedef struct {
  int32_t position[N_AXIS];          // The planner position of the tool in absolute steps. Kept separate
                                     // from g-code position for movements requiring multiple line motions,
                                     // i.e. arcs, canned cycles, and backlash compensation.
  float previous_unit_vec[N_AXIS];   // Unit vector of previous path line segment
  float previous_nominal_speed_sqr;  // Nominal speed of previous path line segment
} planner_t;
extern planner_t pl;

uint8_t move_to_from_limit(uint8_t cut_of, uint8_t axis, float *target, float homing_rate) {
    uint8_t bytesAvailable, c;
    //uint8_t limit_state;
//    uint8_t ret_status = false;
    int32_t axis_position;
    uint8_t axislock = 0;
    if (axis == X_AXIS) {
        axis_position = system_convert_corexy_to_y_axis_steps(sys.position);
        sys.position[A_MOTOR] = axis_position;
        sys.position[B_MOTOR] = -axis_position;
    } else {
        axis_position = system_convert_corexy_to_x_axis_steps(sys.position);
        sys.position[A_MOTOR] = sys.position[B_MOTOR] = axis_position;
        target[X_AXIS] = 0.0;
    }

    plan_sync_position(); // Sync planner position to current machine position.
    plan_buffer_line(target, homing_rate, false);

    sys.state = STATE_CYCLE;
    st_prep_buffer(); // Initialize step segment buffer before beginning cycle.
    st_wake_up();
    axislock = (1<<X_AXIS)|(1<<Y_AXIS);

    do {
        //check limits
        if (!cut_of && limits_get_state()) {
            axislock = 0;
        }

        //check commands
        bytesAvailable = CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface);
        while(bytesAvailable--) {
            c = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
            protocol_check_cmds(c);
        }
        //serial_tick();

        //abort if accur
        if (sys_rt_exec_state) {
            if (sys_rt_exec_state & (EXEC_RESET)) {
                mc_reset(); // Stop motors, if they are running.
                protocol_execute_realtime();
                return false;
            }

            //send report
            if (sys_rt_exec_state & EXEC_STATUS_REPORT) {
                report_realtime_status();
                bit_false_atomic(sys_rt_exec_state,EXEC_STATUS_REPORT);
            }
            if (sys_rt_exec_state & EXEC_CYCLE_STOP) {
                bit_false_atomic(sys_rt_exec_state, EXEC_CYCLE_STOP);
                //printPgmString(PSTR("stop\r\n"));
                break;
            }
        }
        st_prep_buffer(); // Check and prep segment buffer. NOTE: Should take no longer than 200us.

    } while(axislock);

    st_reset(); // Immediately force kill steppers and reset step segment buffer.
    plan_reset(); // Reset planner buffer to zero planner current position and to clear previous motions.
    delay_ms(250);

    return true;
}

void limits_go_home(uint8_t cycle_mask) {
    if (sys.abort) {
        return;
    } // Block if system reset has been issued.

    // Initialize
    //uint8_t step_pin[N_AXIS];
    float max_travel = 0.0;
    uint8_t idx = X_AXIS;
    // Set search mode with approach at seek rate to quickly engage the specified cycle_mask limit switches.
    //uint8_t bytesAvailable, c;
   // uint8_t limit_state, axislock;

    float target[N_AXIS] = {0.0};

    for (idx = 0; idx < N_AXIS; idx++) {

        max_travel = max(max_travel, (-HOMING_AXIS_SEARCH_SCALAR)*settings.max_travel[idx]);
        target[idx] = -max_travel;

        if(!move_to_from_limit(false, idx, target, settings.homing_seek_rate))
            return;

        //turn back
        max_travel = settings.homing_pulloff;
        target[idx] = max_travel;

        if(!move_to_from_limit(true, idx, target, settings.homing_seek_rate))
            return;
    }

    // The active cycle axes should now be homed and machine limits have been located. By
    // default, Grbl defines machine space as all negative, as do most CNCs. Since limit switches
    // can be on either side of an axes, check and set axes machine zero appropriately. Also,
    // set up pull-off maneuver from axes limit switches that have been homed. This provides
    // some initial clearance off the switches and should also help prevent them from falsely
    // triggering when hard limits are enabled or when more than one axes shares a limit pin.
    int32_t set_axis_position;
    // Set machine positions for homed limit switches. Don't update non-homed axes.
    for (idx = 0; idx < N_AXIS; idx++) {
        // NOTE: settings.max_travel[] is stored as a negative value.

        set_axis_position = lround(-settings.homing_pulloff * settings.steps_per_mm[idx]);

        if (idx == X_AXIS) {
            int32_t off_axis_position = system_convert_corexy_to_y_axis_steps(sys.position);
            sys.position[A_MOTOR] = set_axis_position + off_axis_position;
            sys.position[B_MOTOR] = set_axis_position - off_axis_position;
        } else if (idx == Y_AXIS) {
            int32_t off_axis_position = system_convert_corexy_to_x_axis_steps(sys.position);
            sys.position[A_MOTOR] = off_axis_position + set_axis_position;
            sys.position[B_MOTOR] = off_axis_position - set_axis_position;
        }
    }
    plan_sync_position(); // Sync planner position to homed machine position.

    //sys.state = STATE_HOMING; // Ensure system state set as homing before returning.
}
//
//
//// Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
//// the workspace volume is in all negative space, and the system is in normal operation.
void limits_soft_check(float *target)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {

    #ifdef HOMING_FORCE_SET_ORIGIN
      // When homing forced set origin is enabled, soft limits checks need to account for directionality.
      // NOTE: max_travel is stored as negative
      if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
        if (target[idx] < 0 || target[idx] > -settings.max_travel[idx]) { sys.soft_limit = true; }
      } else {
        if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { sys.soft_limit = true; }
      }
    #else
      // NOTE: max_travel is stored as negative
      if (target[idx] > -settings.max_travel[idx]) {
//              printPgmString(PSTR("\r\n target X "));
//              printFloat(target[X_AXIS], 1);
//              printPgmString(PSTR("\r\n target Y "));
//              printFloat(target[Y_AXIS], 1);
          sys.soft_limit = true;
      }
    #endif

    if (sys.soft_limit) {
      // Force feed hold if cycle is active. All buffered blocks are guaranteed to be within
      // workspace volume so just come to a controlled stop so position is not lost. When complete
      // enter alarm mode.
      if (sys.state == STATE_CYCLE) {
        bit_true_atomic(sys_rt_exec_state, EXEC_FEED_HOLD);
        do {
          protocol_execute_realtime();
          if (sys.abort) { return; }
        } while ( sys.state != STATE_IDLE );
      }

      mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
      bit_true_atomic(sys_rt_exec_alarm, (EXEC_ALARM_SOFT_LIMIT|EXEC_CRITICAL_EVENT)); // Indicate soft limit critical event
      protocol_execute_realtime(); // Execute to enter critical event loop and system abort
      return;
    }
  }
}
