/*
  spindle_control.h - spindle control methods
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

#ifndef spindle_control_h
#define spindle_control_h 


// Initializes spindle pins and hardware PWM, if enabled.
void pen_init();

// Sets spindle direction and spindle rpm via PWM, if enabled.
void spindle_run(uint8_t direction, uint8_t rpm);

//void pen_set_state(pen_state_t state);

// Kills spindle.
void spindle_stop();

#endif
