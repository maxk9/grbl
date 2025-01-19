/*
  pen_control.c - pen control methods
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

/* RC-Servo PWM modification: switch between 0.6ms and 2.5ms pulse-width at 61Hz
   Prescaler 1024 = 15625Hz / 256Steps =  61Hz  64µs/step -> Values 15 / 32 for 1ms / 2ms
   Reload value = 0x07
   Replace this file in C:\Program Files (x86)\Arduino\libraries\GRBL
*/

#include "grbl.h"

#define CNT_ST 100
//189
#define ZERO_CMPR 150

#define POS_DOWN (ZERO_CMPR + 92)
#define POS_UP (ZERO_CMPR + 95)

#define START_CNT4 {TCNT4 = CNT_ST; TIMSK4 = (1<<TOIE4)|(1<<OCIE4A); } /* div = 512 */
#define STOP_CNT4 {TIMSK4 = 0; TCNT4 = CNT_ST; SPINDLE_PWM_PORT &= ~(1<<SPINDLE_PIN); }
//volatile uint8_t cnts_isr_tmr4;

ISR(TIMER4_COMPA_vect) {
    SPINDLE_PWM_PORT |= (1<<SPINDLE_PIN);
}

ISR(TIMER4_OVF_vect) {
   // TC4H = START_CNT4>>8;
    TCNT4 = CNT_ST; //generate 50Hz 1024-625=399
    SPINDLE_PWM_PORT &= ~(1<<SPINDLE_PIN);
}

void pen_init() {
    SPINDLE_PWM_PORT &= ~(1<<SPINDLE_PIN);
    SPINDLE_PWM_DDR |= (1<<SPINDLE_PIN); // Configure as output pin.

    //use tmr4 20ms overflow div = 512 16MHz / 512 = 31.25KHz
    //cnt = 625
    TCCR4A = 1<<PWM4A;
    TCCR4B = 0xC; //div = 2048
    TCCR4C = 0;
    TCNT4 = CNT_ST;
    //TIMSK4 = (1<<TOIE4)|(1<<OCIE4A);
    //OCR4A = 250; //0 0.5-1 ms
    OCR4A = ZERO_CMPR; //90 1.5 ms
    //OCR4A = 237; //180 2-2.5ms
    START_CNT4;
    spindle_run(SPINDLE_ENABLE_CW, 90);  // define initial position as 0 value
}

void spindle_stop() {
   // STOP_CNT4;
}

void spindle_run(uint8_t direction, uint8_t rpm) {
    if (sys.state == STATE_CHECK_MODE) { return; }

    //printPgmString(PSTR("\r\ngot d rpm")); print_uint8_base10(direction); print_uint8_base10(rpm);
    // Empty planner buffer to ensure spindle is set when programmed.
    protocol_auto_cycle_start();  //temp fix for M3 lockup
    protocol_buffer_synchronize();

    if (direction == SPINDLE_DISABLE) {
        OCR4A = POS_DOWN;
    } else {
        /* for handle pen only two position available pull up = 92 and pull down = 95 */
        if ( rpm <= 90 ) { OCR4A = POS_DOWN; }
        else {
            OCR4A = POS_UP;
        }
        //START_CNT4;
    }
}

//void spindle_set_state(pen_state_t state) {
//
//}
