/**
 * StuFW Firmware for 3D Printer
 *
 * Based on MK4duo, Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
 * Part of Grbl
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl. If not, see <http://www.gnu.org/licenses/>.
 */


void restore_stepper_drivers();  // Called by PSU_ON
void reset_stepper_drivers();    // Called by eeprom.load / eeprom.reset

#define X_ENABLE_INIT()           SET_OUTPUT(X_ENABLE_PIN)
#define X_ENABLE_WRITE(STATE)     WRITE(X_ENABLE_PIN,STATE)
#define X_ENABLE_READ()           READ(X_ENABLE_PIN)
#define X_DIR_INIT()              SET_OUTPUT(X_DIR_PIN)
#define X_DIR_WRITE(STATE)        WRITE(X_DIR_PIN,STATE)
#define X_DIR_READ()              READ(X_DIR_PIN)
#define X_STEP_INIT()             SET_OUTPUT(X_STEP_PIN)
#define X_STEP_WRITE(STATE)       WRITE(X_STEP_PIN,STATE)
#define X_STEP_READ()             READ(X_STEP_PIN)


#define Y_ENABLE_INIT()           SET_OUTPUT(Y_ENABLE_PIN)
#define Y_ENABLE_WRITE(STATE)     WRITE(Y_ENABLE_PIN,STATE)
#define Y_ENABLE_READ()           READ(Y_ENABLE_PIN)
#define Y_DIR_INIT()              SET_OUTPUT(Y_DIR_PIN)
#define Y_DIR_WRITE(STATE)        WRITE(Y_DIR_PIN,STATE)
#define Y_DIR_READ()              READ(Y_DIR_PIN)
#define Y_STEP_INIT()             SET_OUTPUT(Y_STEP_PIN)
#define Y_STEP_WRITE(STATE)       WRITE(Y_STEP_PIN,STATE)
#define Y_STEP_READ()             READ(Y_STEP_PIN)


#define Z_ENABLE_INIT()           SET_OUTPUT(Z_ENABLE_PIN)
#define Z_ENABLE_WRITE(STATE)     WRITE(Z_ENABLE_PIN,STATE)
#define Z_ENABLE_READ()           READ(Z_ENABLE_PIN)
#define Z_DIR_INIT()              SET_OUTPUT(Z_DIR_PIN)
#define Z_DIR_WRITE(STATE)        WRITE(Z_DIR_PIN,STATE)
#define Z_DIR_READ()              READ(Z_DIR_PIN)
#define Z_STEP_INIT()             SET_OUTPUT(Z_STEP_PIN)
#define Z_STEP_WRITE(STATE)       WRITE(Z_STEP_PIN,STATE)
#define Z_STEP_READ()             READ(Z_STEP_PIN)


#define X2_ENABLE_INIT()           SET_OUTPUT(X2_ENABLE_PIN)
#define X2_ENABLE_WRITE(STATE)     WRITE(X2_ENABLE_PIN,STATE)
#define X2_ENABLE_READ()           READ(X2_ENABLE_PIN)
#define X2_DIR_INIT()              SET_OUTPUT(X2_DIR_PIN)
#define X2_DIR_WRITE(STATE)        WRITE(X2_DIR_PIN,STATE)
#define X2_DIR_READ()              READ(X2_DIR_PIN)
#define X2_STEP_INIT()             SET_OUTPUT(X2_STEP_PIN)
#define X2_STEP_WRITE(STATE)       WRITE(X2_STEP_PIN,STATE)
#define X2_STEP_READ()             READ(X2_STEP_PIN)


#define Y2_ENABLE_INIT()           SET_OUTPUT(Y2_ENABLE_PIN)
#define Y2_ENABLE_WRITE(STATE)     WRITE(Y2_ENABLE_PIN,STATE)
#define Y2_ENABLE_READ()           READ(Y2_ENABLE_PIN)
#define Y2_DIR_INIT()              SET_OUTPUT(Y2_DIR_PIN)
#define Y2_DIR_WRITE(STATE)        WRITE(Y2_DIR_PIN,STATE)
#define Y2_DIR_READ()              READ(Y2_DIR_PIN)
#define Y2_STEP_INIT()             SET_OUTPUT(Y2_STEP_PIN)
#define Y2_STEP_WRITE(STATE)       WRITE(Y2_STEP_PIN,STATE)
#define Y2_STEP_READ()             READ(Y2_STEP_PIN)


#define Z2_ENABLE_INIT()           SET_OUTPUT(Z2_ENABLE_PIN)
#define Z2_ENABLE_WRITE(STATE)     WRITE(Z2_ENABLE_PIN,STATE)
#define Z2_ENABLE_READ()           READ(Z2_ENABLE_PIN)
#define Z2_DIR_INIT()              SET_OUTPUT(Z2_DIR_PIN)
#define Z2_DIR_WRITE(STATE)        WRITE(Z2_DIR_PIN,STATE)
#define Z2_DIR_READ()              READ(Z2_DIR_PIN)
#define Z2_STEP_INIT()             SET_OUTPUT(Z2_STEP_PIN)
#define Z2_STEP_WRITE(STATE)       WRITE(Z2_STEP_PIN,STATE)
#define Z2_STEP_READ()             READ(Z2_STEP_PIN)


#define E0_ENABLE_INIT()           SET_OUTPUT(E0_ENABLE_PIN)
#define E0_ENABLE_WRITE(STATE)     WRITE(E0_ENABLE_PIN,STATE)
#define E0_ENABLE_READ()           READ(E0_ENABLE_PIN)
#define E0_DIR_INIT()              SET_OUTPUT(E0_DIR_PIN)
#define E0_DIR_WRITE(STATE)        WRITE(E0_DIR_PIN,STATE)
#define E0_DIR_READ()              READ(E0_DIR_PIN)
#define E0_STEP_INIT()             SET_OUTPUT(E0_STEP_PIN)
#define E0_STEP_WRITE(STATE)       WRITE(E0_STEP_PIN,STATE)
#define E0_STEP_READ()             READ(E0_STEP_PIN)



#define E1_ENABLE_INIT()           SET_OUTPUT(E1_ENABLE_PIN)
#define E1_ENABLE_WRITE(STATE)     WRITE(E1_ENABLE_PIN,STATE)
#define E1_ENABLE_READ()           READ(E1_ENABLE_PIN)
#define E1_DIR_INIT()              SET_OUTPUT(E1_DIR_PIN)
#define E1_DIR_WRITE(STATE)        WRITE(E1_DIR_PIN,STATE)
#define E1_DIR_READ()              READ(E1_DIR_PIN)
#define E1_STEP_INIT()             SET_OUTPUT(E1_STEP_PIN)
#define E1_STEP_WRITE(STATE)       WRITE(E1_STEP_PIN,STATE)
#define E1_STEP_READ()             READ(E1_STEP_PIN)


#define E2_ENABLE_INIT()           SET_OUTPUT(E2_ENABLE_PIN)
#define E2_ENABLE_WRITE(STATE)     WRITE(E2_ENABLE_PIN,STATE)
#define E2_ENABLE_READ()           READ(E2_ENABLE_PIN)
#define E2_DIR_INIT()              SET_OUTPUT(E2_DIR_PIN)
#define E2_DIR_WRITE(STATE)        WRITE(E2_DIR_PIN,STATE)
#define E2_DIR_READ()              READ(E2_DIR_PIN)
#define E2_STEP_INIT()             SET_OUTPUT(E2_STEP_PIN)
#define E2_STEP_WRITE(STATE)       WRITE(E2_STEP_PIN,STATE)
#define E2_STEP_READ()             READ(E2_STEP_PIN)


#define E3_ENABLE_INIT()           SET_OUTPUT(E3_ENABLE_PIN)
#define E3_ENABLE_WRITE(STATE)     WRITE(E3_ENABLE_PIN,STATE)
#define E3_ENABLE_READ()           READ(E3_ENABLE_PIN)
#define E3_DIR_INIT()              SET_OUTPUT(E3_DIR_PIN)
#define E3_DIR_WRITE(STATE)        WRITE(E3_DIR_PIN,STATE)
#define E3_DIR_READ()              READ(E3_DIR_PIN)
#define E3_STEP_INIT()             SET_OUTPUT(E3_STEP_PIN)
#define E3_STEP_WRITE(STATE)       WRITE(E3_STEP_PIN,STATE)
#define E3_STEP_READ()             READ(E3_STEP_PIN)


#define E4_ENABLE_INIT()           SET_OUTPUT(E4_ENABLE_PIN)
#define E4_ENABLE_WRITE(STATE)     WRITE(E4_ENABLE_PIN,STATE)
#define E4_ENABLE_READ()           READ(E4_ENABLE_PIN)
#define E4_DIR_INIT()              SET_OUTPUT(E4_DIR_PIN)
#define E4_DIR_WRITE(STATE)        WRITE(E4_DIR_PIN,STATE)
#define E4_DIR_READ()              READ(E4_DIR_PIN)
#define E4_STEP_INIT()             SET_OUTPUT(E4_STEP_PIN)
#define E4_STEP_WRITE(STATE)       WRITE(E4_STEP_PIN,STATE)
#define E4_STEP_READ()             READ(E4_STEP_PIN)


#define E5_ENABLE_INIT()           SET_OUTPUT(E5_ENABLE_PIN)
#define E5_ENABLE_WRITE(STATE)     WRITE(E5_ENABLE_PIN,STATE)
#define E5_ENABLE_READ()           READ(E5_ENABLE_PIN)
#define E5_DIR_INIT()              SET_OUTPUT(E5_DIR_PIN)
#define E5_DIR_WRITE(STATE)        WRITE(E5_DIR_PIN,STATE)
#define E5_DIR_READ()              READ(E5_DIR_PIN)
#define E5_STEP_INIT()             SET_OUTPUT(E5_STEP_PIN)
#define E5_STEP_WRITE(STATE)       WRITE(E5_STEP_PIN,STATE)
#define E5_STEP_READ()             READ(E5_STEP_PIN)

/**
 * Extruder Step for the single E axis
 */
#if DRIVER_EXTRUDERS > 5
  #define E_STEP_WRITE(E,V)     do{ switch (E) { case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); break; case 3: E3_STEP_WRITE(V); break; case 4: E4_STEP_WRITE(V); break; case 5: E5_STEP_WRITE(V); } }while(0)
#elif DRIVER_EXTRUDERS > 4
  #define E_STEP_WRITE(E,V)     do{ switch (E) { case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); break; case 3: E3_STEP_WRITE(V); break; case 4: E4_STEP_WRITE(V); } }while(0)
#elif DRIVER_EXTRUDERS > 3
  #define E_STEP_WRITE(E,V)     do{ switch (E) { case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); break; case 3: E3_STEP_WRITE(V); } }while(0)
#elif DRIVER_EXTRUDERS > 2
  #define E_STEP_WRITE(E,V)     do{ switch (E) { case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); } }while(0)
#elif DRIVER_EXTRUDERS > 1
  #define E_STEP_WRITE(E,V)   do{ if (E == 0) { E0_STEP_WRITE(V); } else { E1_STEP_WRITE(V); } }while(0)
#elif DRIVER_EXTRUDERS > 0
  #define E_STEP_WRITE(E,V)   E0_STEP_WRITE(V)
#endif // DRIVER_EXTRUDERS

class Stepper {

  public: /** Constructor */

    Stepper() { };

  public: /** Public Parameters */

    static flagword_t direction_flag; // Driver Stepper direction flag

    #if HAS_MULTI_ENDSTOP
      static bool separate_multi_axis;
    #endif

    static uint8_t  minimum_pulse;
    static uint32_t maximum_rate,
                    direction_delay;

  private: /** Private Parameters */

    static block_t* current_block;          // A pointer to the block currently being traced

    static uint8_t  last_direction_bits,    // The next stepping-bits to be output
                    axis_did_move;          // Last Movement in the given direction is not null, as computed when the last movement was fetched from planner

    static bool     abort_current_block;    // Signals to the stepper that current block should be aborted

    // Last-moved extruder, as set when the last movement was fetched from planner
    #if EXTRUDERS < 2
      static constexpr uint8_t last_moved_extruder = 0;
    #elif DISABLED(COLOR_MIXING_EXTRUDER)
      static uint8_t last_moved_extruder;
    #endif

    #if ENABLED(X_TWO_ENDSTOPS)
      static bool locked_X_motor, locked_X2_motor;
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      static bool locked_Y_motor, locked_Y2_motor;
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS)
      static bool locked_Z_motor, locked_Z2_motor;
    #endif

    static uint32_t acceleration_time, deceleration_time; // time measured in Stepper Timer ticks
    static uint8_t  steps_per_isr;                        // Count of steps to perform per Stepper ISR call

    // Delta error variables for the Bresenham line tracer
    static int32_t  delta_error[XYZE];
    static uint32_t advance_dividend[XYZE],
                    advance_divisor,
                    step_events_completed,  // The number of step events executed in the current block
                    accelerate_until,       // The point from where we need to stop data.acceleration
                    decelerate_after,       // The point from where we need to start decelerating
                    step_event_count;       // The total event count for the current block

    #if EXTRUDERS > 1 || ENABLED(COLOR_MIXING_EXTRUDER)
      static uint8_t  active_extruder,        // Active extruder
                      active_extruder_driver; // Active extruder driver
    #else
      static constexpr uint8_t  active_extruder = 0,
                                active_extruder_driver = 0;
    #endif

    #if ENABLED(BEZIER_JERK_CONTROL)
      static int32_t  bezier_A,     // A coefficient in B�zier speed curve
                      bezier_B,     // B coefficient in B�zier speed curve
                      bezier_C;     // C coefficient in B�zier speed curve
      static uint32_t bezier_F,     // F coefficient in B�zier speed curve
                      bezier_AV;    // AV coefficient in B�zier speed curve
      #if ENABLED(__AVR__)
        static bool A_negative;     // If A coefficient was negative
      #endif
      static bool bezier_2nd_half;  // If B�zier curve has been initialized or not
    #endif

    static uint32_t nextMainISR;    // time remaining for the next Step ISR
    #if ENABLED(LIN_ADVANCE)
      static uint32_t nextAdvanceISR, LA_isr_rate;
      static uint16_t LA_current_adv_steps, LA_final_adv_steps, LA_max_adv_steps;
      static int8_t   LA_steps;
      static bool     LA_use_advance_lead;
    #endif // !LIN_ADVANCE

    static int32_t ticks_nominal;
    #if DISABLED(BEZIER_JERK_CONTROL)
      static uint32_t acc_step_rate; // needed for deceleration start point
    #endif

    static volatile int32_t endstops_trigsteps[XYZ];

    /**
     * Positions of stepper motors, in step units
     */
    static volatile int32_t count_position[NUM_AXIS];

    /**
     * Current direction of stepper motors (+1 or -1)
     */
    static int8_t count_direction[NUM_AXIS];


  public: /** Public Function */

    /**
     * Initialize stepper hardware
     */
    static void init();

    /**
     * Initialize Factory parameters
     */
    static void factory_parameters();

    /**
     * This is called by the interrupt service routine to execute steps.
     */
    static void Step();

    /**
     * Check if the given block is busy or not - Must not be called from ISR contexts
     */
    static bool is_block_busy(const block_t* const block);

    /**
     * Get the position of a stepper, in steps
     */
    static int32_t position(const AxisEnum axis);

    /**
     * Report the positions of the steppers, in steps
     */
    static void report_positions();

    /**
     * Set direction bits for all steppers
     */
    static void set_directions();

    /**
     * The stepper subsystem goes to sleep when it runs out of things to execute. Call this
     * to notify the subsystem that it is time to go to work.
     */
    FORCE_INLINE static void wake_up() { ENABLE_STEPPER_INTERRUPT(); }

    /**
     * Enabled or Disable one or all stepper driver
     */
    static void enable_X();
    static void disable_X();
    static void enable_Y();
    static void disable_Y();
    static void enable_Z();
    static void disable_Z();
    static void enable_E();
    static void disable_E();
    static void enable_all();
    static void disable_all();

    /**
     * Enabled or Disable Extruder Stepper Driver
     */
    static void enable_E0();
    static void disable_E0();
    #if ENABLED(COLOR_MIXING_EXTRUDER)
      FORCE_INLINE static void enable_E1() { /* nada */ }
      FORCE_INLINE static void enable_E2() { /* nada */ }
      FORCE_INLINE static void enable_E3() { /* nada */ }
      FORCE_INLINE static void enable_E4() { /* nada */ }
      FORCE_INLINE static void enable_E5() { /* nada */ }
      FORCE_INLINE static void disable_E1() { /* nada */ }
      FORCE_INLINE static void disable_E2() { /* nada */ }
      FORCE_INLINE static void disable_E3() { /* nada */ }
      FORCE_INLINE static void disable_E4() { /* nada */ }
      FORCE_INLINE static void disable_E5() { /* nada */ }
    #else
      static void enable_E1();
      static void disable_E1();
      static void enable_E2();
      static void disable_E2();
      static void enable_E3();
      static void disable_E3();
      static void enable_E4();
      static void disable_E4();
      static void enable_E5();
      static void disable_E5();
    #endif

    /**
     * Quickly stop all steppers and clear the blocks queue
     */
    FORCE_INLINE static void quick_stop() { abort_current_block = true; }

    /**
     * The direction of a single motor
     */
    FORCE_INLINE static bool motor_direction(const AxisEnum axis) { return TEST(last_direction_bits, axis); }

    /**
     * The last movement direction was not null on the specified axis. Note that motor direction is not necessarily the same.
     */
    FORCE_INLINE static bool axis_is_moving(const AxisEnum axis) { return TEST(axis_did_move, axis); }

    /**
     * The extruder associated to the last movement
     */
    FORCE_INLINE static uint8_t movement_extruder() {
      return
        #if ENABLED(COLOR_MIXING_EXTRUDER) || EXTRUDERS < 2
          0
        #else
          last_moved_extruder
        #endif
      ;
    }

    /**
     * Handle a triggered endstop
     */
    static void endstop_triggered(const AxisEnum axis);

    /**
     * Triggered position of an axis in steps
     */
    static int32_t triggered_position(const AxisEnum axis);


    #if HAS_MULTI_ENDSTOP
      FORCE_INLINE static void set_separate_multi_axis(const bool state) { separate_multi_axis = state; }
    #endif
    #if ENABLED(X_TWO_ENDSTOPS)
      FORCE_INLINE static void set_x_lock(const bool state) { locked_X_motor = state; }
      FORCE_INLINE static void set_x2_lock(const bool state) { locked_X2_motor = state; }
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      FORCE_INLINE static void set_y_lock(const bool state) { locked_Y_motor = state; }
      FORCE_INLINE static void set_y2_lock(const bool state) { locked_Y2_motor = state; }
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS)
      FORCE_INLINE static void set_z_lock(const bool state) { locked_Z_motor = state; }
      FORCE_INLINE static void set_z2_lock(const bool state) { locked_Z2_motor = state; }
    #endif

    // Set the current position in steps
    static void set_position(const int32_t &a, const int32_t &b, const int32_t &c, const int32_t &e);
    static void set_position(const AxisEnum a, const int32_t &v);

    /**
     * Flag Stepper direction function
     */
    FORCE_INLINE static void setStepDir(const AxisEnum axis, const bool onoff) {
      SET_BIT(direction_flag._word, axis, onoff);
    }
    FORCE_INLINE static bool isStepDir(const AxisEnum axis) { return TEST(direction_flag._word, axis); }


  private: /** Private Function */

    /**
     * Pulse phase Step
     */
    static void pulse_phase_step();

    /**
     * Block phase Step
     */
    static uint32_t block_phase_step();

    /**
     * Pulse tick Start
     */
    static void pulse_tick_start();

    /**
     * Pulse tick Stop
     */
    static void pulse_tick_stop();

    /**
     * Start step X Y Z
     */
    static void start_X_step();
    static void start_Y_step();
    static void start_Z_step();

    /**
     * Stop step X Y Z
     */
    static void stop_X_step();
    static void stop_Y_step();
    static void stop_Z_step();

    /**
     * Set X Y Z direction
     */
    static void set_X_dir(const bool dir);
    static void set_Y_dir(const bool dir);
    static void set_Z_dir(const bool dir);
    static void set_nor_E_dir(const uint8_t e=0);
    static void set_rev_E_dir(const uint8_t e=0);

    /**
     * Set current position in steps
     */
    static void _set_position(const int32_t &a, const int32_t &b, const int32_t &c, const int32_t &e);

    #if DISABLED(COLOR_MIXING_EXTRUDER)
      // Get active driver
      static uint8_t get_active_extruder_driver();
    #endif

    #if ENABLED(LIN_ADVANCE)
      // The Linear advance stepper Step
      static uint32_t lin_advance_step();
    #endif

    #if ENABLED(BEZIER_JERK_CONTROL)
      static void _calc_bezier_curve_coeffs(const int32_t v0, const int32_t v1, const uint32_t av);
      static int32_t _eval_bezier_curve(const uint32_t curr_step);
    #endif

    #if ENABLED(BABYSTEPPING)
      static void babystep(const AxisEnum axis, const bool direction); // perform a short step with a single stepper motor, outside of any convention
    #endif

    #if HAS_EXT_ENCODER
      static void test_extruder_encoder();
    #endif

};

extern Stepper stepper;
