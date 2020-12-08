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

/**
 * tools.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../StuFW.h"

#if EXTRUDERS > 0

  Tools tools;

  uint8_t Tools::active_extruder    = 0,
          Tools::previous_extruder  = 0,
          Tools::target_extruder    = 0;

  int16_t Tools::flow_percentage[EXTRUDERS]       = ARRAY_BY_EXTRUDERS(100),
          Tools::density_percentage[EXTRUDERS]    = ARRAY_BY_EXTRUDERS(100);
  float   Tools::e_factor[EXTRUDERS]              = ARRAY_BY_EXTRUDERS(1.0);

  #if ENABLED(VOLUMETRIC_EXTRUSION)
    float Tools::filament_size[EXTRUDERS]         = ARRAY_BY_EXTRUDERS(NOMINAL_FILAMENT_DIA),
          Tools::volumetric_area_nominal          = CIRCLE_AREA(float(NOMINAL_FILAMENT_DIA) * 0.5f),
          Tools::volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS(1.0);
  #endif

  float   Tools::hotend_offset[XYZ][HOTENDS] = { 0.0 };

  #if HAS_EXT_ENCODER
    uint8_t Tools::encLastSignal[EXTRUDERS]           = ARRAY_BY_EXTRUDERS(0);
    int8_t  Tools::encLastDir[EXTRUDERS]              = ARRAY_BY_EXTRUDERS(1);
    int32_t Tools::encStepsSinceLastSignal[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0),
            Tools::encLastChangeAt[EXTRUDERS]         = ARRAY_BY_EXTRUDERS(0),
            Tools::encErrorSteps[EXTRUDERS]           = ARRAY_BY_EXTRUDERS(ENC_ERROR_STEPS);
  #endif

  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    int16_t Tools::lpq_len = 20;
  #endif

  void Tools::change(const uint8_t tmp_extruder, const float fr_mm_s/*=0.0*/, bool no_move/*=false*/) {

    #if ENABLED(COLOR_MIXING_EXTRUDER)

      UNUSED(fr_mm_s);
      UNUSED(no_move);

      if (tmp_extruder >= MIXING_VIRTUAL_TOOLS)
        return invalid_extruder_error(tmp_extruder);

      #if MIXING_VIRTUAL_TOOLS > 1
        // T0-Tnnn: Switch virtual tool by changing the index to the mix
        mixer.T(tmp_extruder);
      #endif

    #elif EXTRUDERS < 2

      UNUSED(fr_mm_s);
      UNUSED(no_move);
      if (tmp_extruder) invalid_extruder_error(tmp_extruder);
      return;

    #else // EXTRUDERS > 1

      planner.synchronize();

      #if HAS_LEVELING
        // Set current position to the physical position
        const bool leveling_was_active = bedlevel.flag.leveling_active;
        bedlevel.set_bed_leveling_enabled(false);
      #endif

      if (tmp_extruder >= EXTRUDERS)
        return invalid_extruder_error(tmp_extruder);

      #if HOTENDS > 1

        const float old_feedrate_mm_s = fr_mm_s > 0.0 ? fr_mm_s : mechanics.feedrate_mm_s;

        mechanics.feedrate_mm_s = fr_mm_s > 0.0 ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;

        if (tmp_extruder != active_extruder) {
          if (!no_move && mechanics.axis_unhomed_error()) {
            SERIAL_EM("No move on toolchange");
            no_move = true;
          }

        mechanics.set_destination_to_current();

        const float x_diff = hotend_offset[X_AXIS][tmp_extruder] - hotend_offset[X_AXIS][active_extruder],
                    y_diff = hotend_offset[Y_AXIS][tmp_extruder] - hotend_offset[Y_AXIS][active_extruder],
                    z_diff = hotend_offset[Z_AXIS][tmp_extruder] - hotend_offset[Z_AXIS][active_extruder];

        #if ENABLED(DEBUG_FEATURE)
          if (printer.debugFeature()) {
            SERIAL_MV("Offset Tool XYZ by { ", x_diff);
            SERIAL_MV(", ", y_diff);
            SERIAL_MV(", ", z_diff);
            SERIAL_EM(" }");
           }
         #endif

        // The newly-selected extruder XY is actually at...
        mechanics.current_position[X_AXIS] += x_diff;
        mechanics.current_position[Y_AXIS] += y_diff;
        mechanics.current_position[Z_AXIS] += z_diff;

        // Set the new active extruder
        previous_extruder = active_extruder;
        active_extruder = tmp_extruder;

        // Tell the planner the new "current position"
        mechanics.sync_plan_position();

        constexpr bool safe_to_move = true;

        // Raise, move, and lower again
        if (safe_to_move && !no_move && printer.isRunning()) {
          // Do a small lift to avoid the workpiece in the move back (below)
          mechanics.current_position[Z_AXIS] += 1.0;
          planner.buffer_line(mechanics.current_position, mechanics.data.max_feedrate_mm_s[Z_AXIS], active_extruder);

          #if ENABLED(DEBUG_FEATURE)
            if (printer.debugFeature()) DEBUG_POS("Move back", mechanics.destination);
          #endif
          // Move back to the original (or tweaked) position
          mechanics.do_blocking_move_to(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS]);
          }
        // Move back down. (Including when the new tool is higher.)
        mechanics.do_blocking_move_to_z(mechanics.destination[Z_AXIS], mechanics.data.max_feedrate_mm_s[Z_AXIS]);
        } // (tmp_extruder != active_extruder)

        planner.synchronize();

        mechanics.feedrate_mm_s = old_feedrate_mm_s;

      #else // HOTENDS <= 1

        UNUSED(fr_mm_s);
        UNUSED(no_move);

        #if HAS_MKMULTI_TOOLS
          MK_multi_tool_change(tmp_extruder);
        #else
          // Set the new active extruder
          previous_extruder = active_extruder;
          active_extruder = tmp_extruder;
        #endif

      #endif // HOTENDS <= 1

      #if HAS_LEVELING
        planner.synchronize();
        // Restore leveling to re-establish the logical position
        bedlevel.set_bed_leveling_enabled(leveling_was_active);
      #endif

      SERIAL_LMV(ECHO, MSG_ACTIVE_EXTRUDER, (int)active_extruder);

    #endif // EXTRUDERS <= 1 && (!MIXING_EXTRUDER || MIXING_VIRTUAL_TOOLS <= 1)

  }

  void Tools::print_M218(const uint8_t h) {
    SERIAL_LM(CFG, "Hotend offset (unit): H<Hotend> X<offset> Y<offset> Z<offset>:");
    SERIAL_SMV(CFG, "  M218 H", (int)h);
    SERIAL_MV(" X", LINEAR_UNIT(tools.hotend_offset[X_AXIS][h]), 3);
    SERIAL_MV(" Y", LINEAR_UNIT(tools.hotend_offset[Y_AXIS][h]), 3);
    SERIAL_MV(" Z", LINEAR_UNIT(tools.hotend_offset[Z_AXIS][h]), 3);
    SERIAL_EOL();
  }


  void Tools::invalid_extruder_error(const uint8_t e) {
    SERIAL_SMV(ER, "T", (int)e);
    SERIAL_EM(" " MSG_INVALID_EXTRUDER);
  }

  #if ENABLED(VOLUMETRIC_EXTRUSION)

    /**
     * Get a volumetric multiplier from a filament diameter.
     * This is the reciprocal of the circular cross-section area.
     * Return 1.0 with volumetric off or a diameter of 0.0.
     */
    float Tools::calculate_volumetric_multiplier(const float diameter) {
      return (printer.isVolumetric() && diameter) ? RECIPROCAL(CIRCLE_AREA(diameter * 0.5)) : 1.0;
    }

    /**
     * Convert the filament sizes into volumetric multipliers.
     * The multiplier converts a given E value into a length.
     */
    void Tools::calculate_volumetric_multipliers() {
      for (uint8_t e = 0; e < EXTRUDERS; e++) {
        volumetric_multiplier[e] = calculate_volumetric_multiplier(filament_size[e]);
        refresh_e_factor(e);
      }
    }

  #endif // ENABLED(VOLUMETRIC_EXTRUSION)

  #if ENABLED(MKSE6)

    void Tools::MK_multi_tool_change(const uint8_t e) {

      planner.synchronize(); // Finish all movement

      const int angles[EXTRUDERS] = ARRAY_BY_EXTRUDERS_N (
        MKSE6_SERVOPOS_E0, MKSE6_SERVOPOS_E1,
        MKSE6_SERVOPOS_E2, MKSE6_SERVOPOS_E3,
        MKSE6_SERVOPOS_E4, MKSE6_SERVOPOS_E5
      );
      MOVE_SERVO(MKSE6_SERVO_INDEX, angles[e]);

      #if (MKSE6_SERVO_DELAY > 0)
        printer.safe_delay(MKSE6_SERVO_DELAY);
      #endif

      // Set the new active extruder
      previous_extruder = active_extruder;
      active_extruder = e;
    }

  #elif ENABLED(MKR4)

    void Tools::MK_multi_tool_change(const uint8_t e) {

      planner.synchronize(); // Finish all movement
      stepper.disable_E();

      #if (EXTRUDERS == 4) && HAS_E0E2 && HAS_E1E3 && (DRIVER_EXTRUDERS == 2)

        switch(e) {
          case 0:
            WRITE_RELE(E0E2_CHOICE_PIN, LOW);
            WRITE_RELE(E1E3_CHOICE_PIN, LOW);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
          case 1:
            WRITE_RELE(E0E2_CHOICE_PIN, LOW);
            WRITE_RELE(E1E3_CHOICE_PIN, LOW);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
          case 2:
            WRITE_RELE(E0E2_CHOICE_PIN, HIGH);
            WRITE_RELE(E1E3_CHOICE_PIN, LOW);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
          case 3:
            WRITE_RELE(E0E2_CHOICE_PIN, LOW);
            WRITE_RELE(E1E3_CHOICE_PIN, HIGH);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
        }

      #elif (EXTRUDERS == 3) && HAS_E0E2 && (DRIVER_EXTRUDERS == 2)

        switch(e) {
          case 0:
            WRITE_RELE(E0E2_CHOICE_PIN, LOW);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
          case 1:
            WRITE_RELE(E0E2_CHOICE_PIN, LOW);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
          case 2:
            WRITE_RELE(E0E2_CHOICE_PIN, HIGH);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
        }

      #elif (EXTRUDERS == 2) && HAS_E0E1 && (DRIVER_EXTRUDERS == 1)

        switch(e) {
          case 0:
            WRITE_RELE(E0E1_CHOICE_PIN, LOW);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
          case 1:
            WRITE_RELE(E0E1_CHOICE_PIN, HIGH);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
        }

      #endif // E0E1_CHOICE_PIN E0E2_CHOICE_PIN E1E3_CHOICE_PIN

      // Set the new active extruder
      previous_extruder = active_extruder;
      active_extruder = e;
    }

  #elif ENABLED(MKR6) || ENABLED(MKR12)

    void Tools::MK_multi_tool_change(const uint8_t e) {

      planner.synchronize(); // Finish all movement
      stepper.disable_E();

      #if (EXTRUDERS == 2) && HAS_EX1 && (DRIVER_EXTRUDERS == 1)

        switch(e) {
          case 0:
            WRITE_RELE(EX1_CHOICE_PIN, LOW);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
          case 1:
            WRITE_RELE(EX1_CHOICE_PIN, HIGH);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
        }

      #elif (EXTRUDERS == 3) && HAS_EX1 && HAS_EX2 && (DRIVER_EXTRUDERS == 1)

        switch(e) {
          case 0:
            WRITE_RELE(EX1_CHOICE_PIN, LOW);
            WRITE_RELE(EX2_CHOICE_PIN, LOW);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
          case 1:
            WRITE_RELE(EX1_CHOICE_PIN, HIGH);
            WRITE_RELE(EX2_CHOICE_PIN, LOW);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
          case 2:
            WRITE_RELE(EX1_CHOICE_PIN, HIGH);
            WRITE_RELE(EX2_CHOICE_PIN, HIGH);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
        }

      #elif (EXTRUDERS > 3) && HAS_EX1 && HAS_EX2

        uint8_t multiply = e, driver;

        for (driver = 0; driver < DRIVER_EXTRUDERS; driver++) {
          if (multiply < 3) break;
          multiply -= 3;
        }

        switch(multiply) {
          case 0:
            WRITE_RELE(EX1_CHOICE_PIN, LOW);
            WRITE_RELE(EX2_CHOICE_PIN, LOW);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
          case 1:
            WRITE_RELE(EX1_CHOICE_PIN, HIGH);
            WRITE_RELE(EX2_CHOICE_PIN, LOW);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
          case 2:
            WRITE_RELE(EX1_CHOICE_PIN, HIGH);
            WRITE_RELE(EX2_CHOICE_PIN, HIGH);
            printer.safe_delay(500); // 500 microseconds delay for relay
            stepper.enable_E();
            break;
          default:
            SERIAL_LM(ER, "More Driver Extruders");
            break;
        }

      #endif

      // Set the new active extruder
      previous_extruder = active_extruder;
      active_extruder = e;
    }
  #endif
#endif // EXTRUDERS > 0
