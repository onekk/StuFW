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


#if DISABLED(STRING_CONFIG_H_AUTHOR)
  #define STRING_CONFIG_H_AUTHOR "(none, default config)"
#endif


#if DISABLED(LCD_TIMEOUT_TO_STATUS)
  #define LCD_TIMEOUT_TO_STATUS 15000
#endif

#if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
  #define DOGLCD
  #define U8GLIB_ST7920
  #define REPRAP_DISCOUNT_SMART_CONTROLLER
#endif

#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
  #define ULTIPANEL
  #define NEWPANEL  // click-encoder panel
  #define ULTRA_LCD

#endif

#ifndef STD_ENCODER_PULSES_PER_STEP
  #define STD_ENCODER_PULSES_PER_STEP 5
#endif

#ifndef STD_ENCODER_STEPS_PER_MENU_ITEM
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
#endif

#ifndef ENCODER_PULSES_PER_STEP
  #define ENCODER_PULSES_PER_STEP STD_ENCODER_PULSES_PER_STEP
#endif

#ifndef ENCODER_STEPS_PER_MENU_ITEM
  #define ENCODER_STEPS_PER_MENU_ITEM STD_ENCODER_STEPS_PER_MENU_ITEM
#endif

#ifndef ENCODER_FEEDRATE_DEADZONE
  #define ENCODER_FEEDRATE_DEADZONE 6
#endif

// Aliases for LCD features
#define HAS_SPI_LCD           ENABLED(ULTRA_LCD)
#define HAS_GRAPHICAL_LCD     ENABLED(DOGLCD)
#define HAS_CHARACTER_LCD     (HAS_SPI_LCD && !HAS_GRAPHICAL_LCD)
#define HAS_LCD               ENABLED(NEWPANEL)
#define HAS_LCD_MENU          ENABLED(ULTIPANEL)

#define HAS_ENCODER_ACTION    (HAS_LCD_MENU || ENABLED(ULTIPANEL_FEEDMULTIPLY))
#define HAS_DIGITAL_BUTTONS   ENABLED(NEWPANEL)
#define HAS_ENCODER_WHEEL     ENABLED(NEWPANEL)

// Boot screens
#if !HAS_SPI_LCD
  #undef SHOW_BOOTSCREEN
#elif DISABLED(BOOTSCREEN_TIMEOUT)
  #define BOOTSCREEN_TIMEOUT 2500
#endif

/**
 * Extruders have some combination of stepper motors and hotends
 * so we separate these concepts into the defines:
 *
 *  EXTRUDERS         - Number of Selectable Tools
 *  HOTENDS           - Number of hotends, whether connected or separate
 *  DRIVER_EXTRUDERS  - Number of driver extruders
 *  E_MANUAL          - Number of E steppers for LCD move options
 *
 */
#if ENABLED(COLOR_MIXING_EXTRUDER)      // Multi-stepper, unified E axis, one hotend
  #define SINGLENOZZLE
  #undef  EXTRUDERS
  #undef  DRIVER_EXTRUDERS
  #define EXTRUDERS         1
  #define DRIVER_EXTRUDERS  MIXING_STEPPERS
  #define E_MANUAL          1
  #define HAS_GRADIENT_MIX  (MIXING_STEPPERS == 2)
#else
  #define E_MANUAL          DRIVER_EXTRUDERS
#endif

// One hotend, multi-extruder
#if ENABLED(SINGLENOZZLE) || (EXTRUDERS <= 1)
  #undef HOTENDS
  #define HOTENDS           1
  #undef TEMP_SENSOR_1_AS_REDUNDANT
  #undef HOTEND_OFFSET_X
  #undef HOTEND_OFFSET_Y
  #undef HOTEND_OFFSET_Z
  #define HOTEND_OFFSET_X   { 0 }
  #define HOTEND_OFFSET_Y   { 0 }
  #define HOTEND_OFFSET_Z   { 0 }
  #define HOTEND_INDEX      0
  #define ACTIVE_HOTEND     0
  #define TARGET_HOTEND     0
#else
  #undef HOTENDS
  #define HOTENDS           EXTRUDERS
  #define HOTEND_INDEX      h
  #define ACTIVE_HOTEND     tools.active_extruder
  #define TARGET_HOTEND     tools.target_extruder
#endif

/**
 * Multi-extruders support
 */
#if EXTRUDERS > 1
  #define XYZE_N          (3 + EXTRUDERS)
  #define E_AXIS_N(E)     (uint8_t(E_AXIS) + E)
  #define E_INDEX         (uint8_t(E_AXIS) + tools.active_extruder)
  #define TARGET_EXTRUDER tools.target_extruder
#elif EXTRUDERS == 1
  #define XYZE_N          XYZE
  #define E_AXIS_N(E)     E_AXIS
  #define E_INDEX         E_AXIS
  #define TARGET_EXTRUDER 0
#elif EXTRUDERS == 0
  #undef PIDTEMP
  #define PIDTEMP         false
  #undef FWRETRACT
  #define XYZE_N          XYZ
  #define E_AXIS_N(E)     0
  #define E_INDEX         0
  #define TARGET_EXTRUDER 0
#endif

/**
 * The BLTouch Probe emulates a servo probe
 */
#if ENABLED(BLTOUCH)
  #if DISABLED(ENABLE_SERVOS)
    #define ENABLE_SERVOS
  #endif
  #if Z_PROBE_SERVO_NR < 0
    #undef Z_PROBE_SERVO_NR
    #define Z_PROBE_SERVO_NR 0
  #endif
  #if NUM_SERVOS < 1
    #undef NUM_SERVOS
    #define NUM_SERVOS (Z_PROBE_SERVO_NR + 1)
  #endif
  #undef DEACTIVATE_SERVOS_AFTER_MOVE
  #undef SERVO_DEACTIVATION_DELAY
  #define SERVO_DEACTIVATION_DELAY 50
  #if DISABLED(BLTOUCH_DELAY)
    #define BLTOUCH_DELAY 375
  #endif
  #undef Z_SERVO_ANGLES
  #define Z_SERVO_ANGLES { BLTOUCH_DEPLOY, BLTOUCH_STOW }

  #define BLTOUCH_DEPLOY    10
  #define BLTOUCH_STOW      90
  #define BLTOUCH_SELFTEST 120
  #define BLTOUCH_RESET    160

#endif

// Label Preheat
#ifndef PREHEAT_1_LABEL
  #define PREHEAT_1_LABEL "PLA"
#endif
#ifndef PREHEAT_2_LABEL
  #define PREHEAT_2_LABEL "ABS"
#endif
#ifndef PREHEAT_3_LABEL
  #define PREHEAT_3_LABEL "GUM"
#endif
