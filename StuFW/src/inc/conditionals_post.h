/**
 * StuFW Firmware for 3D Printer
 *
 * Based on MK4duo, Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 *-------------------------------------
 * conditionals_post.h
 * Defines that depend on configuration but are not editable.
 */

#pragma once

/**
 * Stored Position
 */
#if DISABLED(NUM_POSITON_SLOTS)
  #define NUM_POSITON_SLOTS 2
#elif NUM_POSITON_SLOTS < 2
  #define NUM_POSITON_SLOTS 2
#endif


#if ((X_MIN_POS) <= 0)
  #define X_MIN_POS_ZERO 0
#else
  #define X_MIN_POS_ZERO (X_MIN_POS)
#endif

#if ((Y_MIN_POS) <= 0)
  #define Y_MIN_POS_ZERO 0
#else
  #define Y_MIN_POS_ZERO (Y_MIN_POS)
#endif


/**
 * Axis lengths and center
 */
#define X_MAX_LENGTH (X_MAX_POS - (X_MIN_POS))
#define Y_MAX_LENGTH (Y_MAX_POS - (Y_MIN_POS))
#define Z_MAX_LENGTH (Z_MAX_POS - (Z_MIN_POS))

// Defined only if the sanity-check is bypassed
#ifndef X_BED_SIZE
  #define X_BED_SIZE X_MAX_LENGTH
#endif
#ifndef Y_BED_SIZE
  #define Y_BED_SIZE Y_MAX_LENGTH
#endif

// Define center values for future use
#ifdef BED_CENTER_AT_0_0
  #define X_CENTER 0
  #define Y_CENTER 0
#else
  #define X_CENTER ((X_BED_SIZE) / 2)
  #define Y_CENTER ((Y_BED_SIZE) / 2)
#endif
#define Z_CENTER ((Z_MIN_POS + Z_MAX_POS) / 2)

// Get the linear boundaries of the bed
#define X_MIN_BED (X_CENTER - (X_BED_SIZE) / 2)
#define X_MAX_BED (X_CENTER + (X_BED_SIZE) / 2)
#define Y_MIN_BED (Y_CENTER - (Y_BED_SIZE) / 2)
#define Y_MAX_BED (Y_CENTER + (Y_BED_SIZE) / 2)

/**
 * CoreXY, CoreXZ, and CoreYZ - and their reverse
 */
#if IS_CORE
  #if CORE_IS_XY
    #define CORE_AXIS_1 A_AXIS
    #define CORE_AXIS_2 B_AXIS
    #define NORMAL_AXIS Z_AXIS
  #elif CORE_IS_XZ
    #define CORE_AXIS_1 A_AXIS
    #define NORMAL_AXIS Y_AXIS
    #define CORE_AXIS_2 C_AXIS
  #elif CORE_IS_YZ
    #define NORMAL_AXIS X_AXIS
    #define CORE_AXIS_1 B_AXIS
    #define CORE_AXIS_2 C_AXIS
  #endif
  #if (MECH(COREXY) || MECH(COREXZ) || MECH(COREYZ))
    #define CORESIGN(n) (n)
  #else
    #define CORESIGN(n) (-(n))
  #endif
#endif

 /**
 * Set the home position based on settings or manual overrides
 */
#if ENABLED(MANUAL_X_HOME_POS)
  #define X_HOME_POS MANUAL_X_HOME_POS
#elif ENABLED(BED_CENTER_AT_0_0)
  #define X_HOME_POS ((X_MAX_LENGTH) * (X_HOME_DIR) * 0.5)
#else
  #define X_HOME_POS (X_HOME_DIR < 0 ? X_MIN_POS : X_MAX_POS)
#endif

#if ENABLED(MANUAL_Y_HOME_POS)
  #define Y_HOME_POS MANUAL_Y_HOME_POS
#elif ENABLED(BED_CENTER_AT_0_0)
  #define Y_HOME_POS ((Y_MAX_LENGTH) * (Y_HOME_DIR) * 0.5)
#else
  #define Y_HOME_POS (Y_HOME_DIR < 0 ? Y_MIN_POS : Y_MAX_POS)
#endif

#if ENABLED(MANUAL_Z_HOME_POS)
  #define Z_HOME_POS MANUAL_Z_HOME_POS
#else
  #define Z_HOME_POS (Z_HOME_DIR < 0 ? Z_MIN_POS : Z_MAX_POS)
#endif

/**
 * Auto Bed Leveling and Z Probe Repeatability Test
 */
#define HOMING_Z_WITH_PROBE (HAS_BED_PROBE && Z_HOME_DIR < 0 && DISABLED(Z_TWO_ENDSTOPS))

/**
 * Safe Homing Options
 */
#if ENABLED(Z_SAFE_HOMING)
  #if DISABLED(Z_SAFE_HOMING_X_POINT)
    #define Z_SAFE_HOMING_X_POINT ((X_MIN_POS + X_MAX_POS) / 2)
  #endif
  #if DISABLED(Z_SAFE_HOMING_Y_POINT)
    #define Z_SAFE_HOMING_Y_POINT ((Y_MIN_POS + Y_MAX_POS) / 2)
  #endif
  #define X_TILT_FULCRUM Z_SAFE_HOMING_X_POINT
  #define Y_TILT_FULCRUM Z_SAFE_HOMING_Y_POINT
#else
  #define X_TILT_FULCRUM X_HOME_POS
  #define Y_TILT_FULCRUM Y_HOME_POS
#endif

/**
 * Host keep alive
 */
#if DISABLED(DEFAULT_KEEPALIVE_INTERVAL)
  #define DEFAULT_KEEPALIVE_INTERVAL 2
#endif


/**
 * SPI_SPEED
 */
#if ENABLED(SD_HALF_SPEED)
  #define SPI_SPEED SPI_HALF_SPEED
#elif ENABLED(SD_QUARTER_SPEED)
  #define SPI_SPEED SPI_QUARTER_SPEED
#elif ENABLED(SD_EIGHTH_SPEED)
  #define SPI_SPEED SPI_EIGHTH_SPEED
#elif ENABLED(SD_SIXTEENTH_SPEED)
  #define SPI_SPEED SPI_SIXTEENTH_SPEED
#else
  #define SPI_SPEED SPI_FULL_SPEED
#endif

/**
 * SD DETECT
 */
#if ENABLED(SD_DISABLED_DETECT)
  #undef SD_DETECT_PIN
  #define SD_DETECT_PIN   -1
#endif
#if ENABLED(ULTIPANEL)
  #undef SD_DETECT_INVERTED
#endif

/**
 * Power Signal Control Definitions
 * By default use Normal definition
 */
#if DISABLED(POWER_SUPPLY)
  #define POWER_SUPPLY 0
#endif
#if (POWER_SUPPLY == 1)     // 1 = ATX
  #define PS_ON_AWAKE  LOW
  #define PS_ON_ASLEEP HIGH
#elif (POWER_SUPPLY == 2)   // 2 = X-Box 360 203W
  #define PS_ON_AWAKE  HIGH
  #define PS_ON_ASLEEP LOW
#endif
#if DISABLED(DELAY_AFTER_POWER_ON)
  #define DELAY_AFTER_POWER_ON 5
#endif
#define HAS_POWER_SWITCH (POWER_SUPPLY > 0 && PIN_EXISTS(PS_ON))

/**
 * Temp Sensor defines
 */
#if   TEMP_SENSOR_0 == -4 || TEMP_SENSOR_1 == -4 || TEMP_SENSOR_2 == -4 || TEMP_SENSOR_3 == -4
  #define SUPPORT_MAX31855
#elif TEMP_SENSOR_0 == -3 || TEMP_SENSOR_1 == -3 || TEMP_SENSOR_2 == -3 || TEMP_SENSOR_3 == -3
  #define SUPPORT_MAX6675
#elif TEMP_SENSOR_0 == -2 || TEMP_SENSOR_1 == -2 || TEMP_SENSOR_2 == -2 || TEMP_SENSOR_3 == -2 || TEMP_SENSOR_BED == -2 || TEMP_SENSOR_CHAMBER == -2
  #define SUPPORT_AD8495
#elif TEMP_SENSOR_0 == -1 || TEMP_SENSOR_1 == -1 || TEMP_SENSOR_2 == -1 || TEMP_SENSOR_3 == -1 || TEMP_SENSOR_BED == -1 || TEMP_SENSOR_CHAMBER == -1
  #define SUPPORT_AD595
#elif TEMP_SENSOR_0 == 20 || TEMP_SENSOR_1 == 20 || TEMP_SENSOR_2 == 20 || TEMP_SENSOR_3 == 20 || TEMP_SENSOR_BED == 20 || TEMP_SENSOR_CHAMBER == 20
  #define SUPPORT_AMPLIFIER
#endif

/**
 * Shorthand for pin tests, used wherever needed
 */

// EXTRUDERS
#define HAS_EXTRUDERS       (EXTRUDERS > 0)

// Steppers
#define HAS_X_ENABLE        (PIN_EXISTS(X_ENABLE))
#define HAS_X_DIR           (PIN_EXISTS(X_DIR))
#define HAS_X_STEP          (PIN_EXISTS(X_STEP))

#define HAS_X2_ENABLE       (PIN_EXISTS(X2_ENABLE))
#define HAS_X2_DIR          (PIN_EXISTS(X2_DIR))
#define HAS_X2_STEP         (PIN_EXISTS(X2_STEP))

#define HAS_Y_ENABLE        (PIN_EXISTS(Y_ENABLE))
#define HAS_Y_DIR           (PIN_EXISTS(Y_DIR))
#define HAS_Y_STEP          (PIN_EXISTS(Y_STEP))

#define HAS_Y2_ENABLE       (PIN_EXISTS(Y2_ENABLE))
#define HAS_Y2_DIR          (PIN_EXISTS(Y2_DIR))
#define HAS_Y2_STEP         (PIN_EXISTS(Y2_STEP))

#define HAS_Z_ENABLE        (PIN_EXISTS(Z_ENABLE))
#define HAS_Z_DIR           (PIN_EXISTS(Z_DIR))
#define HAS_Z_STEP          (PIN_EXISTS(Z_STEP))

#define HAS_Z2_ENABLE       (PIN_EXISTS(Z2_ENABLE))
#define HAS_Z2_DIR          (PIN_EXISTS(Z2_DIR))
#define HAS_Z2_STEP         (PIN_EXISTS(Z2_STEP))

// Extruder steppers and solenoids
#define HAS_E0_ENABLE       (PIN_EXISTS(E0_ENABLE))
#define HAS_E0_DIR          (PIN_EXISTS(E0_DIR))
#define HAS_E0_STEP         (PIN_EXISTS(E0_STEP))

#define HAS_E1_ENABLE       (PIN_EXISTS(E1_ENABLE))
#define HAS_E1_DIR          (PIN_EXISTS(E1_DIR))
#define HAS_E1_STEP         (PIN_EXISTS(E1_STEP))

#define HAS_E2_ENABLE       (PIN_EXISTS(E2_ENABLE))
#define HAS_E2_DIR          (PIN_EXISTS(E2_DIR))
#define HAS_E2_STEP         (PIN_EXISTS(E2_STEP))

#define HAS_E3_ENABLE       (PIN_EXISTS(E3_ENABLE))
#define HAS_E3_DIR          (PIN_EXISTS(E3_DIR))
#define HAS_E3_STEP         (PIN_EXISTS(E3_STEP))

#define HAS_E4_ENABLE       (PIN_EXISTS(E4_ENABLE))
#define HAS_E4_DIR          (PIN_EXISTS(E4_DIR))
#define HAS_E4_STEP         (PIN_EXISTS(E4_STEP))

#define HAS_E5_ENABLE       (PIN_EXISTS(E5_ENABLE))
#define HAS_E5_DIR          (PIN_EXISTS(E5_DIR))
#define HAS_E5_STEP         (PIN_EXISTS(E5_STEP))


// Extruder Encoder
#define HAS_E0_ENC          (PIN_EXISTS(E0_ENC))
#define HAS_E1_ENC          (PIN_EXISTS(E1_ENC))
#define HAS_E2_ENC          (PIN_EXISTS(E2_ENC))
#define HAS_E3_ENC          (PIN_EXISTS(E3_ENC))
#define HAS_E4_ENC          (PIN_EXISTS(E4_ENC))
#define HAS_E5_ENC          (PIN_EXISTS(E5_ENC))


// Endstops and bed probe
#define HAS_X_MIN           (PIN_EXISTS(X_MIN))
#define HAS_X_MAX           (PIN_EXISTS(X_MAX))
#define HAS_Y_MIN           (PIN_EXISTS(Y_MIN))
#define HAS_Y_MAX           (PIN_EXISTS(Y_MAX))
#define HAS_Z_MIN           (PIN_EXISTS(Z_MIN))
#define HAS_Z_MAX           (PIN_EXISTS(Z_MAX))
#define HAS_X2_MIN          (PIN_EXISTS(X2_MIN))
#define HAS_Y2_MIN          (PIN_EXISTS(Y2_MIN))
#define HAS_Z2_MIN          (PIN_EXISTS(Z2_MIN))
#define HAS_X2_MAX          (PIN_EXISTS(X2_MAX))
#define HAS_Y2_MAX          (PIN_EXISTS(Y2_MAX))
#define HAS_Z2_MAX          (PIN_EXISTS(Z2_MAX))
#define HAS_Z_PROBE_PIN     (PIN_EXISTS(Z_PROBE))

// Multi endstop
#define HAS_MULTI_ENDSTOP   (ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS))

// Utility
#define HAS_DOOR_OPEN       (ENABLED(DOOR_OPEN_FEATURE) && PIN_EXISTS(DOOR_OPEN))

// Thermistors
#define HAS_TEMP_0          (TEMP_SENSOR_0 != 0)
#define HAS_TEMP_1          (TEMP_SENSOR_1 != 0)
#define HAS_TEMP_2          (TEMP_SENSOR_2 != 0)
#define HAS_TEMP_3          (TEMP_SENSOR_3 != 0)
#define HAS_TEMP_HOTEND     (HAS_TEMP_0 || ENABLED(SUPPORT_MAX6675) || ENABLED(SUPPORT_MAX31855))
#define HAS_TEMP_BED        (TEMP_SENSOR_BED != 0)
#define HAS_TEMP_CHAMBER    (TEMP_SENSOR_CHAMBER != 0)
#define HAS_TEMP_HEATER     (HAS_TEMP_HOTEND || HAS_TEMP_BED || HAS_TEMP_CHAMBER)
#define HAS_MCU_TEMPERATURE (ENABLED(HAVE_MCU_TEMPERATURE))

// Thermocouples
#define HAS_MAX6675_SS      (PIN_EXISTS(MAX6675_SS))
#define HAS_MAX31855_SS0    (PIN_EXISTS(MAX31855_SS0))
#define HAS_MAX31855_SS1    (PIN_EXISTS(MAX31855_SS1))
#define HAS_MAX31855_SS2    (PIN_EXISTS(MAX31855_SS2))
#define HAS_MAX31855_SS3    (PIN_EXISTS(MAX31855_SS3))

// Heaters
#define HAS_HEATER_0        (HOTENDS > 0 && PIN_EXISTS(HEATER_0))
#define HAS_HEATER_1        (HOTENDS > 1 && PIN_EXISTS(HEATER_1))
#define HAS_HEATER_2        (HOTENDS > 2 && PIN_EXISTS(HEATER_2))
#define HAS_HEATER_3        (HOTENDS > 3 && PIN_EXISTS(HEATER_3))
#define HAS_HEATER_BED      (TEMP_SENSOR_BED != 0 && PIN_EXISTS(HEATER_BED))
#define HAS_HEATER_CHAMBER  (TEMP_SENSOR_CHAMBER != 0 && PIN_EXISTS(HEATER_CHAMBER))

// Fans
#define HAS_FAN0            (PIN_EXISTS(FAN0))
#define HAS_FAN1            (PIN_EXISTS(FAN1))
#define HAS_FAN2            (PIN_EXISTS(FAN2))
#define HAS_FAN3            (PIN_EXISTS(FAN3))
#define HAS_FAN4            (PIN_EXISTS(FAN4))
#define HAS_FAN5            (PIN_EXISTS(FAN5))

// Servos
#define HAS_SERVO_0         (PIN_EXISTS(SERVO0))
#define HAS_SERVO_1         (PIN_EXISTS(SERVO1))
#define HAS_SERVO_2         (PIN_EXISTS(SERVO2))
#define HAS_SERVO_3         (PIN_EXISTS(SERVO3))
#define HAS_SERVOS          ((ENABLED(ENABLE_SERVOS) && NUM_SERVOS > 0) && (HAS_SERVO_0 || HAS_SERVO_1 || HAS_SERVO_2 || HAS_SERVO_3))

// Sensors
#define HAS_FIL_RUNOUT_0              (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FIL_RUNOUT_0))
#define HAS_FIL_RUNOUT_1              (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FIL_RUNOUT_1))
#define HAS_FIL_RUNOUT_2              (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FIL_RUNOUT_2))
#define HAS_FIL_RUNOUT_3              (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FIL_RUNOUT_3))
#define HAS_FIL_RUNOUT_4              (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FIL_RUNOUT_4))
#define HAS_FIL_RUNOUT_5              (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FIL_RUNOUT_5))

// User Interface
#define HAS_BTN_BACK        (PIN_EXISTS(BTN_BACK))
#define HAS_HOME            (PIN_EXISTS(HOME))
#define HAS_KILL            (PIN_EXISTS(KILL))
#define HAS_SUICIDE         (PIN_EXISTS(SUICIDE))
#define HAS_CHDK            (ENABLED(CHDK) && PIN_EXISTS(CHDK))
#define HAS_PHOTOGRAPH      (ENABLED(PHOTOGRAPH) && PIN_EXISTS(PHOTOGRAPH))
#define HAS_BUZZER          (PIN_EXISTS(BEEPER) || ENABLED(LCD_USE_I2C_BUZZER))
#define HAS_CASE_LIGHT      (ENABLED(CASE_LIGHT) && (PIN_EXISTS(CASE_LIGHT) || ENABLED(CASE_LIGHT_USE_NEOPIXEL)))
#define HAS_RESUME_CONTINUE (HAS_LCD || ENABLED(EMERGENCY_PARSER))

// MK Multi tool system
#define HAS_MKMULTI_TOOLS   (ENABLED(MKSE6) || ENABLED(MKR4) || ENABLED(MKR6) || ENABLED(MKR12))

// MKR4 or MKR6 or MKR12
#define HAS_E0E1            (PIN_EXISTS(E0E1_CHOICE))
#define HAS_E0E2            (PIN_EXISTS(E0E2_CHOICE))
#define HAS_E1E3            (PIN_EXISTS(E1E3_CHOICE))
#define HAS_EX1             (PIN_EXISTS(EX1_CHOICE))
#define HAS_EX2             (PIN_EXISTS(EX2_CHOICE))

// RGB leds
#define HAS_COLOR_LEDS      (ENABLED(BLINKM) || ENABLED(RGB_LED) || ENABLED(RGBW_LED) || ENABLED(PCA9632) || ENABLED(NEOPIXEL_LED))
#define HAS_LEDS_OFF_FLAG   (ENABLED(PRINTER_EVENT_LEDS) && HAS_SD_SUPPORT && HAS_RESUME_CONTINUE)

// EEPROM support
#if ENABLED(EEPROM_FLASH)
  #undef EEPROM_SPI
  #undef EEPROM_I2C
  #undef EEPROM_SD
#elif ENABLED(EEPROM_SD)
  #undef EEPROM_SPI
  #undef EEPROM_I2C
  #undef EEPROM_FLASH
#endif
#define HAS_EEPROM_SPI      (ENABLED(EEPROM_SETTINGS) && ENABLED(EEPROM_SPI))
#define HAS_EEPROM_I2C      (ENABLED(EEPROM_SETTINGS) && ENABLED(EEPROM_I2C))
#define HAS_EEPROM_SD       (ENABLED(EEPROM_SETTINGS) && ENABLED(EEPROM_SD) && ENABLED(SDSUPPORT))
#define HAS_EEPROM_FLASH    (ENABLED(EEPROM_SETTINGS) && ENABLED(EEPROM_FLASH))
#define HAS_EEPROM          (ENABLED(EEPROM_SETTINGS))  // Do not touch, AVR have not define anyone EEPROM.

// SD support
#define HAS_SD_SUPPORT      (ENABLED(SDSUPPORT) || ENABLED(USB_FLASH_DRIVE_SUPPORT))
#if HAS_SD_SUPPORT
  #define SD_MAX_FOLDER_DEPTH 5     // Maximum folder depth
  // Number of VFAT entries used. Each entry has 13 UTF-16 characters
  #if ENABLED(SCROLL_LONG_FILENAMES)
    #define MAX_VFAT_ENTRIES (5)
  #else
    #define MAX_VFAT_ENTRIES (2)
  #endif
  #define FILENAME_LENGTH 13
  /** Total size of the buffer used to store the long filenames */
  #define LONG_FILENAME_LENGTH  (FILENAME_LENGTH * MAX_VFAT_ENTRIES + 1)
  #define MAX_PATH_NAME_LENGHT  (LONG_FILENAME_LENGTH * SD_MAX_FOLDER_DEPTH + SD_MAX_FOLDER_DEPTH + 1)
  #define SHORT_FILENAME_LENGTH 14
  #define GENBY_SIZE 16
#else
  #undef SCROLL_LONG_FILENAMES
  #undef SDCARD_SORT_ALPHA
#endif
#if HAS_SD_SUPPORT && ENABLED(ARDUINO_ARCH_SAM)
  #undef SDCARD_SORT_ALPHA
  #undef SDSORT_LIMIT
  #undef SDSORT_GCODE
  #undef SDSORT_USES_RAM
  #undef SDSORT_USES_STACK
  #undef SDSORT_CACHE_NAMES
  #undef SDSORT_DYNAMIC_RAM
  #define SDCARD_SORT_ALPHA
  #define SDSORT_LIMIT 256
  #define SDSORT_GCODE true
  #define SDSORT_USES_RAM false
  #define SDSORT_USES_STACK false
  #define SDSORT_CACHE_NAMES false
  #define SDSORT_DYNAMIC_RAM false
#endif
#if ENABLED(SDCARD_SORT_ALPHA)
  #define HAS_FOLDER_SORTING  (FOLDER_SORTING || ENABLED(SDSORT_GCODE))
#endif
#define HAS_SD_RESTART        (HAS_SD_SUPPORT && ENABLED(SD_RESTART_FILE))

// Extruder Encoder
#define HAS_EXT_ENCODER       (ENABLED(EXTRUDER_ENCODER_CONTROL) && (HAS_E0_ENC || HAS_E1_ENC || HAS_E2_ENC || HAS_E3_ENC || HAS_E4_ENC || HAS_E5_ENC))

// Other
#define HAS_SOFTWARE_ENDSTOPS (ENABLED(MIN_SOFTWARE_ENDSTOPS) || ENABLED(MAX_SOFTWARE_ENDSTOPS))

/**
 * ENDSTOPPULLUPS
 */
#if DISABLED(ENDSTOPPULLUP_XMIN)
  #define ENDSTOPPULLUP_XMIN    false
#endif
#if DISABLED(ENDSTOPPULLUP_YMIN)
  #define ENDSTOPPULLUP_YMIN    false
#endif
#if DISABLED(ENDSTOPPULLUP_ZMIN)
  #define ENDSTOPPULLUP_ZMIN    false
#endif
#if DISABLED(ENDSTOPPULLUP_XMAX)
  #define ENDSTOPPULLUP_XMAX    false
#endif
#if DISABLED(ENDSTOPPULLUP_YMAX)
  #define ENDSTOPPULLUP_YMAX    false
#endif
#if DISABLED(ENDSTOPPULLUP_ZMAX)
  #define ENDSTOPPULLUP_ZMAX    false
#endif
#if DISABLED(ENDSTOPPULLUP_X2MIN)
  #define ENDSTOPPULLUP_X2MIN   false
#endif
#if DISABLED(ENDSTOPPULLUP_Y2MIN)
  #define ENDSTOPPULLUP_Y2MIN   false
#endif
#if DISABLED(ENDSTOPPULLUP_Z2MIN)
  #define ENDSTOPPULLUP_Z2MIN   false
#endif
#if DISABLED(ENDSTOPPULLUP_X2MAX)
  #define ENDSTOPPULLUP_X2MAX   false
#endif
#if DISABLED(ENDSTOPPULLUP_Y2MAX)
  #define ENDSTOPPULLUP_Y2MAX   false
#endif
#if DISABLED(ENDSTOPPULLUP_Z2MAX)
  #define ENDSTOPPULLUP_Z2MAX   false
#endif
#if DISABLED(ENDSTOPPULLUP_ZPROBE)
  #define ENDSTOPPULLUP_ZPROBE  false
#endif

#if DISABLED(X_MIN_ENDSTOP_LOGIC)
  #define X_MIN_ENDSTOP_LOGIC   false
#endif
#if DISABLED(Y_MIN_ENDSTOP_LOGIC)
  #define Y_MIN_ENDSTOP_LOGIC   false
#endif
#if DISABLED(Z_MIN_ENDSTOP_LOGIC)
  #define Z_MIN_ENDSTOP_LOGIC   false
#endif
#if DISABLED(X_MAX_ENDSTOP_LOGIC)
  #define X_MAX_ENDSTOP_LOGIC   false
#endif
#if DISABLED(Y_MAX_ENDSTOP_LOGIC)
  #define Y_MAX_ENDSTOP_LOGIC   false
#endif
#if DISABLED(Z_MAX_ENDSTOP_LOGIC)
  #define Z_MAX_ENDSTOP_LOGIC   false
#endif
#if DISABLED(X2_MIN_ENDSTOP_LOGIC)
  #define X2_MIN_ENDSTOP_LOGIC  false
#endif
#if DISABLED(Y2_MIN_ENDSTOP_LOGIC)
  #define Y2_MIN_ENDSTOP_LOGIC  false
#endif
#if DISABLED(Z2_MIN_ENDSTOP_LOGIC)
  #define Z2_MIN_ENDSTOP_LOGIC  false
#endif
#if DISABLED(X2_MAX_ENDSTOP_LOGIC)
  #define X2_MAX_ENDSTOP_LOGIC  false
#endif
#if DISABLED(Y2_MAX_ENDSTOP_LOGIC)
  #define Y2_MAX_ENDSTOP_LOGIC  false
#endif
#if DISABLED(Z2_MAX_ENDSTOP_LOGIC)
  #define Z2_MAX_ENDSTOP_LOGIC  false
#endif
#if DISABLED(Z_PROBE_ENDSTOP_LOGIC)
  #define Z_PROBE_ENDSTOP_LOGIC false
#endif

/**
 * JERK or JUNCTION_DEVIATION
 */
#define HAS_CLASSIC_JERK        (IS_KINEMATIC || DISABLED(JUNCTION_DEVIATION))

/**
 * Set granular options based on the specific type of leveling
 */
#define ABL_PLANAR              (ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(AUTO_BED_LEVELING_3POINT))
#define ABL_GRID                (ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(AUTO_BED_LEVELING_BILINEAR))
#define OLD_ABL                 (ABL_PLANAR || ABL_GRID)
#define HAS_ABL                 (OLD_ABL || ENABLED(AUTO_BED_LEVELING_UBL))
#define HAS_LEVELING            (HAS_ABL || ENABLED(MESH_BED_LEVELING))
#define HAS_AUTOLEVEL           (HAS_ABL && DISABLED(PROBE_MANUALLY))
#define HAS_MESH                (ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(MESH_BED_LEVELING))
#define PLANNER_LEVELING        (HAS_LEVELING && DISABLED(AUTO_BED_LEVELING_UBL))
#define HAS_PROBING_PROCEDURE   (HAS_ABL || ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST))
#define HAS_POSITION_MODIFIERS  (ENABLED(FWRETRACT) || HAS_LEVELING)

#if ENABLED(AUTO_BED_LEVELING_UBL)
  #undef LCD_BED_LEVELING
#endif

/**
 * Position Float
 */
#define HAS_POSITION_FLOAT      ENABLED(LIN_ADVANCE)

/**
 * Bed Probing rectangular bounds
  */
// Boundaries for Cartesian probing based on bed limits
#define MIN_PROBE_X (MAX(X_MIN_BED + (MIN_PROBE_EDGE), X_MIN_POS + probe.data.offset[X_AXIS]))
#define MIN_PROBE_Y (MAX(Y_MIN_BED + (MIN_PROBE_EDGE), Y_MIN_POS + probe.data.offset[Y_AXIS]))
#define MAX_PROBE_X (MIN(X_MAX_BED - (MIN_PROBE_EDGE), X_MAX_POS + probe.data.offset[X_AXIS]))
#define MAX_PROBE_Y (MIN(Y_MAX_BED - (MIN_PROBE_EDGE), Y_MAX_POS + probe.data.offset[Y_AXIS]))

/**
 * Set GRID MAX POINTS
 */
#if ENABLED(GRID_MAX_POINTS_X) && ENABLED(GRID_MAX_POINTS_Y)
  #define GRID_MAX_POINTS ((GRID_MAX_POINTS_X) * (GRID_MAX_POINTS_Y))
#endif

/**
 * Z probe repetitions
 */
#if DISABLED(Z_PROBE_REPETITIONS)
  #define Z_PROBE_REPETITIONS 1
#endif

/**
 * Heaters
 */
#if HAS_HEATER_0
  #define HOT0_COUNT 1
  #define HOT0_INDEX 0
  #define HOT0_CHANNEL  HEATER_0_PIN
  #define HOT0_COMMA ,
#else
  #define HOT0_COUNT 0
  #define HOT0_INDEX
  #define HOT0_CHANNEL
  #define HOT0_COMMA
#endif
#if HAS_HEATER_1
  #define HOT1_COUNT 1
  #define HOT1_INDEX    HOT0_COUNT
  #define HOT1_CHANNEL  HOT0_COMMA HEATER_1_PIN
  #define HOT1_COMMA ,
#else
  #define HOT1_COUNT 0
  #define HOT1_INDEX
  #define HOT1_CHANNEL
  #define HOT1_COMMA    HOT0_COMMA
#endif
#if HAS_HEATER_2
  #define HOT2_COUNT 1
  #define HOT2_INDEX    HOT0_COUNT+HOT1_COUNT
  #define HOT2_CHANNEL  HOT1_COMMA HEATER_2_PIN
  #define HOT2_COMMA ,
#else
  #define HOT2_COUNT 0
  #define HOT2_INDEX
  #define HOT2_CHANNEL
  #define HOT2_COMMA    HOT1_COMMA
#endif
#if HAS_HEATER_3
  #define HOT3_COUNT 1
  #define HOT3_INDEX    HOT0_COUNT+HOT1_COUNT+HOT2_COUNT
  #define HOT3_CHANNEL  HOT2_COMMA HEATER_3_PIN
  #define HOT3_COMMA ,
#else
  #define HOT3_COUNT 0
  #define HOT3_INDEX
  #define HOT3_CHANNEL
  #define HOT3_COMMA    HOT2_COMMA
#endif
#if HAS_HEATER_BED
  #define BED_COUNT 1
  #define BED_INDEX     HOT0_COUNT+HOT1_COUNT+HOT2_COUNT+HOT3_COUNT
  #define BED_CHANNEL   HOT3_COMMA HEATER_BED_PIN
  #define BED_COMMA ,
#else
  #define BED_COUNT 0
  #define BED_INDEX
  #define BED_CHANNEL
  #define BED_COMMA     HOT3_COMMA
#endif
#if HAS_HEATER_CHAMBER
  #define CHAMBER_COUNT 1
  #define CHAMBER_INDEX   HOT0_COUNT+HOT1_COUNT+HOT2_COUNT+HOT3_COUNT+BED_COUNT
  #define CHAMBER_CHANNEL BED_COMMA HEATER_CHAMBER_PIN
  #define CHAMBER_COMMA ,
#else
  #define CHAMBER_COUNT 0
  #define CHAMBER_INDEX
  #define CHAMBER_CHANNEL
  #define CHAMBER_COMMA   BED_COMMA
#endif

#define HEATER_COUNT  (HOT0_COUNT+HOT1_COUNT+HOT2_COUNT+HOT3_COUNT+BED_COUNT+CHAMBER_COUNT)
#define HEATER_TYPE 4
#if HEATER_COUNT > 0
  #define HEATER_CHANNELS {HOT0_CHANNEL HOT1_CHANNEL HOT2_CHANNEL HOT3_CHANNEL BED_CHANNEL CHAMBER_CHANNEL}
#else
  #define HEATER_CHANNELS { }
#endif

#if INVERTED_HEATER_PINS
  #define WRITE_HEATER(pin, value) WRITE(pin, !value)
#else
  #define WRITE_HEATER(pin, value) WRITE(pin, value)
#endif
#if INVERTED_BED_PIN
  #define WRITE_HEATER_BED(v) WRITE(HEATER_BED_PIN,!v)
#else
  #define WRITE_HEATER_BED(v) WRITE(HEATER_BED_PIN,v)
#endif
#if INVERTED_CHAMBER_PIN
  #define WRITE_HEATER_CHAMBER(v) WRITE(HEATER_CHAMBER_PIN,!v)
#else
  #define WRITE_HEATER_CHAMBER(v) WRITE(HEATER_CHAMBER_PIN,v)
#endif

/**
 * FANS
 */
#if HAS_FAN0
  #define FAN0_COUNT 1
  #define FAN0_INDEX 0
  #define FAN0_CHANNEL FAN0_PIN
  #define FAN0_COMMA ,
#else
  #define FAN0_COUNT 0
  #define FAN0_INDEX
  #define FAN0_CHANNEL
  #define FAN0_COMMA
#endif

#if HAS_FAN1
  #define FAN1_COUNT 1
  #define FAN1_INDEX FAN0_COUNT
  #define FAN1_CHANNEL FAN0_COMMA FAN1_PIN
  #define FAN1_COMMA ,
#else
  #define FAN1_COUNT 0
  #define FAN1_INDEX
  #define FAN1_CHANNEL
  #define FAN1_COMMA FAN0_COMMA
#endif

#if HAS_FAN2
  #define FAN2_COUNT 1
  #define FAN2_INDEX FAN0_COUNT+FAN1_COUNT
  #define FAN2_CHANNEL FAN1_COMMA FAN2_PIN
  #define FAN2_COMMA ,
#else
  #define FAN2_COUNT 0
  #define FAN2_INDEX
  #define FAN2_CHANNEL
  #define FAN2_COMMA FAN1_COMMA
#endif

#if HAS_FAN3
  #define FAN3_COUNT 1
  #define FAN3_INDEX FAN0_COUNT+FAN1_COUNT+FAN2_COUNT
  #define FAN3_CHANNEL FAN2_COMMA FAN3_PIN
  #define FAN3_COMMA ,
#else
  #define FAN3_COUNT 0
  #define FAN3_INDEX
  #define FAN3_CHANNEL
  #define FAN3_COMMA FAN2_COMMA
#endif

#if HAS_FAN4
  #define FAN4_COUNT 1
  #define FAN4_INDEX FAN0_COUNT+FAN1_COUNT+FAN2_COUNT+FAN3_COUNT
  #define FAN4_CHANNEL FAN3_COMMA FAN4_PIN
  #define FAN4_COMMA ,
#else
  #define FAN4_COUNT 0
  #define FAN4_INDEX
  #define FAN4_CHANNEL
  #define FAN4_COMMA FAN3_COMMA
#endif

#if HAS_FAN5
  #define FAN5_COUNT 1
  #define FAN5_INDEX FAN0_COUNT+FAN1_COUNT+FAN2_COUNT+FAN3_COUNT+FAN4_COUNT
  #define FAN5_CHANNEL FAN4_COMMA FAN5_PIN
  #define FAN5_COMMA ,
#else
  #define FAN5_COUNT 0
  #define FAN5_INDEX
  #define FAN5_CHANNEL
  #define FAN5_COMMA FAN4_COMMA
#endif

#define FAN_COUNT       (FAN0_COUNT+FAN1_COUNT+FAN2_COUNT+FAN3_COUNT+FAN4_COUNT+FAN5_COUNT)

#if FAN_COUNT > 0
  #define FANS_CHANNELS { FAN0_CHANNEL FAN1_CHANNEL FAN2_CHANNEL FAN3_CHANNEL FAN4_CHANNEL FAN5_CHANNEL }
#else
  #define FANS_CHANNELS { }
#endif

#if ENABLED(INVERTED_FAN_PINS)
  #define FAN_INVERTED true
#else
  #define FAN_INVERTED false
#endif

/**
 * Extruder Encoder
 */
#if HAS_EXT_ENCODER
  #if ENABLED(INVERTED_ENCODER_PINS)
    #define READ_ENCODER(v) !HAL::digitalRead(v)
  #else
    #define READ_ENCODER(v) HAL::digitalRead(v)
  #endif
#endif

/**
 * Heater & Fan Pausing
 */
#if FAN_COUNT == 0
  #undef PROBING_FANS_OFF
#endif
#define QUIET_PROBING       (HAS_BED_PROBE && (ENABLED(PROBING_HEATERS_OFF) || ENABLED(PROBING_FANS_OFF)))


/**
 * Multiextruder with relè system
 */
#if ENABLED(MKR4) || ENABLED(MKR6) || ENABLED(MKR12)
  #if ENABLED(INVERTED_RELE_PINS)
    #define WRITE_RELE(pin, value) WRITE(pin, !value)
    #define OUT_WRITE_RELE(pin, value) OUT_WRITE(pin, !value)
  #else
    #define WRITE_RELE(pin, value) WRITE(pin, value)
    #define OUT_WRITE_RELE(pin, value) OUT_WRITE(pin, value)
  #endif
#endif

/**
 * Buzzer/Speaker
 */
#if ENABLED(LCD_USE_I2C_BUZZER)
  #if DISABLED(LCD_FEEDBACK_FREQUENCY_HZ)
    #define LCD_FEEDBACK_FREQUENCY_HZ 1000
  #endif
  #if DISABLED(LCD_FEEDBACK_FREQUENCY_DURATION_MS)
    #define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100
  #endif
#else
  #if DISABLED(LCD_FEEDBACK_FREQUENCY_HZ)
    #define LCD_FEEDBACK_FREQUENCY_HZ 5000
  #endif
  #if DISABLED(LCD_FEEDBACK_FREQUENCY_DURATION_MS)
    #define LCD_FEEDBACK_FREQUENCY_DURATION_MS 10
  #endif
#endif

/**
 * MIN_Z_HEIGHT_FOR_HOMING / Z_PROBE_BETWEEN_HEIGHT
 */
#if DISABLED(MIN_Z_HEIGHT_FOR_HOMING)
  #define MIN_Z_HEIGHT_FOR_HOMING 0
#endif
#if DISABLED(Z_PROBE_BETWEEN_HEIGHT)
  #define Z_PROBE_BETWEEN_HEIGHT MIN_Z_HEIGHT_FOR_HOMING
#endif
#if Z_PROBE_BETWEEN_HEIGHT > MIN_Z_HEIGHT_FOR_HOMING
  #define MANUAL_PROBE_HEIGHT Z_PROBE_BETWEEN_HEIGHT
#else
  #define MANUAL_PROBE_HEIGHT MIN_Z_HEIGHT_FOR_HOMING
#endif

/**
 * Servos
 */
#if HAS_SERVOS
  #if DISABLED(Z_PROBE_SERVO_NR)
    #define Z_PROBE_SERVO_NR -1
  #endif
#endif

/**
 * Set a flag for a servo probe
 */
#define HAS_Z_SERVO_PROBE (HAS_SERVOS && ENABLED(Z_PROBE_SERVO_NR) && Z_PROBE_SERVO_NR >= 0)

/**
 * Set flags for enabled probes
 */
#define HAS_BED_PROBE         (ENABLED(Z_PROBE_FIX_MOUNTED) || ENABLED(Z_PROBE_ALLEN_KEY) || HAS_Z_SERVO_PROBE)
#define PROBE_SELECTED        (HAS_BED_PROBE || ENABLED(PROBE_MANUALLY))
#define PROBE_PIN_CONFIGURED  (HAS_Z_PROBE_PIN || HAS_Z_MIN)

/**
 * Bed Probe dependencies
 */
#if HAS_BED_PROBE
  #if DISABLED(X_PROBE_OFFSET_FROM_NOZZLE)
    #define X_PROBE_OFFSET_FROM_NOZZLE 0
  #endif
  #if DISABLED(Y_PROBE_OFFSET_FROM_NOZZLE)
    #define Y_PROBE_OFFSET_FROM_NOZZLE 0
  #endif
  #if DISABLED(Z_PROBE_OFFSET_FROM_NOZZLE)
    #define Z_PROBE_OFFSET_FROM_NOZZLE 0
  #endif
  #if DISABLED(Z_PROBE_OFFSET_RANGE_MIN)
    #define Z_PROBE_OFFSET_RANGE_MIN -50
  #endif
  #if DISABLED(Z_PROBE_OFFSET_RANGE_MAX)
    #define Z_PROBE_OFFSET_RANGE_MAX 50
  #endif
  #if DISABLED(XY_PROBE_SPEED)
    #if ENABLED(HOMING_FEEDRATE_X)
      #define XY_PROBE_SPEED HOMING_FEEDRATE_X
    #elif define HOMING_FEEDRATE_XYZ
      #define XY_PROBE_SPEED HOMING_FEEDRATE_XYZ
    #else
      #define XY_PROBE_SPEED 3200
    #endif
  #endif
  #if Z_PROBE_BETWEEN_HEIGHT > Z_PROBE_DEPLOY_HEIGHT
    #define _Z_PROBE_DEPLOY_HEIGHT Z_PROBE_BETWEEN_HEIGHT
  #else
    #define _Z_PROBE_DEPLOY_HEIGHT Z_PROBE_DEPLOY_HEIGHT
  #endif
  #if DISABLED(Z_PROBE_AFTER_PROBING)
    #define Z_PROBE_AFTER_PROBING 0
  #endif
  #if DISABLED(Z_PROBE_LOW_POINT)
    #define Z_PROBE_LOW_POINT -2
  #endif
#else
  #undef X_PROBE_OFFSET_FROM_NOZZLE
  #undef Y_PROBE_OFFSET_FROM_NOZZLE
  #undef Z_PROBE_OFFSET_FROM_NOZZLE
  #define X_PROBE_OFFSET_FROM_NOZZLE 0
  #define Y_PROBE_OFFSET_FROM_NOZZLE 0
  #define Z_PROBE_OFFSET_FROM_NOZZLE 0
  #define _Z_PROBE_DEPLOY_HEIGHT (Z_MAX_POS / 2)
#endif

// Add commands that need sub-codes to this list
#define USE_GCODE_SUBCODES ENABLED(G38_PROBE_TARGET)

// PWM SPEED and MASK
#if DISABLED(SOFT_PWM_SPEED)
  #define SOFT_PWM_SPEED 0
#endif
#if SOFT_PWM_SPEED < 0
  #define SOFT_PWM_SPEED 0
#endif
#if SOFT_PWM_SPEED > 4
  #define SOFT_PWM_SPEED 4
#endif

#if SOFT_PWM_SPEED == 0
  #define SOFT_PWM_STEP 1
  #define SOFT_PWM_MASK 255
#elif SOFT_PWM_SPEED == 1
  #define SOFT_PWM_STEP 2
  #define SOFT_PWM_MASK 254
#elif SOFT_PWM_SPEED == 2
  #define SOFT_PWM_STEP 4
  #define SOFT_PWM_MASK 252
#elif SOFT_PWM_SPEED == 3
  #define SOFT_PWM_STEP 8
  #define SOFT_PWM_MASK 248
#elif SOFT_PWM_SPEED == 4
  #define SOFT_PWM_STEP 16
  #define SOFT_PWM_MASK 240
#endif

// TEMPERATURE
#if (HOTENDS > 0 && (PIN_EXISTS(TEMP_0) && TEMP_SENSOR_0 != 0 && TEMP_SENSOR_0 >= -2))
  #define HOT0_ANALOG_INPUTS        1
  #define HOT0_ANALOG_INDEX         0
  #define HOT0_ANALOG_CHANNEL       TEMP_0_PIN
  #define HOT0_ANALOG_COMMA         ,
#else
  #define HOT0_ANALOG_INPUTS        0
  #define HOT0_ANALOG_INDEX         -1
  #define HOT0_ANALOG_CHANNEL
  #define HOT0_ANALOG_COMMA
#endif

#if (HOTENDS > 1 && (PIN_EXISTS(TEMP_1) && TEMP_SENSOR_1 != 0 && TEMP_SENSOR_1 >= -2))
  #define HOT1_ANALOG_INPUTS        1
  #define HOT1_ANALOG_INDEX         HOT0_ANALOG_INPUTS
  #define HOT1_ANALOG_CHANNEL       HOT0_ANALOG_COMMA TEMP_1_PIN
  #define HOT1_ANALOG_COMMA         ,
#else
  #define HOT1_ANALOG_INPUTS        0
  #define HOT1_ANALOG_INDEX         -1
  #define HOT1_ANALOG_CHANNEL
  #define HOT1_ANALOG_COMMA         HOT0_ANALOG_COMMA
#endif

#if (HOTENDS > 2 && (PIN_EXISTS(TEMP_2) && TEMP_SENSOR_2 != 0 && TEMP_SENSOR_2 >= -2))
  #define HOT2_ANALOG_INPUTS        1
  #define HOT2_ANALOG_INDEX         HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS
  #define HOT2_ANALOG_CHANNEL       HOT1_ANALOG_COMMA TEMP_2_PIN
  #define HOT2_ANALOG_COMMA         ,
#else
  #define HOT2_ANALOG_INPUTS        0
  #define HOT2_ANALOG_INDEX         -1
  #define HOT2_ANALOG_CHANNEL
  #define HOT2_ANALOG_COMMA         HOT1_ANALOG_COMMA
#endif

#if (HOTENDS > 3 && (PIN_EXISTS(TEMP_3) && TEMP_SENSOR_3 != 0 && TEMP_SENSOR_3 >= -2))
  #define HOT3_ANALOG_INPUTS        1
  #define HOT3_ANALOG_INDEX         HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS+HOT2_ANALOG_INPUTS
  #define HOT3_ANALOG_CHANNEL       HOT2_ANALOG_COMMA TEMP_3_PIN
  #define HOT3_ANALOG_COMMA         ,
#else
  #define HOT3_ANALOG_INPUTS        0
  #define HOT3_ANALOG_INDEX         -1
  #define HOT3_ANALOG_CHANNEL
  #define HOT3_ANALOG_COMMA         HOT2_ANALOG_COMMA
#endif

#if (PIN_EXISTS(TEMP_BED) && TEMP_SENSOR_BED != 0 && TEMP_SENSOR_BED >= -2)
  #define BED_ANALOG_INPUTS         1
  #define BED_ANALOG_INDEX          HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS+HOT2_ANALOG_INPUTS+HOT3_ANALOG_INPUTS
  #define BED_ANALOG_CHANNEL        HOT3_ANALOG_COMMA TEMP_BED_PIN
  #define BED_ANALOG_COMMA          ,
#else
  #define BED_ANALOG_INPUTS         0
  #define BED_ANALOG_INDEX          -1
  #define BED_ANALOG_CHANNEL
  #define BED_ANALOG_COMMA          HOT3_ANALOG_COMMA
#endif

#if (PIN_EXISTS(TEMP_CHAMBER) && TEMP_SENSOR_CHAMBER != 0 && TEMP_SENSOR_CHAMBER >= -2)
  #define CHAMBER_ANALOG_INPUTS     1
  #define CHAMBER_ANALOG_INDEX      HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS+HOT2_ANALOG_INPUTS+HOT3_ANALOG_INPUTS+BED_ANALOG_INPUTS
  #define CHAMBER_ANALOG_CHANNEL    BED_ANALOG_COMMA TEMP_CHAMBER_PIN
  #define CHAMBER_ANALOG_COMMA      ,
#else
  #define CHAMBER_ANALOG_INPUTS     0
  #define CHAMBER_ANALOG_INDEX      -1
  #define CHAMBER_ANALOG_CHANNEL
  #define CHAMBER_ANALOG_COMMA      BED_ANALOG_COMMA
#endif


#if HAS_MCU_TEMPERATURE
  #define MCU_ANALOG_INPUTS         1
#else
  #define MCU_ANALOG_INPUTS         0
#endif

#define ANALOG_INPUTS (HOT0_ANALOG_INPUTS+HOT1_ANALOG_INPUTS+HOT2_ANALOG_INPUTS+HOT3_ANALOG_INPUTS+BED_ANALOG_INPUTS+CHAMBER_ANALOG_INPUTS+MCU_ANALOG_INPUTS)
#if ANALOG_INPUTS > 0
  /** Channels are the MUX-part of ADMUX register */
  #define ANALOG_INPUT_CHANNELS {HOT0_ANALOG_CHANNEL HOT1_ANALOG_CHANNEL HOT2_ANALOG_CHANNEL HOT3_ANALOG_CHANNEL BED_ANALOG_CHANNEL CHAMBER_ANALOG_CHANNEL}
#else
  #define ANALOG_INPUT_CHANNELS { }
#endif

// Calculate a default maximum stepper rate, if not supplied
#if DISABLED(MAXIMUM_STEPPER_RATE)
  #define MAXIMUM_STEPPER_RATE (500000UL)
#endif

/**
 * LCD define
 */

 // Get LCD character width/height, which may be overridden by pins, configs, etc.
#if DISABLED(LCD_WIDTH)
  #if HAS_GRAPHICAL_LCD
    #define LCD_WIDTH 22
  #elif ENABLED(ULTIPANEL)
    #define LCD_WIDTH 20
  #elif HAS_SPI_LCD
    #define LCD_WIDTH 16
  #endif
#endif
#if DISABLED(LCD_HEIGHT)
  #if HAS_GRAPHICAL_LCD
    #define LCD_HEIGHT 5
  #elif ENABLED(ULTIPANEL)
    #define LCD_HEIGHT 4
  #elif HAS_SPI_LCD
    #define LCD_HEIGHT 2
  #endif
#endif
