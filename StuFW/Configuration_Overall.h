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
 * Configuration_Overall.h
 * Here you can define all your custom settings and they will overwrite
 * configurations in the main configuration files.
 * Except for the configurations in Configurations_Pins.h where default
 * board pin configurations could be overriden.
 */

/*******************************
 *   Firmware Version V0.0.7   *
 *******************************/

#define CONFIGURATION_OVERALL

/***********************
 * Configuration_Basic *
 ***********************/
#define SERIAL_PORT_1 0
#define BAUDRATE_1 250000
#define SERIAL_PORT_2 -2
#define BAUDRATE_2 250000
#define STRING_CONFIG_H_AUTHOR "(none, default config)"
#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"
#define KILL_METHOD 0
#define NO_TIMEOUTS 1000
//#define ADVANCED_OK
//#define EMERGENCY_PARSER
//#define FASTER_GCODE_PARSER
//#define FASTER_GCODE_EXECUTE
#define HOST_KEEPALIVE_FEATURE
#define DEFAULT_KEEPALIVE_INTERVAL 2
// see boards.h for the board names
#define MOTHERBOARD BOARD_RAMPS_13_HFB  //BOARD_MKS_13
#define MECHANISM MECH_CARTESIAN
//#define MECHANISM MECH_COREXY
//#define MECHANISM MECH_COREYX
//#define MECHANISM MECH_COREXZ
//#define MECHANISM MECH_COREZX
//#define MECHANISM MECH_COREYZ
//#define MECHANISM MECH_COREZY
#define POWER_SUPPLY 0
#define PS_DEFAULT_OFF false
#define DELAY_AFTER_POWER_ON 5
#define POWER_TIMEOUT 30
#define EXTRUDERS 1
#define DRIVER_EXTRUDERS 1

/*****************************
 * Configuration_Temperature *
 *****************************/
//#define TEMPERATURE_UNITS_SUPPORT
#define TEMP_SENSOR_0 9
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 9
#define TEMP_SENSOR_BED 0
#define TEMP_SENSOR_CHAMBER 0
#define THERMISTOR_SERIES_RS 4700
#define T1_NAME   "SENSOR1"
#define T1_R25    100000.0
#define T1_BETA     4092.0
#define T2_NAME   "SENSOR2"
#define T2_R25    100000.0
#define T2_BETA     3950.0
#define T3_NAME   "SENSOR3"
#define T3_R25    100000.0
#define T3_BETA     3950.0
#define T4_NAME   "SENSOR4"
#define T4_R25    100000.0
#define T4_BETA     3950.0
#define T5_NAME   "SENSOR5"
#define T5_R25    100000.0
#define T5_BETA     3950.0
#define T6_NAME   "SENSOR6"
#define T6_R25    100000.0
#define T6_BETA     3950.0
#define TEMP_SENSOR_AD595_OFFSET 0
#define TEMP_SENSOR_AD595_GAIN 1
#define DUMMY_THERMISTOR_998_VALUE 25
#define DUMMY_THERMISTOR_999_VALUE 100
#define TEMP_RESIDENCY_TIME 10
#define TEMP_HYSTERESIS 3
#define TEMP_WINDOW     1
#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define HEATER_3_MAXTEMP 275
#define BED_MAXTEMP 150
#define CHAMBER_MAXTEMP 150
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define HEATER_3_MINTEMP 5
#define BED_MINTEMP 5
#define CHAMBER_MINTEMP 5
#define PREHEAT_1_LABEL "PLA"
#define PREHEAT_1_TEMP_HOTEND 190
#define PREHEAT_1_TEMP_BED 60
#define PREHEAT_1_FAN_SPEED 255
#define PREHEAT_2_LABEL "ABS"
#define PREHEAT_2_TEMP_HOTEND 240
#define PREHEAT_2_TEMP_BED 100
#define PREHEAT_2_FAN_SPEED 255
#define PREHEAT_3_LABEL "GUM"
#define PREHEAT_3_TEMP_HOTEND 230
#define PREHEAT_3_TEMP_BED 60
#define PREHEAT_3_FAN_SPEED 255
//#define AUTOTEMP
#define AUTOTEMP_OLDWEIGHT 0.98
//#define TEMP_STAT_LEDS
#define PIDTEMP true
#define PID_MAX 255
#define PID_DRIVE_MIN 40
#define PID_DRIVE_MAX 230
#define PID_FUNCTIONAL_RANGE 10
#define PID_AUTOTUNE_MENU // Add PID Autotune to the LCD "Temperature" menu to run M303 and apply the result.
//#define PID_DEBUG       // Sends debug data to the serial port.
//#define PID_ADD_EXTRUSION_RATE
#define LPQ_MAX_LEN 50
#define DEFAULT_Kp {40,40,40,40}
#define DEFAULT_Ki {7,7,7,7}
#define DEFAULT_Kd {60,60,60,60}
#define DEFAULT_Kc {100,100,100,100}
#define PIDTEMPBED false
#define BED_HYSTERESIS        2 // Only disable heating if T>target+BED_HYSTERESIS and enable heating if T<target-BED_HYSTERESIS
#define BED_CHECK_INTERVAL 5000 // ms between checks in bang-bang control
#define DEFAULT_bedKp 10
#define DEFAULT_bedKi 1
#define DEFAULT_bedKd 305
#define PIDTEMPCHAMBER false
#define CHAMBER_HYSTERESIS        2 // only disable heating if T>target+CHAMBER_HYSTERESIS and enable heating if T<target-CHAMBER_HYSTERESIS
#define CHAMBER_CHECK_INTERVAL 5000 // ms between checks in bang-bang control
#define CHAMBER_PID_MAX       255   // Limits current to chamber while in PID mode;       255 = full current
#define CHAMBER_PID_DRIVE_MIN  80   // Limits min current to chamber while PID is active;   0 = no current
#define CHAMBER_PID_DRIVE_MAX 255   // Limits max current to chamber while PID is active; 255 = full current
#define DEFAULT_chamberKp 10
#define DEFAULT_chamberKi 1
#define DEFAULT_chamberKd 305
#define INVERTED_HEATER_PINS false
#define INVERTED_BED_PIN false
#define INVERTED_CHAMBER_PIN false
#define THERMAL_PROTECTION_HOTENDS false
#define THERMAL_PROTECTION_BED false
#define THERMAL_PROTECTION_CHAMBER false
#define THERMAL_PROTECTION_PERIOD 40
#define THERMAL_PROTECTION_HYSTERESIS 4
//#define ADAPTIVE_FAN_SPEED
#define WATCH_TEMP_PERIOD 20
#define WATCH_TEMP_INCREASE 2
#define WATCH_BED_TEMP_PERIOD 60
#define WATCH_BED_TEMP_INCREASE 2
#define WATCH_CHAMBER_TEMP_PERIOD 60
#define WATCH_CHAMBER_TEMP_INCREASE 2
#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 170
//#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH 400

/***************************
 * Configuration_Mechanics *
 ***************************/
#define CUSTOM_MACHINE_NAME "Generic"
#define ENDSTOPPULLUP_XMIN true
#define ENDSTOPPULLUP_YMIN true
#define ENDSTOPPULLUP_ZMIN true
#define ENDSTOPPULLUP_XMAX true
#define ENDSTOPPULLUP_YMAX true
#define ENDSTOPPULLUP_ZMAX true
#define ENDSTOPPULLUP_X2MIN true
#define ENDSTOPPULLUP_Y2MIN true
#define ENDSTOPPULLUP_Z2MIN true
#define ENDSTOPPULLUP_X2MAX true
#define ENDSTOPPULLUP_Y2MAX true
#define ENDSTOPPULLUP_Z2MAX true
#define ENDSTOPPULLUP_ZPROBE true
#define X_MIN_ENDSTOP_LOGIC false
#define Y_MIN_ENDSTOP_LOGIC false
#define Z_MIN_ENDSTOP_LOGIC false
#define X_MAX_ENDSTOP_LOGIC false
#define Y_MAX_ENDSTOP_LOGIC false
#define Z_MAX_ENDSTOP_LOGIC false
#define X2_MIN_ENDSTOP_LOGIC false
#define Y2_MIN_ENDSTOP_LOGIC false
#define Z2_MIN_ENDSTOP_LOGIC false
#define X2_MAX_ENDSTOP_LOGIC false
#define Y2_MAX_ENDSTOP_LOGIC false
#define Z2_MAX_ENDSTOP_LOGIC false
#define Z_PROBE_ENDSTOP_LOGIC false
//#define ENDSTOP_INTERRUPTS_FEATURE
#define Z_PROBE_SERVO_NR -1
#define Z_SERVO_ANGLES {90,0} // Z Servo Deploy and Stow angles
//#define PROBE_MANUALLY
//#define Z_PROBE_FIX_MOUNTED
//#define BLTOUCH
//#define BLTOUCH_DELAY 375 // (ms) Enable and increase if needed
#define X_PROBE_OFFSET_FROM_NOZZLE 0
#define Y_PROBE_OFFSET_FROM_NOZZLE 0
#define Z_PROBE_OFFSET_FROM_NOZZLE -1
#define XY_PROBE_SPEED 8000
#define Z_PROBE_SPEED_FAST 1000
#define Z_PROBE_SPEED_SLOW 500
#define Z_PROBE_REPETITIONS 1
//#define Z_MIN_PROBE_REPEATABILITY_TEST
//#define PAUSE_BEFORE_DEPLOY_STOW
#define Z_PROBE_DEPLOY_HEIGHT 15
#define Z_PROBE_BETWEEN_HEIGHT 10
#define Z_PROBE_AFTER_PROBING 0
#define Z_PROBE_LOW_POINT -2
#define Z_PROBE_OFFSET_RANGE_MIN -50
#define Z_PROBE_OFFSET_RANGE_MAX  50
//#define PROBING_HEATERS_OFF       // Turn heaters off when probing
//#define PROBING_FANS_OFF          // Turn fans off when probing
//#define LCD_BED_LEVELING
#define MESH_EDIT_Z_STEP 0.025  // (mm) Step size while manually probing Z axis.
#define LCD_PROBE_Z_RANGE 4     // (mm) Z Range centered on Z MIN POS for LCD Z adjustment
//#define MESH_EDIT_MENU        // Add a menu to edit mesh points
//#define LEVEL_BED_CORNERS
#define LEVEL_CORNERS_INSET 30    // (mm) An inset for corner leveling
//#define LEVEL_CENTER_TOO        // Move to the center after the last corner
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1
#define MIN_Z_HEIGHT_FOR_HOMING 0
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false
#define INVERT_X_DIR false
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false
#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false
#define INVERT_E5_DIR false
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false
//#define DISABLE_INACTIVE_EXTRUDER
#define X_MAX_POS 200
#define X_MIN_POS 0
#define Y_MAX_POS 200
#define Y_MIN_POS 0
#define Z_MAX_POS 200
#define Z_MIN_POS 0
#define E_MIN_POS 0
#define AXIS_RELATIVE_MODES {false, false, false, false}
//#define Z_SAFE_HOMING
#define Z_SAFE_HOMING_X_POINT 100
#define Z_SAFE_HOMING_Y_POINT 100
//#define MESH_BED_LEVELING
//#define AUTO_BED_LEVELING_UBL
//#define AUTO_BED_LEVELING_LINEAR
//#define AUTO_BED_LEVELING_BILINEAR
//#define AUTO_BED_LEVELING_3POINT
//#define MESH_EDIT_GFX_OVERLAY
#define MESH_INSET 10
//#define G26_MESH_VALIDATION
#define MESH_TEST_NOZZLE_SIZE    0.4  // (mm) Diameter of primary nozzle.
#define MESH_TEST_LAYER_HEIGHT   0.2  // (mm) Default layer height for the G26 Mesh Validation Tool.
#define MESH_TEST_HOTEND_TEMP  200.0  // (c)  Default nozzle temperature for the G26 Mesh Validation Tool.
#define MESH_TEST_BED_TEMP      60.0  // (c)  Default bed temperature for the G26 Mesh Validation Tool.
#define MESH_MIN_X (X_MIN_POS + (MESH_INSET))
#define MESH_MAX_X (X_MAX_POS - (MESH_INSET))
#define MESH_MIN_Y (Y_MIN_POS + (MESH_INSET))
#define MESH_MAX_Y (Y_MAX_POS - (MESH_INSET))
//#define MESH_G28_REST_ORIGIN
#define UBL_MESH_EDIT_MOVES_Z
//#define UBL_Z_RAISE_WHEN_OFF_MESH 2.5
#define GRID_MAX_POINTS_X 3
#define GRID_MAX_POINTS_Y 3
#define LEFT_PROBE_BED_POSITION 20
#define RIGHT_PROBE_BED_POSITION 180
#define FRONT_PROBE_BED_POSITION 20
#define BACK_PROBE_BED_POSITION 180
#define MIN_PROBE_EDGE 10
//#define PROBE_Y_FIRST
//#define ABL_BILINEAR_SUBDIVISION
#define BILINEAR_SUBDIVISIONS 3
#define PROBE_PT_1_X 15
#define PROBE_PT_1_Y 180
#define PROBE_PT_2_X 15
#define PROBE_PT_2_Y 15
#define PROBE_PT_3_X 180
#define PROBE_PT_3_Y 15
//#define Z_PROBE_END_SCRIPT "G1 Z10 F8000\nG1 X10 Y10\nG1 Z0.5"
//#define ENABLE_LEVELING_FADE_HEIGHT
//#define BED_CENTER_AT_0_0
//#define MANUAL_X_HOME_POS 0
//#define MANUAL_Y_HOME_POS 0
//#define MANUAL_Z_HOME_POS 0
#define DEFAULT_AXIS_STEPS_PER_UNIT {80,80,4000,625,625,625,625,625,625}
#define DEFAULT_MAX_FEEDRATE {300,300,2,100,100,100,100,100,100}
#define MANUAL_FEEDRATE {100*60,100*60,2*60,10*60}
#define DEFAULT_MIN_FEEDRATE          0.0
#define DEFAULT_MIN_TRAVEL_FEEDRATE   0.0
#define MINIMUM_PLANNER_SPEED         0.05                      // (mm/sec)
#define DEFAULT_MAX_ACCELERATION {3000,3000,50,3000,3000,3000,3000,3000,3000}
#define DEFAULT_RETRACT_ACCELERATION {10000,10000,10000,10000,10000,10000}
#define DEFAULT_ACCELERATION 3000
#define DEFAULT_TRAVEL_ACCELERATION 3000
#define DEFAULT_XJERK 10
#define DEFAULT_YJERK 10
#define DEFAULT_ZJERK 0.4
#define DEFAULT_EJERK {5,5,5,5,5,5}
#define HOMING_FEEDRATE_X (100*60)
#define HOMING_FEEDRATE_Y (100*60)
#define HOMING_FEEDRATE_Z (2*60)
#define X_HOME_BUMP_MM 5
#define Y_HOME_BUMP_MM 5
#define Z_HOME_BUMP_MM 2
#define HOMING_BUMP_DIVISOR {5,5,2}
#define HOTEND_OFFSET_X {0,0,0,0}
#define HOTEND_OFFSET_Y {0,0,0,0}
#define HOTEND_OFFSET_Z {0,0,0,0}
//#define HYSTERESIS_FEATURE
#define HYSTERESIS_AXIS_MM {0,0,0}
#define HYSTERESIS_CORRECTION 1

/*************************
 * Configuration_Feature *
 *************************/
#define X_DRIVER_TYPE DRV8825
#define Y_DRIVER_TYPE DRV8825
#define Z_DRIVER_TYPE DRV8825
#define X2_DRIVER_TYPE A4988
#define Y2_DRIVER_TYPE A4988
#define Z2_DRIVER_TYPE A4988
#define E0_DRIVER_TYPE DRV8825
#define E1_DRIVER_TYPE A4988
#define E2_DRIVER_TYPE A4988
#define E3_DRIVER_TYPE A4988
#define E4_DRIVER_TYPE A4988
#define E5_DRIVER_TYPE A4988
#define SOFT_PWM_SPEED 0
#define FAN_MIN_PWM 0
#define FAN_MAX_PWM 255
//#define INVERTED_FAN_PINS
#define FAN_PWM_FREQUENCY 250
//#define FAN_KICKSTART_TIME 0
#define AUTO_FAN {-1,0,-1,-1,-1,-1}
#define HOTEND_AUTO_FAN_TEMPERATURE 50
#define HOTEND_AUTO_FAN_SPEED 255
#define HOTEND_AUTO_FAN_MIN_SPEED 0
#define CONTROLLERFAN_SECS 60
#define CONTROLLERFAN_SPEED 255
#define CONTROLLERFAN_MIN_SPEED 0
//#define VOLUMETRIC_EXTRUSION
//#define VOLUMETRIC_DEFAULT_ON
#define NOMINAL_FILAMENT_DIA 1.75
//#define SINGLENOZZLE
//#define COLOR_MIXING_EXTRUDER
#define MIXING_STEPPERS 2
#define MIXING_VIRTUAL_TOOLS 16
//#define MKR4
//#define INVERTED_RELE_PINS
//#define MKR6
//#define INVERTED_RELE_PINS
//#define MKR12
//#define INVERTED_RELE_PINS
//#define MKSE6
#define MKSE6_SERVO_INDEX    0
#define MKSE6_SERVOPOS_E0  -60
#define MKSE6_SERVOPOS_E1  -30
#define MKSE6_SERVOPOS_E2    0
#define MKSE6_SERVOPOS_E3   30
#define MKSE6_SERVOPOS_E4   60
#define MKSE6_SERVOPOS_E5   90
#define MKSE6_SERVO_DELAY 1000
//#define IDLE_OOZING_PREVENT
#define IDLE_OOZING_MINTEMP           190
#define IDLE_OOZING_FEEDRATE          50    //default feedrate for retracting (mm/s)
#define IDLE_OOZING_SECONDS           5
#define IDLE_OOZING_LENGTH            15    //default retract length (positive mm)
#define IDLE_OOZING_RECOVER_LENGTH    0     //default additional recover length (mm, added to retract length when recovering)
#define IDLE_OOZING_RECOVER_FEEDRATE  50    //default feedrate for recovering from retraction (mm/s)
//#define EXTRUDER_RUNOUT_PREVENT
#define EXTRUDER_RUNOUT_MINTEMP 190
#define EXTRUDER_RUNOUT_SECONDS  30
#define EXTRUDER_RUNOUT_SPEED  1500 // mm/m
#define EXTRUDER_RUNOUT_EXTRUDE   5 // mm
//#define LIN_ADVANCE
#define LIN_ADVANCE_K 0.22
//#define LA_DEBUG
//#define WORKSPACE_OFFSETS
#define DEFAULT_STEPPER_DEACTIVE_TIME 120
#define DISABLE_INACTIVE_X
#define DISABLE_INACTIVE_Y
#define DISABLE_INACTIVE_Z
#define DISABLE_INACTIVE_E
#define MIN_SOFTWARE_ENDSTOPS
#define MAX_SOFTWARE_ENDSTOPS
#define ENDSTOPS_ONLY_FOR_HOMING
//#define ENABLED_ALL_SIX_ENDSTOP
//#define ABORT_ON_ENDSTOP_HIT
#define ABORT_ON_ENDSTOP_HIT_DEFAULT true
//#define G38_PROBE_TARGET
#define G38_MINIMUM_MOVE 0.0275
//#define ENABLE_SERVOS
#define NUM_SERVOS 0
//#define DEACTIVATE_SERVOS_AFTER_MOVE
#define SERVO_DEACTIVATION_DELAY 300
//#define Z_LATE_ENABLE
#define SLOWDOWN
//#define QUICK_HOME
//#define HOME_Y_BEFORE_X
//#define FORCE_HOME_XY_BEFORE_Z
//#define BABYSTEPPING
//#define BABYSTEP_XY
#define BABYSTEP_INVERT_Z false
#define BABYSTEP_MULTIPLICATOR 1
//#define BABYSTEP_ZPROBE_OFFSET
//#define DOUBLECLICK_FOR_Z_BABYSTEPPING
#define DOUBLECLICK_MAX_INTERVAL 1250
//#define BABYSTEP_ZPROBE_GFX_OVERLAY
//#define BABYSTEP_ZPROBE_GFX_REVERSE
//#define FWRETRACT
#define MIN_AUTORETRACT               0.1 // When auto-retract is on, convert E moves of this length and over
#define MAX_AUTORETRACT              10.0 // Upper limit for auto-retract conversion
#define RETRACT_LENGTH                3   // Default retract length (positive mm)
#define RETRACT_LENGTH_SWAP          13   // Default swap retract length (positive mm), for extruder change
#define RETRACT_FEEDRATE             45   // Default feedrate for retracting (mm/s)
#define RETRACT_ZLIFT                 0   // Default retract Z-lift
#define RETRACT_RECOVER_LENGTH        0   // Default additional recover length (mm, added to retract length when recovering)
#define RETRACT_RECOVER_LENGTH_SWAP   0   // Default additional swap recover length (mm, added to retract length when recovering from extruder change)
#define RETRACT_RECOVER_FEEDRATE      8   // Default feedrate for recovering from retraction (mm/s)
#define RETRACT_RECOVER_FEEDRATE_SWAP 8   // Default feedrate for recovering from swap retraction (mm/s)
#define TOOLCHANGE_PARK_ZLIFT   0.2      // the distance to raise Z axis when parking an extruder
#define TOOLCHANGE_UNPARK_ZLIFT 1        // the distance to raise Z axis when unparking an extruder
//#define X_TWO_STEPPER_DRIVERS
#define INVERT_X2_VS_X_DIR false
//#define X_TWO_ENDSTOPS
//#define Y_TWO_STEPPER_DRIVERS
#define INVERT_Y2_VS_Y_DIR false
//#define Y_TWO_ENDSTOPS
//#define Z_TWO_STEPPER_DRIVERS
#define INVERT_Z2_VS_Z_DIR false
//#define Z_TWO_ENDSTOPS
//#define XY_FREQUENCY_LIMIT  15
//#define SF_ARC_FIX
//#define EXTRUDER_ENCODER_CONTROL
#define ENC_ERROR_STEPS 500
#define ENC_MIN_STEPS 10
//#define INVERTED_ENCODER_PINS
//#define FILAMENT_RUNOUT_SENSOR
#define FIL_RUNOUT_0_LOGIC false
#define FIL_RUNOUT_1_LOGIC false
#define FIL_RUNOUT_2_LOGIC false
#define FIL_RUNOUT_3_LOGIC false
#define FIL_RUNOUT_4_LOGIC false
#define FIL_RUNOUT_5_LOGIC false
#define FIL_RUNOUT_0_PULLUP false
#define FIL_RUNOUT_1_PULLUP false
#define FIL_RUNOUT_2_PULLUP false
#define FIL_RUNOUT_3_PULLUP false
#define FIL_RUNOUT_4_PULLUP false
#define FIL_RUNOUT_5_PULLUP false
#define FILAMENT_RUNOUT_DOUBLE_CHECK 0
#define FILAMENT_RUNOUT_SCRIPT "M600"
//#define DOOR_OPEN_FEATURE
#define DOOR_OPEN_LOGIC false
#define PULLUP_DOOR_OPEN true
#define EEPROM_SETTINGS
#define EEPROM_CHITCHAT
//#define EEPROM_I2C
//#define EEPROM_SPI
//#define EEPROM_SD
//#define EEPROM_FLASH
//#define DISABLE_M503
#define SDSUPPORT
//#define USB_FLASH_DRIVE_SUPPORT
#define SD_HALF_SPEED
//#define SD_QUARTER_SPEED
//#define SD_EIGHTH_SPEED
//#define SD_SIXTEENTH_SPEED
//#define SD_CHECK_AND_RETRY
//#define SD_EXTENDED_DIR
//#define SD_DISABLED_DETECT
//#define SD_DETECT_INVERTED
#define SD_FINISHED_STEPPERRELEASE true           // if sd support and the file is finished: disable steppers?
#define SD_FINISHED_RELEASECOMMAND "M84 X Y Z E"  // You might want to keep the z enabled so your bed stays in place.
//#define MENU_ADDAUTOSTART
//#define SCROLL_LONG_FILENAMES
//#define SDCARD_SORT_ALPHA
#define SDSORT_LIMIT       40     // Maximum number of sorted items (10-256). Costs 27 bytes each.
#define FOLDER_SORTING     -1     // -1=above  0=none  1=below
#define SDSORT_GCODE       false  // Allow turning sorting on/off with LCD and M36 g-code.
#define SDSORT_USES_RAM    false  // Pre-allocate a static array for faster pre-sorting.
#define SDSORT_USES_STACK  false  // Prefer the stack for pre-sorting to give back some SRAM. (Negated by next 2 options.)
#define SDSORT_CACHE_NAMES false  // Keep sorted items in RAM longer for speedy performance. Most expensive option.
#define SDSORT_DYNAMIC_RAM false  // Use dynamic allocation (within SD menus). Least expensive option. Set SDSORT_LIMIT before use!
#define SDSORT_CACHE_VFATS 2      // Maximum number of 13-byte VFAT entries to use for sorting.
//#define SD_RESTART_FILE           // Uncomment to enable
#define SD_RESTART_FILE_SAVE_TIME 1
#define LCD_LANGUAGE it
#define DISPLAY_CHARSET_HD44780 JAPANESE
#define LCD_INFO_SCREEN_STYLE 0
//#define REPRAP_DISCOUNT_SMART_CONTROLLER
#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
#define SHOW_BOOTSCREEN
#define STRING_SPLASH_LINE1 "v" SHORT_BUILD_VERSION   // will be shown during bootup in line 1
#define STRING_SPLASH_LINE2 STRING_DISTRIBUTION_DATE  // will be shown during bootup in line 2
#define BOOTSCREEN_TIMEOUT 2500
//#define SHOW_CUSTOM_BOOTSCREEN
//#define CUSTOM_STATUS_SCREEN_IMAGE
#define XYZ_HOLLOW_FRAME
#define MENU_HOLLOW_FRAME
//#define USE_BIG_EDIT_FONT
//#define USE_SMALL_INFOFONT
//#define DOGM_SPI_DELAY_US 5
//#define OVERLAY_GFX_REVERSE
//#define STATUS_COMBINE_HEATERS    // Use combined heater images instead of separate ones
//#define STATUS_HOTEND_NUMBERLESS  // Use plain hotend icons instead of numbered ones (with 2+ hotends)
#define STATUS_HOTEND_INVERTED      // Show solid nozzle bitmaps when heating (Requires STATUS_HOTEND_ANIM)
#define STATUS_HOTEND_ANIM          // Use a second bitmap to indicate hotend heating
#define STATUS_BED_ANIM             // Use a second bitmap to indicate bed heating
//#define STATUS_ALT_BED_BITMAP     // Use the alternative bed bitmap
//#define STATUS_ALT_FAN_BITMAP     // Use the alternative fan bitmap
//#define STATUS_FAN_FRAMES 3       // :[0,1,2,3,4] Number of fan animation frames
//#define STATUS_HEAT_PERCENT       // Show heating in a progress bar
//#define NO_LCD_MENUS
//#define SLIM_LCD_MENUS
#define ENCODER_PULSES_PER_STEP 5
#define ENCODER_STEPS_PER_MENU_ITEM 1
//#define LCD_SCREEN_ROT_90    // Rotate screen orientation for graphics display by 90 degree clockwise
//#define LCD_SCREEN_ROT_180   // Rotate screen orientation for graphics display by 180 degree clockwise
//#define LCD_SCREEN_ROT_270   // Rotate screen orientation for graphics display by 270 degree clockwise
//#define INVERT_CLICK_BUTTON           // Option for invert encoder button logic
//#define INVERT_BACK_BUTTON            // Option for invert back button logic if avaible
//#define REVERSE_ENCODER_DIRECTION
//#define REVERSE_MENU_DIRECTION
#define ENCODER_RATE_MULTIPLIER         // If defined, certain menu edit operations automatically multiply the steps when the encoder is moved quickly
#define ENCODER_10X_STEPS_PER_SEC 75    // If the encoder steps per sec exceeds this value, multiply steps moved x10 to quickly advance the value
#define ENCODER_100X_STEPS_PER_SEC 160  // If the encoder steps per sec exceeds this value, multiply steps moved x100 to really quickly advance the value
#define ULTIPANEL_FEEDMULTIPLY
//#define SPEAKER
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 2
//#define LCD_FEEDBACK_FREQUENCY_HZ 5000
//#define UI_VOLTAGE_LEVEL 0 // 3.3 V
#define UI_VOLTAGE_LEVEL 1   // 5 V
#define LCD_INFO_MENU
//#define STATUS_MESSAGE_SCROLLING
//#define LCD_DECIMAL_SMALL_XY
//#define LCD_TIMEOUT_TO_STATUS 15000
//#define LED_CONTROL_MENU
//#define LED_COLOR_PRESETS             // Enable the Preset Color menu option
//#define LED_USER_PRESET_STARTUP       // Have the printer display the user preset color on startup
#define LED_USER_PRESET_RED        255  // User defined RED value
#define LED_USER_PRESET_GREEN      255  // User defined GREEN value
#define LED_USER_PRESET_BLUE       255  // User defined BLUE value
#define LED_USER_PRESET_WHITE      255  // User defined WHITE value
#define LED_USER_PRESET_BRIGHTNESS 255  // User defined intensity
//#define LCD_PROGRESS_BAR
#define PROGRESS_BAR_BAR_TIME 5000
#define PROGRESS_BAR_MSG_TIME 1500
#define PROGRESS_MSG_EXPIRE 0
//#define PROGRESS_MSG_ONCE
//#define LCD_PROGRESS_BAR_TEST
//#define PHOTOGRAPH
//#define CHDK
#define CHDK_DELAY 50   //How long in ms the pin should stay HIGH before going LOW again
//#define BLINKM
//#define RGB_LED
//#define RGBW_LED
//#define PCA9632
//#define NEOPIXEL_LED
#define NEOPIXEL_TYPE NEO_GRB
#define NEOPIXEL_PIXELS 16
#define NEOPIXEL_IS_SEQUENTIAL
#define NEOPIXEL_BRIGHTNESS 127
//#define NEOPIXEL_STARTUP_TEST
//#define PRINTER_EVENT_LEDS
//#define CASE_LIGHT
#define INVERT_CASE_LIGHT false
#define CASE_LIGHT_DEFAULT_ON false
#define CASE_LIGHT_DEFAULT_BRIGHTNESS 255
//#define CASE_LIGHT_USE_NEOPIXEL
#define CASE_LIGHT_NEOPIXEL_COLOR { 255, 255, 255, 255 }
//#define DISABLE_DOUBLE_QUAD_STEPPING
//#define JUNCTION_DEVIATION
#define JUNCTION_DEVIATION_MM 0.02
//#define BEZIER_JERK_CONTROL
#define MINIMUM_STEPPER_PULSE 0
#define MAXIMUM_STEPPER_RATE 500000
#define DIRECTION_STEPPER_DELAY 0
//#define ADAPTIVE_STEP_SMOOTHING
#define BLOCK_BUFFER_SIZE 16
#define MAX_CMD_SIZE 96
#define BUFSIZE 4
#define TX_BUFFER_SIZE 0
#define RX_BUFFER_SIZE 128
//#define SERIAL_XON_XOFF
//#define SERIAL_STATS_MAX_RX_QUEUED
//#define SERIAL_STATS_DROPPED_RX
#define NUM_POSITON_SLOTS 2
#define DEFAULT_MIN_SEGMENT_TIME 20000
#define ARC_SUPPORT
#define MM_PER_ARC_SEGMENT 1    // Length of each arc segment
#define N_ARC_CORRECTION  25    // Number of intertpolated segments between corrections
//#define ARC_P_CIRCLES         // Enable the 'P' parameter to specify complete circles
#define MIN_STEPS_PER_SEGMENT 6
//#define M100_FREE_MEMORY_WATCHER
#define M100_FREE_MEMORY_DUMPER
#define M100_FREE_MEMORY_CORRUPTOR
//#define NOZZLE_CLEAN_FEATURE
#define NOZZLE_CLEAN_STROKES 12
#define NOZZLE_CLEAN_TRIANGLES 3
#define NOZZLE_CLEAN_START_POINT {30,30,1}
#define NOZZLE_CLEAN_END_POINT {100,60,1}
#define NOZZLE_CLEAN_CIRCLE_RADIUS 6.5
#define NOZZLE_CLEAN_CIRCLE_FN 10
#define NOZZLE_CLEAN_CIRCLE_MIDDLE NOZZLE_CLEAN_START_POINT
#define NOZZLE_CLEAN_GOBACK
//#define NOZZLE_PARK_FEATURE
#define NOZZLE_PARK_POINT {10,10,20}
#define NOZZLE_PARK_XY_FEEDRATE 100
#define NOZZLE_PARK_Z_FEEDRATE 5
//#define ADVANCED_PAUSE_FEATURE
#define PAUSE_PARK_RETRACT_FEEDRATE 20
#define PAUSE_PARK_RETRACT_LENGTH 5
#define PAUSE_PARK_UNLOAD_FEEDRATE 50
#define PAUSE_PARK_UNLOAD_LENGTH 100
#define PAUSE_PARK_SLOW_LOAD_FEEDRATE 6
#define PAUSE_PARK_SLOW_LOAD_LENGTH 5
#define PAUSE_PARK_FAST_LOAD_FEEDRATE 50
#define PAUSE_PARK_FAST_LOAD_LENGTH 100
#define PAUSE_PARK_EXTRUDE_FEEDRATE 5
#define PAUSE_PARK_EXTRUDE_LENGTH 50
#define FILAMENT_UNLOAD_RETRACT_LENGTH 10
#define FILAMENT_UNLOAD_DELAY 5000
#define FILAMENT_UNLOAD_PURGE_LENGTH 8
#define PAUSE_PARK_NOZZLE_TIMEOUT 45
#define PAUSE_PARK_PRINTER_OFF 5
#define PAUSE_PARK_NUMBER_OF_ALERT_BEEPS 5
#define PAUSE_PARK_NO_STEPPER_TIMEOUT         // Enable for XYZ steppers to stay powered on during filament change.
//#define PARK_HEAD_ON_PAUSE                  // Park the nozzle during pause and filament change.
//#define HOME_BEFORE_FILAMENT_CHANGE         // Ensure homing has been completed prior to parking for filament change
//#define FILAMENT_LOAD_UNLOAD_GCODES         // Add M701/M702 Load/Unload G-codes, plus Load/Unload in the LCD Prepare menu.
//#define FILAMENT_UNLOAD_ALL_EXTRUDERS       // Allow M702 to unload all extruders above a minimum target temp (as set by M302)
//#define INCH_MODE_SUPPORT
//#define JSON_OUTPUT
//#define SCAD_MESH_OUTPUT
//#define PINS_DEBUGGING
//#define DEBUG_FEATURE
//#define USE_WATCHDOG
//#define WATCHDOG_RESET_MANUAL
//#define START_GCODE
#define START_PRINTING_SCRIPT "G28\nG1 Z10 F8000"
//#define STOP_GCODE
#define STOP_PRINTING_SCRIPT "G28\nM107\nM104 T0 S0\nM140 S0\nM84\nM81"
#define PROPORTIONAL_FONT_RATIO 1
//#define CUSTOM_USER_MENUS
#define USER_SCRIPT_DONE "M117 User Script Done"
#define USER_DESC_1 "Home & ABL"
#define USER_GCODE_1 "G28\nG29"
#define USER_DESC_2 "Preheat for " PREHEAT_1_LABEL
#define USER_GCODE_2 "M140 S" STRINGIFY(PREHEAT_1_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND)
#define USER_DESC_3 "Preheat for " PREHEAT_2_LABEL
#define USER_GCODE_3 "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND)
#define USER_DESC_4 "Heat Bed/Home/Level"
#define USER_GCODE_4 "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nG28\nG29"
#define USER_DESC_5 "Home & Info"
#define USER_GCODE_5 "G28\nM503"
