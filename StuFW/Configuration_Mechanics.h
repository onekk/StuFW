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
 * Configuration_Mechanics.h
 * This configuration file contains mechanism common to all printer mechanics.
 * the scope of this file is to not duplicate settings and make maintenance
 * less prone to errors.
 */


 /***********************************************************************
  *********************************** Machine name **********************
  ***********************************************************************
  *                                                                     *
  * This to set a custom name for your generic Mendel.                  *
  * Displayed in the LCD "Ready" message.                               *
  *                                                                     *
  ***********************************************************************/
 #define CUSTOM_MACHINE_NAME "3D Printer"
 /***********************************************************************/

 /***********************************************************************
  ************************* Endstop pullup resistors ********************
  ***********************************************************************
  *                                                                     *
  * true to enable or put false to disable endstop pullup resistors     *
  *                                                                     *
  ***********************************************************************/
 #define ENDSTOPPULLUP_XMIN    false
 #define ENDSTOPPULLUP_YMIN    false
 #define ENDSTOPPULLUP_ZMIN    false
 #define ENDSTOPPULLUP_XMAX    false
 #define ENDSTOPPULLUP_YMAX    false
 #define ENDSTOPPULLUP_ZMAX    false
 #define ENDSTOPPULLUP_Z2MIN   false
 #define ENDSTOPPULLUP_Z3MIN   false
 #define ENDSTOPPULLUP_Z2MAX   false
 #define ENDSTOPPULLUP_Z3MAX   false
 #define ENDSTOPPULLUP_ZPROBE  false

 #if MECH(CARTESIAN)
   // Cartesian Mechanics could have dual endstop for x and y
   #define ENDSTOPPULLUP_X2MIN   false
   #define ENDSTOPPULLUP_Y2MIN   false
   #define ENDSTOPPULLUP_X2MAX   false
   #define ENDSTOPPULLUP_Y2MAX   false
 #endif
 /***********************************************************************/


 /***********************************************************************
  ************************************ Endstops logic *******************
  ***********************************************************************
  *                                                                     *
  * Mechanical endstop with COM to ground and NC to Signal              *
  * uses "false" here (most common setup).                              *
  * set to true to invert endstop logic.                                *
  *                                                                     *
  ***********************************************************************/
 #define X_MIN_ENDSTOP_LOGIC   false
 #define Y_MIN_ENDSTOP_LOGIC   false
 #define Z_MIN_ENDSTOP_LOGIC   false
 #define X_MAX_ENDSTOP_LOGIC   false
 #define Y_MAX_ENDSTOP_LOGIC   false
 #define Z_MAX_ENDSTOP_LOGIC   false
 #define Z2_MIN_ENDSTOP_LOGIC  false
 #define Z3_MIN_ENDSTOP_LOGIC  false
 #define Z2_MAX_ENDSTOP_LOGIC  false
 #define Z3_MAX_ENDSTOP_LOGIC  false
 #define Z_PROBE_ENDSTOP_LOGIC false

 #if MECH(CARTESIAN)
   // Cartesian Mechanics could have dual endstop for x and y
   #define X2_MIN_ENDSTOP_LOGIC  false
   #define Y2_MIN_ENDSTOP_LOGIC  false
   #define X2_MAX_ENDSTOP_LOGIC  false
   #define Y2_MAX_ENDSTOP_LOGIC  false
 #endif
 /***********************************************************************/


 /***********************************************************************
  ***************************** Endstop interrupts feature **************
  ***********************************************************************
  *                                                                     *
  * Enable this feature if all enabled endstop are wired to             *
  * interrupt-capable pins.                                             *
  * This will remove the need to poll the interrupt pins,               *
  * saving many CPU cycles.                                             *
  *                                                                     *
  ***********************************************************************/
 //#define ENDSTOP_INTERRUPTS_FEATURE
 /***********************************************************************/
 /***********************************************************************
  ******************************* Z probe Options ***********************
  ***********************************************************************
  *                                                                     *
  * Probes are sensors/switches that need to be activated before they   *
  * can be used and deactivated after their use.                        *
  * Servo Probes, Z Sled Probe, Fix mounted Probe, etc.                 *
  * You must activate one of these to use AUTO BED LEVELING FEATURE     *
  *                                                                     *
  * If you want to still use the Z min endstop for homing,              *
  * disable Z SAFE HOMING.                                              *
  * Eg: to park the head outside the bed area when homing with G28.     *
  *                                                                     *
  * WARNING: The Z MIN endstop will need to set properly as it would    *
  * without a Z PROBE to prevent head crashes and premature stopping    *
  * during a print.                                                     *
  * To use a separte Z PROBE endstop, you must have a Z PROBE PIN       *
  * defined in the Configuration_Pins.h file for your control board.    *
  *                                                                     *
  * Use M851 X Y Z to set the probe offset from the nozzle.             *
  Store with M500.                                                      *
  * WARNING: Setting the wrong pin may have unexpected and potentially  *
  * disastrous outcomes. Use with caution and do your homework.         *
  *                                                                     *
  ***********************************************************************/

  /**********************************************************************
  * Z Servo Endstop                                                     *
  ***********************************************************************
  * Remember active servos in Configuration_Feature.h                   *
  * Define nr servo for endstop -1 not define. Servo index start 0      *
  ***********************************************************************/

 #define Z_PROBE_SERVO_NR -1
 #define Z_SERVO_ANGLES {90,0} // Z Servo Deploy and Stow angles

  /**********************************************************************
  * Manual Probing                                                      *
  ***********************************************************************
  * PROBE_MANUALLY provides a means to do "Auto" Bed Leveling without a *
  * proper probe, using Host or LCD to adjust Z height.                 *
  ***********************************************************************/
 //#define PROBE_MANUALLY

  /**********************************************************************
  * Fix-Mounted Probe                                                   *
  ***********************************************************************
  * Fix-Mounted Probe either doesn't deploy or needs manual deployment. *
  * Most common example is an inductive probe.                          *
  * Note: Inductive probe must be deactivated to go below its           *
  *   trigger-point if hardware endstops are active.                    *
  ***********************************************************************/

 //#define Z_PROBE_FIX_MOUNTED

  /**********************************************************************
  * BLTouch                                                             *
  ***********************************************************************
  * BLTouch probe uses a Hall effect sensor and emulates a servo.       *
  * Default connection of the servo part is SERVO 0.                    *
  ***********************************************************************/
 //#define BLTOUCH
 //#define BLTOUCH_DELAY 375 // (ms) Enable and increase if needed

  /**********************************************************************
  * Offsets to the probe relative to the nozzle tip (Nozzle - Probe)    *
  * X and Y offsets MUST be INTEGERS                                    *
  *                                                                     *
 *    +-- BACK ---+                                                     *
  *    |           |                                                    *
  *  L |    (+) P  | R <-- probe (10,10)                                *
  *  E |           | I                                                  *
  *  F | (-) N (+) | G <-- nozzle (0,0)                                 *
  *  T |           | H                                                  *
  *    |  P (-)    | T <-- probe (-10,-10)                              *
  *    |           |                                                    *
  *    O-- FRONT --+                                                    *
  *  (0,0)                                                              *
  *                                                                     *
  *  X offset: -left  [of the nozzle] +right                            *
  *  Y offset: -front [of the nozzle] +behind                           *
  *  Z offset: -below [of the nozzle] (always negative!)                *
  ***********************************************************************/
 #define X_PROBE_OFFSET_FROM_NOZZLE  0
 #define Y_PROBE_OFFSET_FROM_NOZZLE  0
 #define Z_PROBE_OFFSET_FROM_NOZZLE -1

 // X and Y axis travel speed between probes, in mm/min
 #define XY_PROBE_SPEED 10000
 // Speed for the first approach when double-probing, in mm/min
 #define Z_PROBE_SPEED_FAST 120
 // Speed for the "accurate" probe of each point, in mm/min
 #define Z_PROBE_SPEED_SLOW 60
 // Z Probe repetitions, median for best result
 #define Z_PROBE_REPETITIONS 1

 // Enable Z Probe Repeatability test to see how accurate your probe is
 //#define Z_MIN_PROBE_REPEATABILITY_TEST

 // Before deploy/stow pause for user confirmation
 //#define PAUSE_BEFORE_DEPLOY_STOW



 // Probe Raise options provide clearance for the probe to deploy, stow, and travel.
 #define Z_PROBE_DEPLOY_HEIGHT 15  // Z position for the probe to deploy/stow
 #define Z_PROBE_BETWEEN_HEIGHT 5  // Z position for travel between points
 #define Z_PROBE_AFTER_PROBING  0  // Z position after probing is done
 #define Z_PROBE_LOW_POINT     -2  // Farthest distance below the trigger-point to go before stopping

 // For M851 give a range for adjusting the Probe Z Offset
 #define Z_PROBE_OFFSET_RANGE_MIN -50
 #define Z_PROBE_OFFSET_RANGE_MAX  50

 /***********************************************************************
  * Enable these settings if probing seems unreliable.                  *
  * Heaters and/or fans according to options selected will be disabled  *
  * during probing to minimize potential EM interference, shutting off  *
  * source of the 'noise' (change in current flowing through the wires) *
  *                                                                     *
  ***********************************************************************/
 //#define PROBING_HEATERS_OFF
 //#define PROBING_FANS_OFF

 /**********************************************************************
 * Add a bed leveling sub-menu for ABL or MBL.                         *
 * Include a guided procedure if manual probing is enabled.            *
  ***********************************************************************/
//#define LCD_BED_LEVELING
// (mm) Step size while manually probing Z axis.
#define MESH_EDIT_Z_STEP 0.025
// (mm) Z Range centered on Z MIN POS for LCD Z adjustment
#define LCD_PROBE_Z_RANGE 4
// Add a menu to edit mesh points
//#define MESH_EDIT_MENU

// Add a menu item to move between bed corners for manual bed adjustment
//#define LEVEL_BED_CORNERS
// (mm) An inset for corner leveling
#define LEVEL_CORNERS_INSET 30
// Move to the center after the last corner
//#define LEVEL_CENTER_TOO

 /**********************************************************************
 * HOMING DIRECTION                                                    *
 ***********************************************************************
 *                                                                     *                                                                                       *
 * Sets homing direction when homing for each endstop;                 *
 * Values:  1 = MAX, -1 = MIN                                          *
 *                                                                     *
 ***********************************************************************/
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1


 /**********************************************************************
 * MIN Z HEIGHT FOR HOMING                                             *
 ***********************************************************************
 *                                                                     *
 * Minimal z height before homing (G28) for Z clearance above the bed, *
 * clamps, or other obstacles                                          *
 * Be sure you have this distance over your Z MAX POS in case.         *
 * Value in mm                                                         *
 ***********************************************************************/
#define MIN_Z_HEIGHT_FOR_HOMING 0

/***********************************************************************
 * STEPPERS: ENABLE PIN LOGIC                                          *
 ***********************************************************************
 *                                                                     *
 * Values:                                                             *
 *  0 = (Active Low) Inverting Stepper                                 *
 *  1 = (Active High) Non Inverting                                    *
 *                                                                     *
 ***********************************************************************/
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0

/***********************************************************************
 * STEPPERS: STEP PIN LOGIC                                            *
 ***********************************************************************
 *                                                                     *
 * Values:                                                             *
 *  true = (Active Low)  some high power drivers                       *
 *  false = (Active High) most common on Poulu drivers and similar     *
*                                                                      *
 ***********************************************************************/

#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false


/***********************************************************************
 * STEPPERS: DIR PIN LOGIC                                             *
 ***********************************************************************
 *                                                                     *
 * This setting invert stepper direction setting DIR pin               *
 * Change to true if an axis goes the wrong way.                       *
 * It is better to set motor connections the right way                 *
 ***********************************************************************/
#define INVERT_X_DIR false
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false
#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false
#define INVERT_E5_DIR false

/***********************************************************************
 * DISABLE MOTORS                                                      *
 ***********************************************************************
 *                                                                     *
 * Disables axis motors when it's not being used.                      *
 * Use with caution as it could create problem due to the lack of      *
 * holding torque, expecialy for Z axis this setting is dangerous      *
 ***********************************************************************/
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false
// Disable only inactive extruder and keep active extruder enabled
//#define DISABLE_INACTIVE_EXTRUDER

/**********************************************************************
* TRAVEL LIMITS                                                       *
***********************************************************************
*                                                                     *
* Travel limits after homing (values are in mm)                       *
*                                                                     *
* Note: X_MIN_POS and Y_MIN_POS cpould be negative endstops X and Y   *
* positions are outside the bed corner, suppose you have:             *
* X_MIN endstop at 15 mm from the border of the bed                   *
* Y_MIN endstop at 25mm from the border of the bed                    *
* in this case you have to set:                                       *
* X_MIN_POS -15                                                       *
* Y_MIN_POS -25                                                       *
* X_MAX_POS and Y_MAX_POS are the distance from the "Bed origin" and  *
* the max reachable position of the axis.                             *
* suppose that you bed origin is 0,0 and your X axis could travel     *
* 250mm without touching somethiung you could set X_MAX_POS 249       *
* same for Y_MAX_POS, some allowance could be useful                  *
*                                                                     *
* Bed limits are usually set in Slicer see:                           *
* https://reprap.org/wiki/Configuring_Marlin_Bed_Dimensions           *
*                                                                     *
***********************************************************************/
#define X_MAX_POS 200
#define X_MIN_POS 0
#define Y_MAX_POS 200
#define Y_MIN_POS 0
#define Z_MAX_POS 200
#define Z_MIN_POS 0
#define E_MIN_POS 0

/**********************************************************************
* Axis relative mode                                                  *
***********************************************************************/
#define AXIS_RELATIVE_MODES {false, false, false, false}

/**********************************************************************
* Safe Z homing                                                       *
***********************************************************************
*                                                                     *
* If you have enabled auto bed levelling feature or are using         *
* Z Probe for Z Homing, it is highly recommended you let              *
* this Z_SAFE_HOMING enabled!!!                                       *
*                                                                     *
* X point for Z homing when homing all axis (G28)                     *
* Y point for Z homing when homing all axis (G28)                     *
*                                                                     *
* Uncomment Z_SAFE_HOMING to enable                                   *
*                                                                     *
***********************************************************************/
//#define Z_SAFE_HOMING
#define Z_SAFE_HOMING_X_POINT ((X_MIN_POS + X_MAX_POS) / 2)
#define Z_SAFE_HOMING_Y_POINT ((Y_MIN_POS + Y_MAX_POS) / 2)

/***********************************************************************
 ********************************** Bed Leveling ***********************
 ***********************************************************************
 *                                                                     *
 * Select one from of Bed Leveling below.                              *
 *                                                                     *
 *  If you're also using the Probe for Z Homing, it's                  *
 *  highly recommended to enable Z SAFE HOMING also!                   *
 *                                                                     *
 * - MESH                                                              *
 *   Probe a grid manually                                             *
 *   The result is a mesh, suitable for large or uneven beds.          *
 *   (See BILINEAR)                                                    *
 *   For machines without a probe, Mesh Bed Leveling provides a method *
 *   to perform leveling in steps so you can manually adjust Z height  *
 *   at each grid-point.                                               *
 *   With an LCD controller the process is guided step-by-step.        *
 *                                                                     *
 * - UBL (Unified Bed Leveling) (FOR NOW IS BROKEN)                    *
 *   A comprehensive bed leveling system combining the features and    *
 *   benefits of other systems.                                        *
 *   UBL also includes integrated Mesh Generation, Mesh Validation     *
 *   and Mesh Editing systems.                                         *
 *                                                                     *
 * - LINEAR                                                            *
 *   Probe several points in a grid.                                   *
 *   You specify limits and density of sample points.                  *
 *   The result is a single tilted plane. Best for a flat bed set not  *
 *   in level with the nozzle travel                                   *
 *                                                                     *
 * - BILINEAR (Prefered)                                               *
 *   Probe several points in a grid.                                   *
 *   You specify limits and density of sample points.                  *
 *   The result is a grid, best for large or uneven beds.              *
 *                                                                                       *
 * - 3POINT                                                                              *
 *   Probe 3 arbitrary points on the bed (that aren't collinear)       *
 *   You specify the XY coordinates of all 3 points.                   *
 *   The result is a single tilted plane. Best for a flat bed.         *
 *                                                                     *
 ***********************************************************************/
//#define MESH_BED_LEVELING
//#define AUTO_BED_LEVELING_UBL
//#define AUTO_BED_LEVELING_LINEAR
//#define AUTO_BED_LEVELING_BILINEAR
//#define AUTO_BED_LEVELING_3POINT

// enable a graphics overly while editing the mesh from auto-level
//#define MESH_EDIT_GFX_OVERLAY

// Mesh inset margin on print area
#define MESH_INSET 10

// Enable the G26 Mesh Validation Pattern tool.
//#define G26_MESH_VALIDATION
// (Values in mm)
// Diameter of primary nozzle.
#define MESH_TEST_NOZZLE_SIZE    0.4
// Default layer height for the G26 Mesh Validation Tool.
#define MESH_TEST_LAYER_HEIGHT   0.2
// Default nozzle temperature for the G26 Mesh Validation Tool.
#define MESH_TEST_HOTEND_TEMP  200.0
// Default bed temperature for the G26 Mesh Validation Tool.
#define MESH_TEST_BED_TEMP      60.0

// Default mesh area is an area with an inset margin on the print area.
// Below are the macros that are used to define the borders for the mesh
// area, made available here for specialized needs.
#define MESH_MIN_X (X_MIN_POS + (MESH_INSET))
#define MESH_MAX_X (X_MAX_POS - (MESH_INSET))
#define MESH_MIN_Y (Y_MIN_POS + (MESH_INSET))
#define MESH_MAX_Y (Y_MAX_POS - (MESH_INSET))

// After homing all axes ('G28' or 'G28 XYZ') rest Z at Z MIN POS
//#define MESH_G28_REST_ORIGIN


/***********************************************************************
 * VALID FOR UNIFIED BED LEVELING                                      */
// Sophisticated users prefer no movement of nozzle
#define UBL_MESH_EDIT_MOVES_Z

// When the nozzle is off the mesh, this value is used as
// Z-Height correction value.
//#define UBL_Z_RAISE_WHEN_OFF_MESH 2.5
 /**********************************************************************/

  /**********************************************************************
  * VALID FOR:                                                          *
  *    MESH BED LEVELING                                                *
  *    AUTO BED LEVELING LINEAR                                         *
  *    AUTO BED LEVELING BILINEAR                                       *
  *   UNIFIED BED LEVELING                                              *
  *                                                                     *
  * Set number of grid points per dimension                             */
 #define GRID_MAX_POINTS_X 3
 #define GRID_MAX_POINTS_Y 3

  /**********************************************************************
  * VALID FOR:                                                          *
  *    AUTO BED LEVELING LINEAR                                         *
  *    AUTO BED LEVELING BILINEAR                                       *
  *                                                                     *
  * Set the boundaries for probing (where the probe can reach).         *
  ***********************************************************************/
 #define LEFT_PROBE_BED_POSITION 20
 #define RIGHT_PROBE_BED_POSITION 180
 #define FRONT_PROBE_BED_POSITION 20
 #define BACK_PROBE_BED_POSITION 180

 // Z probe minimum outer margin (to validate G29 parameters).
 #define MIN_PROBE_EDGE 10

 // Probe along Y axis, advancing X after each column
 //#define PROBE_Y_FIRST

 // Experimental Subdivision of the grid by Catmull-Rom method.
 // Synthesizes intermediate points to produce a more detailed mesh.
 //#define ABL_BILINEAR_SUBDIVISION
 // Number of subdivisions between probe points
 #define BILINEAR_SUBDIVISIONS 3

  /**********************************************************************/
  /**********************************************************************
  * VALID FOR:                                                          *
  *    AUTO_BED_LEVELING_3POINT                                         *
  *    UNIFIED BED LEVELING                                             *
  *                                                                     *
  *  3 arbitrary points to probe.                                       *
  *                                                                     *
  * A simple cross-product is used to estimate plane of the bed.        *
  ***********************************************************************/
 #define PROBE_PT_1_X 15
 #define PROBE_PT_1_Y 180
 #define PROBE_PT_2_X 15
 #define PROBE_PT_2_Y 15
 #define PROBE_PT_3_X 180
 #define PROBE_PT_3_Y 15
  /**********************************************************************/

  /**********************************************************************
  * Commands to execute at the end of G29 probing.                      *
  * Useful to retract or move the Z probe out of the way.               *
  ***********************************************************************/
 //#define Z_PROBE_END_SCRIPT "G1 Z10 F8000\nG1 X10 Y10\nG1 Z0.5"

 /**********************************************************************
 *  Leveling Fade Height                                               *
 ***********************************************************************
 * VALID FOR:                                                          *
 *    MESH BED LEVELING                                                *
 *    AUTO_BED_LEVELING_BILINEAR                                       *
 *                                                                     *
 * Gradually reduce leveling correction until a set height is reached, *
 * at which point movement will be level to the machine's XY plane.    *
 * and no correction is applied anymore.                               *
 * The height can be set with M420 Z<height>                           *
 *                                                                     *
 ***********************************************************************/
//#define ENABLE_LEVELING_FADE_HEIGHT

 /**********************************************************************
 *  Manual home positions                                              *
 ***********************************************************************/
// Center of bed is at (X=0, Y=0)
//#define BED_CENTER_AT_0_0

// Manually set home position.
// Leave these undefined for automatic settings.
//#define MANUAL_X_HOME_POS 0
//#define MANUAL_Y_HOME_POS 0
//#define MANUAL_Z_HOME_POS 0

/**********************************************************************
*  Movement Settings                                                  *
***********************************************************************
*                                                                     *
* Default Settings                                                    *
* These settings can be reset issuing a M502 command                  *
*                                                                     *
* Note: if EEPROM is enabled, saved values will override these.       *
*                                                                     *
***********************************************************************/

/**********************************************************************
*  Axis steps per unit                                                *
***********************************************************************
*                                                                     *
* Default Axis Steps Per Unit (steps/mm)                              *
* Override with M92                                                   *
*                                                                     *
***********************************************************************/
// Default steps per unit               X,  Y,    Z,  E0...(per extruder)
#define DEFAULT_AXIS_STEPS_PER_UNIT   {80, 80, 3200, 625, 625, 625, 625}

/**********************************************************************
*  Axis feedrate                                                      *
***********************************************************************
*                                                                     */
//                                  X,   Y, Z,  E0...(per extruder). (mm/sec)
#define DEFAULT_MAX_FEEDRATE     {300, 300, 2, 100, 100, 100, 100}
// Feedrates for manual moves         X,     Y,     Z,  E from panel
#define MANUAL_FEEDRATE          {50*60, 50*60, 4*60, 10*60}
// Minimum feedrate
#define DEFAULT_MIN_FEEDRATE          0.0
#define DEFAULT_MIN_TRAVEL_FEEDRATE   0.0
// Minimum planner junction speed. Sets default minimum speed planner plans
// for at end of the buffer and all stops. This should not be much greater
// than zero and should only be changed if unwanted behavior is observed on
// a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED         0.05                      // (mm/sec)

/**********************************************************************
*  Axis acceleration                                                  *
***********************************************************************
*                                                                     */
//  Maximum start speed for accelerated moves.    X,    Y,  Z,   E0...(per extruder)
#define DEFAULT_MAX_ACCELERATION              {3000, 3000, 50, 1000, 1000, 1000, 1000}
//  Maximum acceleration in mm/s^2 for retracts   E0... (per extruder)
#define DEFAULT_RETRACT_ACCELERATION          {10000, 10000, 10000, 10000}
//  X, Y, Z and E* maximum acceleration in mm/s^2 for printing moves
#define DEFAULT_ACCELERATION          3000
//  X, Y, Z acceleration in mm/s^2 for travel (non printing) moves
#define DEFAULT_TRAVEL_ACCELERATION   3000

/**********************************************************************
*  Axis jerk                                                          *
***********************************************************************
*                                                                     *
*                                                                     *
* Default Jerk (mm/s)                                                 *
* Override with M205 X Y Z E                                          *
*                                                                     *
* "Jerk" specifies minimum speed change that requires acceleration.   *
* When changing speed and direction, if the difference is less than   *
* value set here, it may happen instantaneously.                      *
*                                                                     *
***********************************************************************/
#define DEFAULT_XJERK 10.0
#define DEFAULT_YJERK 10.0
#define DEFAULT_ZJERK  0.4
// E0... (mm/sec) per extruder
#define DEFAULT_EJERK                   {5.0, 5.0, 5.0, 5.0}

/**********************************************************************
*  Homing feedrate                                                    *
***********************************************************************
* Values in (mm/m)                                                    *
***********************************************************************/
#define HOMING_FEEDRATE_X (50*60)
#define HOMING_FEEDRATE_Y (50*60)
#define HOMING_FEEDRATE_Z (2*60)

/**********************************************************************
*  HOME BUMP                                                          *
***********************************************************************
* Values in mm                                                        *
* During homing when and endstop is hit, printer will retracts by     *
* distance sepcified by this value and then does a slower bump.       *
* Slower speed is calculated using HOMING_BUMP_DIVISOR (X, Y, Z)      *
* These values divide Homing Feedrate by these amount                 *
* i.e. if homing feedrate fo X is (50*60) == 3000 mm/m                *
* re-bumoping speed is 3000/5 = 600 mm/m                              *
***********************************************************************/
#define X_HOME_BUMP_MM 5
#define Y_HOME_BUMP_MM 5
#define Z_HOME_BUMP_MM 2

#define HOMING_BUMP_DIVISOR {5, 5, 2}

/**********************************************************************
*  HOTEND OFFSETS                                                     *
*                                                                     *
* Offset of the hotends (uncomment if using more than one and using   *
* firmware to position when changing).                                *
*
*
* Offset has to be X=0, Y=0, Z=0 for hotend 0 (default hotend).       *
* Other values are relative to the HE 0.                              *
*                                                                     *
* Values are expressed as (HE0, HE1, HE2, HE3)                        *
* with HE0 ALWAYS = 0 for each axis.                                  *
* Values are in mm                                                    *
***********************************************************************/

#define HOTEND_OFFSET_X {0.0, 0.0, 0.0, 0.0}
#define HOTEND_OFFSET_Y {0.0, 0.0, 0.0, 0.0}
#define HOTEND_OFFSET_Z {0.0, 0.0, 0.0, 0.0}

/**********************************************************************
*  Hysteresis Feature                                                 *
*                                                                     *
* TODO: Check if it conflits with core mechanics                      *
* Hysteresis:                                                         *
* These are extra distances that are performed when an axis changes   *
* direction, they are used to compensate for any possible mechanical  *
* hysteresis your printer could have.                                 *
* HYSTERESIS_AXIS_MM    {X, Y, Z}                                     *
* These parameters could be modified with:                            *
* M99 X<mm> Y<mm> Z<mm>                                               *
*                                                                     *
***********************************************************************/
//#define HYSTERESIS_FEATURE

#define HYSTERESIS_AXIS_MM    { 0, 0, 0 }
// 0.0 = no correction; 1.0 = full correction
#define HYSTERESIS_CORRECTION 0.0