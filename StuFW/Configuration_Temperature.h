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
 */

/**
 * Configuration_Temperature.h
 *
 * This configuration file contains temperature settings.
 *
 * - Temperature Units support
 * - Thermistor type
 * - Temperature limits
 * - Automatic temperature
 * - Temperature status LEDs
 * - PWM Heater Speed
 * - PID Settings - HOTEND
 * - PID Settings - BED
 * - PID Settings - CHAMBER
 * - Inverted PINS
 * - Thermal runaway protection
 * - Prevent cold extrusion
 *
 */

#ifndef _CONFIGURATION_TEMPERATURE_H_
#define _CONFIGURATION_TEMPERATURE_H_

/************************************************************************************
 ******************************** Temperature Units Support *************************
 ************************************************************************************
 *                                                                                  *
 * Enable TEMPERATURE UNITS SUPPORT for change unit with command Gcode M149         *
 *                                                                                  *
 ************************************************************************************/
//#define TEMPERATURE_UNITS_SUPPORT
/************************************************************************************/


/******************************************************************************
 ************************************** Thermistor type ***********************
 ******************************************************************************
 *                                                                            *
 * Please choose the one that matches your setup and set to TEMP SENSOR.      *
 *                                                                            *
 *  -4 is thermocouple with MAX31855                                          *
 *  -3 is thermocouple with MAX6675                                           *
 *  -2 is thermocouple with AD8495                                            *
 *  -1 is thermocouple with AD595 or AD597                                    *
 *   0 means sensor not used                                                  *
 *   1 - 9   NTC Thermistors, put the apprpropriate values                    *
 *                                                                            *
 *   THIS PART OF SETTING IS DIFFERENT FROM ORIGINAL MK4duo WAY               *
 *                                                                            *
 *   Mk4duo permits only ONE user sensor settings here, T9                    *
 *   StuFW could use a different NTC setting for each sensor                  *
 *   Some spare places reamins due to the way the original MK4duo code        *
 *   was taylored, no change in code, for semplicity and has it didn't make   *
 *   no harm to memory or to code flow readability                            *
 *                                                                            *
 *   You have to choose a number and put the appropriate values from the      *
 *   table below.                                                             *
 *                                                                            *
 *   According from this data obtained from original data in MK4Duo           *
 *   NAME         RS     R25      BETA       NTC Type                         *
 *   EPCOS 100k   4700   100000    4092.0                                     *
 *   NTC3950      4700   100000    3950.0  - Good start for a generic sensor  *
 *   ATC 204GT-2  4700   200000    4338.0  - ATC Semitec 204GT-2              *
 *   ATC 104GT-2  4700   100000    4725.0  - ATC Semitec 104GT-2              *
 *                                          (Used in ParCan & J-Head)         *
 *   HW 104LAG    4700   100000    3974.0  - Honeywell 135-104LAG-J01         *
 *   E3104FXT     4700   100000    4100.0  - Vishay NTCS0603E3104FXT          *
 *   GE AL03006   4700   100000    3952.0  - GE Sensing AL03006-58.2K-97-G1   *
 *   RS 198-961   4700   100000    3960.0  - RS thermistor 198-961            *
 *                                                                            *
 *  20 is the PT100 circ. amplif. found in Ultimainboard V2.x and Wanhao D6   *
 *                                                                            *
 *       Use these for Testing or Development purposes. chine.                *
 * 998 : Dummy Table that ALWAYS reads  25 degC or temperature defined below. *
 * 999 : Dummy Table that ALWAYS reads 100 degC or temperature defined below. *
 *                                                                            *
 *  Example:                                                                  *
 *                                                                            *
 *  Suppose you have two extruder and a bed and three different sensors       *
 *  with MK4duo you are limited to choose one sensor from the first 8         *
 *  standard values, and use T9 values for on of the remaining sensors.       *
 *                                                                            *
 *  with StuFW you are not limited, it is sufficient to choose a number       *
 *  from 1 to 9 and assign it to the input pin                                *
 *  and assign proper values                                                  *
 *                                                                            *
 *    You have say an ATC Semitec 204GT-2     for HE0                         *
 *    #define TEMP_SENSOR_0 1                                                 *
 *    #define T1_NAME   "ATC 204GT-2"                                         *
 *    #define T1_R25    200000.0                                              *
 *    #define T1_BETA     4338.0                                              *
 *                                                                            *
 *    You have an EPCOS 100k  on HE1                                          *
 *    #define TEMP_SENSOR_0 2                                                 *
 *    #define T2_NAME   "EPCOS 100k"                                          *
 *    #define T2_R25    100000.0                                              *
 *    #define T2_BETA     4092.0                                              *
 *                                                                            *
 *    You have a generic 3950 sensor for bed                                  *
 *    #define TEMP_SENSOR_BED 3                                               *
 *    #define T3_NAME   "BED SENSOR"                                          *
 *    #define T3_R25    100000.0                                              *
 *    #define T3_BETA     3950.0                                              *
 *                                                                            *
 * As you have 6 different thermal sensors defined in firmware                *
 * only Tx_NAME, Tx_R25, Tx_BETA are defined here.                            *
 * for future development (if there where enough ADC pins left on the Mega)   *
 * spares defines from 7 to 9 are left in the original place:                 *
 *     src/core/heater/sensor/thermistor.h                                    *
 *                                                                            *
 ******************************************************************************/
#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_BED 0
#define TEMP_SENSOR_CHAMBER 0

// Thermistor series resistor value in Ohms (see on your board)
#define THERMISTOR_SERIES_RS 4700.0

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

// These 2 defines help to calibrate the AD595 sensor in case you get wrong
// temperature measurements.
// Measured temperature is defined as:
// actualTemp = (measuredTemp * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0

// Use it for Testing or Development purposes. NEVER for production machine.
#define DUMMY_THERMISTOR_998_VALUE  25
#define DUMMY_THERMISTOR_999_VALUE 100
/*******************************************************************************/


/******************************************************************************************************
 ************************************** Temperature limits ********************************************
 ******************************************************************************************************/
// Temperature must be close to target for this long before M109-M190-M191-M192 returns success
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally,
// but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 275 // (degC)
#define HEATER_1_MAXTEMP 275 // (degC)
#define HEATER_2_MAXTEMP 275 // (degC)
#define HEATER_3_MAXTEMP 275 // (degC)
#define BED_MAXTEMP      150 // (degC)
#define CHAMBER_MAXTEMP  100 // (degC)

// The minimal temperature defines temperature below which heater will not be enabled
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5 // (degC)
#define HEATER_1_MINTEMP 5 // (degC)
#define HEATER_2_MINTEMP 5 // (degC)
#define HEATER_3_MINTEMP 5 // (degC)
#define BED_MINTEMP      5 // (degC)
#define CHAMBER_MINTEMP  5 // (degC)

// Preheat Constants
#define PREHEAT_1_LABEL       "PLA"
#define PREHEAT_1_TEMP_HOTEND 190
#define PREHEAT_1_TEMP_BED     60
#define PREHEAT_1_FAN_SPEED   255   // Insert Value between 0 and 255

#define PREHEAT_2_LABEL       "ABS"
#define PREHEAT_2_TEMP_HOTEND 240
#define PREHEAT_2_TEMP_BED    100
#define PREHEAT_2_FAN_SPEED   255   // Insert Value between 0 and 255

#define PREHEAT_3_LABEL       "GUM"
#define PREHEAT_3_TEMP_HOTEND 230
#define PREHEAT_3_TEMP_BED     60
#define PREHEAT_3_FAN_SPEED   255   // Insert Value between 0 and 255
/*****************************************************************************************/


/*****************************************************************************************
 ******************************** Automatic temperature **********************************
 *****************************************************************************************
 *                                                                                       *
 * The hotend target temperature is calculated by all the buffered lines of gcode.       *
 * The maximum buffered steps/sec of the extruder motor is called "se".                  *
 * Start autotemp mode with M109 S<mintemp> B<maxtemp> F<factor>                         *
 * The target temperature is set to mintemp+factor*se[steps/sec] and is limited by       *
 * mintemp and maxtemp. Turn this off by excuting M109 without F*                        *
 * Also, if the temperature is set to a value below mintemp, it will not be changed      *
 * by autotemp.                                                                          *
 * On an Ultimaker, some initial testing worked with M109 S215 B260 F1                   *
 * in the start.gcode                                                                    *
 *                                                                                       *
 *****************************************************************************************/
//#define AUTOTEMP
#define AUTOTEMP_OLDWEIGHT 0.98
/*****************************************************************************************/


/***********************************************************************
 ********************* Temperature status LEDs *************************
 ***********************************************************************
 *                                                                     *
 * Temperature status LEDs that display the hotend and bed             *
 * temperature.                                                        *
 * Otherwise the RED led is on. There is 1C hysteresis.                *
 *                                                                     *
 ***********************************************************************/
//#define TEMP_STAT_LEDS
/***********************************************************************/


/***********************************************************************
 ********************** PID Settings - HOTEND **************************
 ***********************************************************************
 *                                                                     *
 * PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning            *
 *                                                                     *
 ***********************************************************************/
// Put to false following line to disable PID and enable bang-bang.
#define PIDTEMP true

#define PID_MAX       255 // Limits current to nozzle while in PID mode;        255 = full current
#define PID_DRIVE_MIN  40 // Limits min current to nozzle while PID is active;    0 = no current
#define PID_DRIVE_MAX 230 // Limits max current to nozzle while PID is active;  255 = full current

// If the temperature difference between the target temperature and the actual temperature
// is more then PID FUNCTIONAL RANGE then the PID will be shut off and the heater will be set to min/max.
#define PID_FUNCTIONAL_RANGE 10

#define PID_AUTOTUNE_MENU // Add PID Autotune to the LCD "Temperature" menu to run M303 and apply the result.
//#define PID_DEBUG       // Sends debug data to the serial port.

// this adds an experimental additional term to the heating power, proportional to the extrusion speed.
// if Kc is chosen well, the additional required power due to increased melting should be compensated.
//#define PID_ADD_EXTRUSION_RATE
#define LPQ_MAX_LEN 50

//           HotEnd{HE0,HE1,HE2,HE3}
#define DEFAULT_Kp {40, 40, 40, 40}     // Kp for H0, H1, H2, H3
#define DEFAULT_Ki {07, 07, 07, 07}     // Ki for H0, H1, H2, H3
#define DEFAULT_Kd {60, 60, 60, 60}     // Kd for H0, H1, H2, H3
#define DEFAULT_Kc {100, 100, 100, 100} // heating power = Kc * (e_speed)
/***********************************************************************/


/***********************************************************************
 ************************ PID Settings - BED ***************************
 ***********************************************************************
 *                                                                     *
 * PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning            *
 * Select PID or bang-bang with PIDTEMPBED.                            *
 * If bang-bang, BED_HYSTERESIS will enable hysteresis                 *
 *                                                                     *
 ***********************************************************************/
// Put true to enable PID on the bed. It uses the same frequency PWM as the hotend.
// which is fine for driving a square wave into a resistive load and does not significantly impact you FET heating.
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater.
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
// shouldn't use bed PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.
#define PIDTEMPBED false

#define BED_HYSTERESIS        2 // Only disable heating if T>target+BED_HYSTERESIS and enable heating if T<target-BED_HYSTERESIS
#define BED_CHECK_INTERVAL 5000 // ms between checks in bang-bang control

// 120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
// from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)
#define DEFAULT_bedKp   10.00
#define DEFAULT_bedKi    0.1
#define DEFAULT_bedKd  300.0

// FIND YOUR OWN: "M303 H-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.
/***********************************************************************/


/***********************************************************************
 ************************ PID Settings - CHAMBER ***********************
 ***********************************************************************
 *                                                                     *
 * PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning            *
 * Select PID or bang-bang with PIDTEMPCHAMBER.                        *
 * If bang-bang, CHAMBER_LIMIT_SWITCHING will enable hysteresis        *
 *                                                                     *
 ***********************************************************************/
// Put true to enable PID on the chamber. It uses the same frequency PWM as the hotend.
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater.
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
// shouldn't use chamber PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.
#define PIDTEMPCHAMBER false

#define CHAMBER_HYSTERESIS        2 // only disable heating if T>target+CHAMBER_HYSTERESIS and enable heating if T<target-CHAMBER_HYSTERESIS
#define CHAMBER_CHECK_INTERVAL 5000 // ms between checks in bang-bang control

// This sets the max power delivered to the chamber.
// all forms of chamber control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 255 enables a form of PWM to the chamber,
// so you shouldn't use it unless you are OK with PWM on your chamber.  (see the comment on enabling PIDTEMPCHAMBER)
#define CHAMBER_PID_MAX       255   // Limits current to chamber while in PID mode;       255 = full current
#define CHAMBER_PID_DRIVE_MIN  80   // Limits min current to chamber while PID is active;   0 = no current
#define CHAMBER_PID_DRIVE_MAX 255   // Limits max current to chamber while PID is active; 255 = full current

// 120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
// from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)
#define DEFAULT_chamberKp   10.00
#define DEFAULT_chamberKi    0.1
#define DEFAULT_chamberKd  300.0
/***********************************************************************/

/*******************************************************************************
 **************************** Inverted PINS ************************************
 *******************************************************************************
 *                                                                             *
 * For inverted logical Heater, Bed, Chamber pins                              *
 *                                                                             *
 *******************************************************************************/
#define INVERTED_HEATER_PINS false
#define INVERTED_BED_PIN false
#define INVERTED_CHAMBER_PIN false


/**********************************************************************************
 ************************ Thermal runaway protection ******************************
 **********************************************************************************
 *                                                                                *
 * This protects your printer from damage and fire if a thermistor                *
 * falls out or temperature sensors fail in any way.                              *
 *                                                                                *
 * The issue: If a thermistor falls out or a temperature sensor fails,            *
 * StuFW can no longer sense the actual temperature. Since a                      *
 * disconnected thermistor reads as a low temperature, the firmware               *
 * will keep the heater on.                                                *
 *                                                                                *
 * The solution: Once the temperature reaches the target, start                   *
 * observing. If the temperature stays too far below the                          *
 * target(hysteresis) for too long, the firmware will halt                        *
 * as a safety precaution.                                                        *
 *                                                                                *
 * Put THERMAL PROTECTION HOTENDS at true to enable this feature for all hotends. *
 * Put THERMAL PROTECTION BED at true to enable this feature for the heated bed.  *
 * Put THERMAL PROTECTION CHAMBER at true to enable this feature for the chamber. *
 *                                                                                *
 **********************************************************************************/
#define THERMAL_PROTECTION_HOTENDS false
#define THERMAL_PROTECTION_BED false
#define THERMAL_PROTECTION_CHAMBER false

#define THERMAL_PROTECTION_PERIOD    40     // Seconds
#define THERMAL_PROTECTION_HYSTERESIS 4     // Degrees Celsius

// If thermal protection hotends is true, this parameter adapt fan speed if temperature drops
//#define ADAPTIVE_FAN_SPEED

/**
 * When ever increases the target temperature the firmware will wait for the
 * WATCH TEMP PERIOD to expire, and if the temperature hasn't increased by WATCH TEMP INCREASE
 * degrees, the machine is halted, requiring a hard reset.
 *
 * If you get false positives for "Heating failed" increase WATCH TEMP PERIOD and/or decrease WATCH TEMP INCREASE
 * WATCH TEMP INCREASE should not be below 2.
 */
#define WATCH_TEMP_PERIOD  20               // Seconds
#define WATCH_TEMP_INCREASE 2               // Degrees Celsius

#define WATCH_BED_TEMP_PERIOD 60
#define WATCH_BED_TEMP_INCREASE 2

#define WATCH_CHAMBER_TEMP_PERIOD 60
#define WATCH_CHAMBER_TEMP_INCREASE 2

/***********************************************************************/


/***********************************************************************
 ************************ Prevent cold extrusion ***********************
 ***********************************************************************
 * This option prevents extrusion if the temperature is                *
 * below EXTRUDE MINTEMP.                                              *
 * Add M302 to set the minimum extrusion temperature and/or turn       *
 * cold extrusion prevention on and off.                               *
 *                                                                     *
 *      IT IS HIGHLY RECOMMENDED TO LEAVE THIS OPTION ENABLED!         *
 *                                                                     *
 ***********************************************************************/
#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 170                 // Degree Celsius

// Prevent a single extrusion longer than EXTRUDE MAXLENGTH.
// Note: For Bowden Extruders make this large enough to allow load/unload.
//#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH + Y_MAX_LENGTH)
/***********************************************************************/

#endif /* _CONFIGURATION_TEMPERATURE_H_ */
