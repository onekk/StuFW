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
 * temperature.h - temperature controller
 */

class Temperature {

  public: /** Constructor */

    Temperature() {};

  public: /** Public Parameters */

    #if HAS_MCU_TEMPERATURE
      static float    mcu_current_temperature,
                      mcu_highest_temperature,
                      mcu_lowest_temperature,
                      mcu_alarm_temperature;
      static int16_t  mcu_current_temperature_raw;
    #endif

    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      static float    redundant_temperature;
    #endif

    #if HAS_EXTRUDERS && ENABLED(PREVENT_COLD_EXTRUSION)
      static int16_t extrude_min_temp;
    #endif

  private: /** Private Parameters */

    static uint8_t pid_pointer;

    static millis_t next_check_ms[HEATER_COUNT];

    #if ENABLED(FILAMENT_SENSOR)
      static int8_t   meas_shift_index;     // Index of a delayed sample in buffer
      static uint16_t current_raw_filwidth; // Measured filament diameter - one extruder only
    #endif

    #if ENABLED(PROBING_HEATERS_OFF)
      static bool paused;
    #endif

  public: /** Public Function */

    /**
     * Initialize the temperature manager
     */
    static void init();

    /**
     * Called from the Temperature ISR
     */
    static void set_current_temp_raw();

    /**
     * Call periodically to HAL isr
     */
    static void spin();

    /**
     * Perform auto-tuning for hotend, bed, chamber or cooler in response to M303
     */
    static void PID_autotune(Heater *act, const float target_temp, const uint8_t ncycles, const uint8_t method, const bool storeValues=false);

    /**
     * Switch off all heaters, set all target temperatures to 0
     */
    static void disable_all_heaters();

    /**
     * Check if there are heaters Active
     */
    static bool heaters_isActive();

    #if ENABLED(SUPPORT_MAX6675) || ENABLED(SUPPORT_MAX31855)
      static void getTemperature_SPI();
    #endif

    #if HAS_FILAMENT_SENSOR
      static int8_t widthFil_to_size_ratio(); // Convert Filament Width (mm) to an extrusion ratio
    #endif    

    #if ENABLED(PROBING_HEATERS_OFF)
      static void pause(const bool p);
      static bool is_paused() { return paused; }
    #endif

    static void report_temperatures(const bool showRaw=false);

    #if HAS_EXTRUDERS && ENABLED(PREVENT_COLD_EXTRUSION)
      FORCE_INLINE static bool tooCold(const int16_t temp) {
        return printer.isAllowColdExtrude() ? false : temp < extrude_min_temp;
      }
      FORCE_INLINE static bool tooColdToExtrude(const uint8_t h) {
        #if HOTENDS <= 1
          UNUSED(h);
        #endif
        return tooCold(heaters[HOTEND_INDEX].current_temperature);
      }
      FORCE_INLINE static bool targetTooColdToExtrude(const uint8_t h) {
        #if HOTENDS == 1
          UNUSED(h);
        #endif
        return tooCold(heaters[HOTEND_INDEX].target_temperature);
      }
    #else
      FORCE_INLINE static bool tooColdToExtrude(const uint8_t h) { UNUSED(h); return false; }
      FORCE_INLINE static bool targetTooColdToExtrude(const uint8_t h) { UNUSED(h); return false; }
    #endif

    FORCE_INLINE static bool hotEnoughToExtrude(const uint8_t h) { return !tooColdToExtrude(h); }
    FORCE_INLINE static bool targetHotEnoughToExtrude(const uint8_t h) { return !targetTooColdToExtrude(h); }

  private:

    #if HAS_FILAMENT_SENSOR
      static float analog2widthFil(); // Convert raw Filament Width to millimeters
    #endif

    #if HAS_MCU_TEMPERATURE
      static float analog2tempMCU(const int raw);
    #endif

    static void _temp_error(const uint8_t h, PGM_P const serial_msg, PGM_P const lcd_msg);
    static void min_temp_error(const uint8_t h);
    static void max_temp_error(const uint8_t h);

    #if HEATER_COUNT > 0
      static void print_heater_state(Heater *act, const bool print_ID, const bool showRaw);
    #endif

};

extern Temperature thermalManager;
