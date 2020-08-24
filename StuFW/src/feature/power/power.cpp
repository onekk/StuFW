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

#include "../../../StuFW.h"

#if HAS_POWER_SWITCH || HAS_POWER_CHECK

  Power powerManager;

  /** Public Parameters */
  flagpower_t Power::flag;

  /** Private Parameters */
  #if HAS_POWER_SWITCH
    bool      Power::powersupply_on = false;
    #if (POWER_TIMEOUT > 0)
      watch_t Power::watch_lastPowerOn(POWER_TIMEOUT * 1000UL);
    #endif
  #endif

  /** Public Function */
  #if HAS_POWER_SWITCH || HAS_POWER_CHECK

    /** Public Function */
    void Power::init() {
      #if HAS_POWER_CHECK
        SET_INPUT(POWER_CHECK_PIN);
      #endif
      #if HAS_POWER_SWITCH
        #if PS_DEFAULT_OFF
          power_off();
        #else
          power_on();
        #endif
      #endif
    }

    #if HAS_POWER_CHECK

      void Power::factory_parameters() {
        setLogic(POWER_CHECK_LOGIC);
        setPullup(PULLUP_POWER_CHECK);
      }

      void Power::setup_pullup() {
        HAL::setInputPullup(POWER_CHECK_PIN, isPullup());
      }

      void Endstops::report() {
        SERIAL_LOGIC("POWER CHECK Logic", isLogic());
        SERIAL_LOGIC(" Pullup", isPullup());
      }

    #endif 

    void Power::spin() {
      if (is_power_needed())
        power_on();
      #if (POWER_TIMEOUT > 0)
        else if (watch_lastPowerOn.elapsed())
          power_off();
      #endif
    }

    void Power::power_on() {
      #if (POWER_TIMEOUT > 0)
        watch_lastPowerOn.start();
      #endif
      if (!powersupply_on) {
        OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
        powersupply_on = true;
        HAL::delayMilliseconds((DELAY_AFTER_POWER_ON) * 1000UL);
      }
    }

    void Power::power_off() {
      if (powersupply_on) {
        OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
        powersupply_on = false;
        #if (POWER_TIMEOUT > 0)
          watch_lastPowerOn.stop();
        #endif
      }
    }

    bool Power::is_power_needed() {

      #if HEATER_COUNT > 0
        if (thermalManager.heaters_isActive()) return true;
      #endif

      #if FAN_COUNT > 0
        LOOP_FAN() if (fans[f].Speed > 0) return true;
      #endif

      if (X_ENABLE_READ() == X_ENABLE_ON || Y_ENABLE_READ() == Y_ENABLE_ON || Z_ENABLE_READ() == Z_ENABLE_ON
          || E0_ENABLE_READ() == E_ENABLE_ON // If any of the drivers are enabled...
          #if DRIVER_EXTRUDERS > 1
            || E1_ENABLE_READ() == E_ENABLE_ON
            #if HAS_X2_ENABLE
              || X2_ENABLE_READ() == X_ENABLE_ON
            #endif
            #if DRIVER_EXTRUDERS > 2
              || E2_ENABLE_READ() == E_ENABLE_ON
              #if DRIVER_EXTRUDERS > 3
                || E3_ENABLE_READ() == E_ENABLE_ON
                #if DRIVER_EXTRUDERS > 4
                  || E4_ENABLE_READ() == E_ENABLE_ON
                  #if DRIVER_EXTRUDERS > 5
                    || E5_ENABLE_READ() == E_ENABLE_ON
                  #endif
                #endif
              #endif
            #endif
          #endif
      ) return true;

      return false;
    }

  #endif // HAS_POWER_SWITCH


#endif // HAS_POWER_SWITCH || HAS_POWER_CHECK
