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

union flagpower_t {
  bool all;
  struct {
    bool  Logic   : 1;
    bool  Pullup  : 1;
    bool  bit2    : 1;
    bool  bit3    : 1;
    bool  bit4    : 1;
    bool  bit5    : 1;
    bool  bit6    : 1;
    bool  bit7    : 1;
  };
  flagpower_t() { all = false; }
};

#if HAS_POWER_SWITCH

  class Power {

    public: /** Constructor */

    Power() {};

    public: /** Public Parameters */

      static flagpower_t flag;

    private: /** Private Parameters */

      static bool powersupply_on;
      #if (POWER_TIMEOUT > 0)
        static watch_t watch_lastPowerOn;
      #endif

    public: /** Public Function */

      /**
      * Initialize the Power switch
      */
      static void init();

      static void spin();
      static void power_on();
      static void power_off();

      FORCE_INLINE static bool is_on() { return powersupply_on; }

    private: /** Private Function */

      static bool is_power_needed();
  };

  extern Power powerManager;

#endif // HAS_POWER_SWITCH
