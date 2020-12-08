/**
 * StuFW Firmware for 3D Printer
 *
 * Based on MK4duo, Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
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
 * mcode
 */

#if HAS_MULTI_ENDSTOP

  #define CODE_M666

  /**
   * M666: Set Two Endstops offsets for X, Y, and/or Z.
   *
   *        X = X2 Endstop Adjust
   *        Y = Y2 Endstop Adjust
   *        Z = Z2 Endstop Adjust
   */
  inline void gcode_M666(void) {

    SERIAL_MSG("Dual Endstop Adjustment (mm): ");
    #if ENABLED(X_TWO_ENDSTOPS)
      if (parser.seen('X')) endstops.x2_endstop_adj = parser.value_linear_units();
      SERIAL_MV(" X2:", endstops.x2_endstop_adj);
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      if (parser.seen('Y')) endstops.y2_endstop_adj = parser.value_linear_units();
      SERIAL_MV(" Y2:", endstops.y2_endstop_adj);
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS)
      if (parser.seen('Z')) endstops.z2_endstop_adj = parser.value_linear_units();
      SERIAL_MV(" Z2:", endstops.z2_endstop_adj);
    #endif
    SERIAL_EOL();
  }

#endif
