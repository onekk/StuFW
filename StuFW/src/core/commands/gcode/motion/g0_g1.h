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
 * G0, G1: Coordinated movement of X Y Z E axes
 */
inline void gcode_G0_G1() {
  if (printer.isRunning()) {
    commands.get_destination(); // For X Y Z E F

    #if ENABLED(FWRETRACT)
      if (MIN_AUTORETRACT <= MAX_AUTORETRACT) {
        // When M209 Autoretract is enabled, convert E-only moves to firmware retract/recover moves
        if (fwretract.autoretract_enabled && parser.seen('E') && !(parser.seen('X') || parser.seen('Y') || parser.seen('Z'))) {
          const float echange = mechanics.destination[E_AXIS] - mechanics.current_position[E_AXIS];
          // Is this move an attempt to retract or recover?
          if (WITHIN(ABS(echange), MIN_AUTORETRACT, MAX_AUTORETRACT) && fwretract.retracted[tools.active_extruder] == (echange > 0.0)) {
            mechanics.current_position[E_AXIS] = mechanics.destination[E_AXIS]; // Hide a G1-based retract/recover from calculations
            mechanics.sync_plan_position_e();                                   // AND from the planner
            return fwretract.retract(echange < 0.0);                            // Firmware-based retract/recover (double-retract ignored)
          }
        }
      }
    #endif // FWRETRACT

    mechanics.prepare_move_to_destination();

  }
}
