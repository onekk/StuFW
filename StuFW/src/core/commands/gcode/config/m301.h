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
 * mcode
 */

#define CODE_M301

/**
 * M301: Set PID parameters P I D (and optionally C, L)
 *
 *   H[heaters] H = 0-3 Hotend, H = -1 BED, H = -2 CHAMBER
 *
 *   P[float] Kp term
 *   I[float] Ki term
 *   D[float] Kd term
 *
 * With PID_ADD_EXTRUSION_RATE:
 *
 *   C[float] Kc term
 *   L[int] LPQ length
 */
inline void gcode_M301(void) {

  int8_t h = 0;

  if (!commands.get_target_heater(h)) return;

  Heater *act = &heaters[h];

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M301 report.
    if (!parser.seen("PIDCL")) {
      act->print_M301();
      return;
    }
  #endif

  if (parser.seen('P')) act->pid.Kp = parser.value_float();
  if (parser.seen('I')) act->pid.Ki = parser.value_float();
  if (parser.seen('D')) act->pid.Kd = parser.value_float();
  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    if (parser.seen('C')) act->pid.Kc = parser.value_float();
    if (parser.seen('L')) tools.lpq_len = parser.value_int();
    NOMORE(tools.lpq_len, LPQ_MAX_LEN);
    NOLESS(tools.lpq_len, 0);
  #endif

  act->pid.update();
  act->setTuning(true);
  act->ResetFault();

}
