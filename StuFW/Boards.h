/**
 * StuFW Firmware for 3D Printer
 *
 * Based on MK4Duo, Marlin, Sprinter and grbl
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
#pragma once

// Macros for board type
#define BOARD_UNKNOWN -1
#define MB(board) (MOTHERBOARD==BOARD_##board)

/**
 * RAMPS 1.3 / 1.4 - ATmega1280, ATmega2560
 */
#define BOARD_RAMPS_OLD          3    // MEGA/RAMPS up to 1.2
#define BOARD_RAMPS_13_HFB      33    // RAMPS 1.3 / 1.4 (Power outputs: Hotend, Fan, Bed)
#define BOARD_RAMPS_13_HHB      34    // RAMPS 1.3 / 1.4 (Power outputs: Hotend0, Hotend1, Bed)
#define BOARD_RAMPS_13_HFF      35    // RAMPS 1.3 / 1.4 (Power outputs: Hotend, Fan, Fan)
#define BOARD_RAMPS_13_HHF      36    // RAMPS 1.3 / 1.4 (Power outputs: Hotend0, Hotend1, Fan)
#define BOARD_RAMPS_13_HHH      37    // RAMPS 1.3 / 1.4 (Power outputs: Hotend0, Hotend1, Hotend2)

/**
 * RAMPS Derivates - ATmega1280, ATmega2560
 */
#define BOARD_MKS_13            47    // MKS GEN v1.3 or 1.4

/**
 * Custom Boards
 */
#define BOARD_99                99    // Custom Board
