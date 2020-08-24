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

#ifndef _CONFIGURATION_VERSION_H_
#define _CONFIGURATION_VERSION_H_

#define FIRMWARE_NAME             "StuFW"
#define SHORT_BUILD_VERSION       "0.0.6"
#define FIRMWARE_REVISION         "20201128"
#define BUILD_VERSION             FIRMWARE_NAME "_" SHORT_BUILD_VERSION
#define STRING_DISTRIBUTION_DATE  __DATE__ " " __TIME__    // build date and time
#define FIRMWARE_URL              "none"

#endif /* _CONFIGURATION_VERSION_H_ */
