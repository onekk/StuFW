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

// Models
#define A4988               0x001
#define A5984               0x002
#define DRV8825             0x003


// Type
#define AXIS_DRV_TYPE(A,T)  (A##_DRIVER_TYPE == T)
#define  X_HAS_DRV(TYPE)    (AXIS_DRV_TYPE(X,TYPE))
#define  Y_HAS_DRV(TYPE)    (AXIS_DRV_TYPE(Y,TYPE))
#define  Z_HAS_DRV(TYPE)    (AXIS_DRV_TYPE(Z,TYPE))
#define X2_HAS_DRV(TYPE)    (ENABLED(X_TWO_STEPPER_DRIVERS) && AXIS_DRV_TYPE(X2,TYPE))
#define Y2_HAS_DRV(TYPE)    (ENABLED(Y_TWO_STEPPER_DRIVERS) && AXIS_DRV_TYPE(Y2,TYPE))
#define Z2_HAS_DRV(TYPE)    (ENABLED(Z_TWO_STEPPER_DRIVERS) && AXIS_DRV_TYPE(Z2,TYPE))
#define E0_HAS_DRV(TYPE)    (DRIVER_EXTRUDERS > 0 && AXIS_DRV_TYPE(E0,TYPE))
#define E1_HAS_DRV(TYPE)    (DRIVER_EXTRUDERS > 1 && AXIS_DRV_TYPE(E1,TYPE))
#define E2_HAS_DRV(TYPE)    (DRIVER_EXTRUDERS > 2 && AXIS_DRV_TYPE(E2,TYPE))
#define E3_HAS_DRV(TYPE)    (DRIVER_EXTRUDERS > 3 && AXIS_DRV_TYPE(E3,TYPE))
#define E4_HAS_DRV(TYPE)    (DRIVER_EXTRUDERS > 4 && AXIS_DRV_TYPE(E4,TYPE))
#define E5_HAS_DRV(TYPE)    (DRIVER_EXTRUDERS > 5 && AXIS_DRV_TYPE(E5,TYPE))

#define HAVE_DRV(TYPE) ( \
            X_HAS_DRV(TYPE) ||  Y_HAS_DRV(TYPE) ||  Z_HAS_DRV(TYPE)   \
        || X2_HAS_DRV(TYPE) || Y2_HAS_DRV(TYPE) || Z2_HAS_DRV(TYPE)   \
        || E0_HAS_DRV(TYPE) || E1_HAS_DRV(TYPE) || E2_HAS_DRV(TYPE)   \
        || E3_HAS_DRV(TYPE) || E4_HAS_DRV(TYPE) || E5_HAS_DRV(TYPE))
