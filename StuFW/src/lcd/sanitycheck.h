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

/**
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#ifndef _LCD_SANITYCHECK_H_
#define _LCD_SANITYCHECK_H_

/**
 * Make sure only one display is enabled
 *
*/
static_assert(1 >= 0
  #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) \
      && DISABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    + 1
  #endif
  , "DEPENDENCY ERROR: Please select no more than one LCD controller option."
);

// Language
#if DISABLED(LCD_LANGUAGE)
  #error "DEPENDENCY ERROR: Missing setting LCD_LANGUAGE."
#endif

// Progress Bar
#if ENABLED(LCD_PROGRESS_BAR)
  #if HAS_GRAPHICAL_LCD
    #error "DEPENDENCY ERROR: LCD_PROGRESS_BAR does not apply to graphical displays."
  #elif ENABLED(FILAMENT_LCD_DISPLAY)
    #error "DEPENDENCY ERROR: LCD_PROGRESS_BAR and FILAMENT_LCD_DISPLAY are not fully compatible. Comment out this line to use both."
  #endif
#endif

// Progress bar
#if ENABLED(ULTIPANEL)
  #if ENABLED(LCD_PROGRESS_BAR)
    #if DISABLED(PROGRESS_BAR_BAR_TIME)
      #error "DEPENDENCY ERROR: Missing setting PROGRESS_BAR_BAR_TIME."
    #endif
    #if DISABLED(PROGRESS_BAR_MSG_TIME)
      #error "DEPENDENCY ERROR: Missing setting PROGRESS_BAR_MSG_TIME."
    #endif
    #if DISABLED(PROGRESS_MSG_EXPIRE)
      #error "DEPENDENCY ERROR: Missing setting PROGRESS_MSG_EXPIRE."
    #endif
  #endif
#endif

// LCD_BED_LEVELING requirements
#if ENABLED(LCD_BED_LEVELING)
  #if DISABLED(ULTIPANEL)
    #error "LCD_BED_LEVELING requires an LCD controller."
  #elif !(ENABLED(MESH_BED_LEVELING) || OLD_ABL)
    #error "LCD_BED_LEVELING requires MESH_BED_LEVELING or AUTO_BED_LEVELING."
  #endif
#endif

// Bootscreen
#if ENABLED(SHOW_BOOTSCREEN)
  #if DISABLED(STRING_SPLASH_LINE1)
    #error "DEPENDENCY ERROR: Missing setting STRING_SPLASH_LINE1."
  #endif
  #if DISABLED(BOOTSCREEN_TIMEOUT)
    #error "DEPENDENCY ERROR: Missing setting BOOTSCREEN_TIMEOUT."
  #endif
#endif

// Encoder rate multipliers
#if ENABLED(ULTIPANEL)
  #if ENABLED(ENCODER_RATE_MULTIPLIER)
    #if DISABLED(ENCODER_10X_STEPS_PER_SEC)
      #error "DEPENDENCY ERROR: Missing setting ENCODER_10X_STEPS_PER_SEC."
    #endif
    #if DISABLED(ENCODER_100X_STEPS_PER_SEC)
      #error "DEPENDENCY ERROR: Missing setting ENCODER_100X_STEPS_PER_SEC."
    #endif
  #endif
#endif

// Manual feedrate
#if ENABLED(ULTIPANEL) && DISABLED(MANUAL_FEEDRATE)
  #error "DEPENDENCY ERROR: Missing setting MANUAL_FEEDRATE."
#endif

// Required LCD language
#if DISABLED(DOGLCD) && HAS_SPI_LCD && DISABLED(DISPLAY_CHARSET_HD44780)
  #error "DEPENDENCY ERROR: You must set DISPLAY_CHARSET_HD44780 to JAPANESE, WESTERN or CYRILLIC for your LCD controller."
#endif

// ULTIPANEL encoder
#if ENCODER_PULSES_PER_STEP < 0
  #error "DEPENDENCY ERROR: ENCODER_PULSES_PER_STEP should not be negative, use REVERSE_MENU_DIRECTION instead."
#endif

// Babystepping
#if ENABLED(BABYSTEPPING) && !HAS_LCD
  #error "DEPENDENCY ERROR: BABYSTEPPING requires an LCD controller."
#endif

#endif /* _LCD_SANITYCHECK_H_ */
