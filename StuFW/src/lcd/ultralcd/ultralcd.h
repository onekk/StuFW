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

#if HAS_SPI_LCD

  #if HAS_GRAPHICAL_LCD
    #define SETCURSOR(col, row)     lcd_moveto(col * (MENU_FONT_WIDTH), (row + 1) * (MENU_FONT_HEIGHT))
    #define SETCURSOR_RJ(len, row)  lcd_moveto(LCD_PIXEL_WIDTH - len * (MENU_FONT_WIDTH), (row + 1) * (MENU_FONT_HEIGHT))
    #define LCDPRINT(p)             u8g.print(p)
    #define LCDWRITE(c)             u8g.print(c)
  #else
    #define SETCURSOR(col, row)     lcd_moveto(col, row)
    #define SETCURSOR_RJ(len, row)  lcd_moveto(LCD_WIDTH - len, row)
    #define LCDPRINT(p)             lcd_put_u8str(p)
    #define LCDWRITE(c)             lcd_put_wchar(c)
  #endif

  #define LCD_UPDATE_INTERVAL       100

  #if HAS_LCD_MENU

    // Manual Movement
    constexpr float manual_feedrate_mm_m[XYZE] = MANUAL_FEEDRATE;
    extern float move_menu_scale;

    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      void lcd_advanced_pause_show_message(const AdvancedPauseMessageEnum message,
                                           const AdvancedPauseModeEnum mode=ADVANCED_PAUSE_MODE_SAME,
                                           const uint8_t hotend=TARGET_HOTEND);
    #endif

    #if ENABLED(AUTO_BED_LEVELING_UBL)
      void lcd_mesh_edit_setup(const float &initial);
      float lcd_mesh_edit();
    #endif

    #if ENABLED(PROBE_MANUALLY)
      void _man_probe_pt(const float &rx, const float &ry);
    #endif

    #if ENABLED(PROBE_MANUALLY)
      float lcd_probe_pt(const float &rx, const float &ry);
    #endif

  #endif // HAS_LCD_MENU

#endif // HAS_SPI_LCD

#if HAS_DIGITAL_BUTTONS

  // Wheel spin pins where BA is 00, 10, 11, 01 (1 bit always changes)
  #define BLEN_A 0
  #define BLEN_B 1

  #define EN_A _BV(BLEN_A)
  #define EN_B _BV(BLEN_B)

  #define BUTTON_EXISTS(BN)   (ENABLED(BTN_## BN) && BTN_## BN >= 0)
  #define BUTTON_PRESSED(BN)  !READ(BTN_## BN)

  #if BUTTON_EXISTS(ENC)
    #define BLEN_C 2
    #define EN_C _BV(BLEN_C)
  #endif

#else

  #define BUTTON_EXISTS(BN) false

  // Shift register bits correspond to buttons:
  #define BL_LE 7   // Left
  #define BL_UP 6   // Up
  #define BL_MI 5   // Middle
  #define BL_DW 4   // Down
  #define BL_RI 3   // Right
  #define BL_ST 2   // Red Button
  #define B_LE (_BV(BL_LE))
  #define B_UP (_BV(BL_UP))
  #define B_MI (_BV(BL_MI))
  #define B_DW (_BV(BL_DW))
  #define B_RI (_BV(BL_RI))
  #define B_ST (_BV(BL_ST))

  #ifndef BUTTON_CLICK
    #define BUTTON_CLICK() (buttons & (B_MI|B_ST))
  #endif

#endif

#if BUTTON_EXISTS(BACK)
  #define BLEN_D 3
  #define EN_D _BV(BLEN_D)
  #if ENABLED(INVERT_BACK_BUTTON)
    #define LCD_BACK_CLICKED()  !(buttons & EN_D)
  #else
    #define LCD_BACK_CLICKED()  (buttons & EN_D)
  #endif
#else
  #define LCD_BACK_CLICKED()    false
#endif

#ifndef BUTTON_CLICK
  #ifdef EN_C
    #if ENABLED(INVERT_CLICK_BUTTON)
      #define BUTTON_CLICK()  !(buttons & EN_C)
    #else
      #define BUTTON_CLICK()  (buttons & EN_C)
    #endif
  #else
    #define BUTTON_CLICK()    false
  #endif
#endif
