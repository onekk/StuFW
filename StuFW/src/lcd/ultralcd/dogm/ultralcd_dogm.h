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

/**
 * ultralcd_dogm.h
 */

#include <U8glib.h>
#include "HAL_LCD_class_defines.h"

// LCD selection
#if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
  #define U8G_CLASS U8GLIB_ST7920_128X64_4X
  #if DISABLED(SDSUPPORT) && (LCD_PINS_D4 == SCK_PIN) && (LCD_PINS_ENABLE == MOSI_PIN)
    #define U8G_PARAM LCD_PINS_RS
  #else
    #define U8G_PARAM LCD_PINS_D4, LCD_PINS_ENABLE, LCD_PINS_RS
  #endif

#elif ENABLED(U8GLIB_ST7920)
  // RepRap Discount Full Graphics Smart Controller
  #if DISABLED(SDSUPPORT) && (LCD_PINS_D4 == SCK_PIN) && (LCD_PINS_ENABLE == MOSI_PIN)
    #define U8G_CLASS U8GLIB_ST7920_128X64_4X_HAL
    #define U8G_PARAM LCD_PINS_RS // 2 stripes, HW SPI (shared with SD card, on AVR does not use standard LCD adapter)
  #else
    //#define U8G_CLASS U8GLIB_ST7920_128X64_4X
    //#define U8G_PARAM LCD_PINS_D4, LCD_PINS_ENABLE, LCD_PINS_RS     // Original u8glib device. 2 stripes, SW SPI
    #define U8G_CLASS U8GLIB_ST7920_128X64_RRD
    #define U8G_PARAM LCD_PINS_D4, LCD_PINS_ENABLE, LCD_PINS_RS       // Number of stripes can be adjusted in ultralcd_st7920_u8glib_rrd.h with PAGE_HEIGHT
                                                                      // AVR version ignores these pin settings
                                                                      // HAL version uses these pin settings
  #endif
#else
  // for regular DOGM128 display with HW-SPI
  //#define U8G_CLASS U8GLIB_DOGM128
  //#define U8G_PARAM DOGLCD_CS, DOGLCD_A0                            // HW-SPI Com: CS, A0  // 8 stripes
  #define U8G_CLASS U8GLIB_DOGM128_2X
  #define U8G_PARAM DOGLCD_CS, DOGLCD_A0                              // HW-SPI Com: CS, A0 // 4 stripes
#endif

#ifndef LCD_PIXEL_WIDTH
  #define LCD_PIXEL_WIDTH 128
#endif
#ifndef LCD_PIXEL_HEIGHT
  #define LCD_PIXEL_HEIGHT 64
#endif

// For selective rendering within a Y range
#define PAGE_OVER(ya)         ((ya) <= u8g.getU8g()->current_page.y1) // Does the current page follow a region top?
#define PAGE_UNDER(yb)        ((yb) >= u8g.getU8g()->current_page.y0) // Does the current page precede a region bottom?
#define PAGE_CONTAINS(ya, yb) ((yb) >= u8g.getU8g()->current_page.y0 && (ya) <= u8g.getU8g()->current_page.y1) // Do two vertical regions overlap?

// Only Western languages support big / small fonts
#if DISABLED(DISPLAY_CHARSET_ISO10646_1)
  #undef USE_BIG_EDIT_FONT
  #undef USE_SMALL_INFOFONT
#endif

#define MENU_FONT_NAME    ISO10646_1_5x7
#define MENU_FONT_WIDTH    6
#define MENU_FONT_ASCENT  10
#define MENU_FONT_DESCENT  2
#define MENU_FONT_HEIGHT  (MENU_FONT_ASCENT + MENU_FONT_DESCENT)

#if ENABLED(USE_BIG_EDIT_FONT)
  #define EDIT_FONT_NAME    u8g_font_9x18
  #define EDIT_FONT_WIDTH    9
  #define EDIT_FONT_ASCENT  10
  #define EDIT_FONT_DESCENT  3
#else
  #define EDIT_FONT_NAME    MENU_FONT_NAME
  #define EDIT_FONT_WIDTH   MENU_FONT_WIDTH
  #define EDIT_FONT_ASCENT  MENU_FONT_ASCENT
  #define EDIT_FONT_DESCENT MENU_FONT_DESCENT
#endif
#define EDIT_FONT_HEIGHT (EDIT_FONT_ASCENT + EDIT_FONT_DESCENT)

// Get the Ascent, Descent, and total Height for the Info Screen font
#if ENABLED(USE_SMALL_INFOFONT)
  extern const u8g_fntpgm_uint8_t u8g_font_6x9[];
  #define INFO_FONT_ASCENT 7
#else
  #define INFO_FONT_ASCENT 8
#endif
#define INFO_FONT_DESCENT 2
#define INFO_FONT_HEIGHT (INFO_FONT_ASCENT + INFO_FONT_DESCENT)
#define INFO_FONT_WIDTH   6

extern U8G_CLASS u8g;
