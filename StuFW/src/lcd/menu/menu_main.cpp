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

//
// Main Menu
//

#include "../../../StuFW.h"

#if HAS_LCD_MENU

void menu_stop_print() {
  lcdui.encoder_direction_menus();
  START_MENU();
  MENU_BACK(MSG_MAIN);
  STATIC_ITEM(MSG_ARE_YOU_SURE);
  MENU_ITEM(function, MSG_YES, lcdui.stop_print);
  MENU_ITEM(function, MSG_NO, lcdui.return_to_status);
  END_MENU();
}

#if HAS_EEPROM
  void menu_eeprom() {
    lcdui.defer_status_screen(true);
    if (lcdui.use_click()) return lcdui.return_to_status();
    START_SCREEN();
    STATIC_ITEM(MSG_EEPROM_CHANGED_ALLERT_1);
    STATIC_ITEM(MSG_EEPROM_CHANGED_ALLERT_2);
    STATIC_ITEM(MSG_EEPROM_CHANGED_ALLERT_3);
    STATIC_ITEM(MSG_EEPROM_CHANGED_ALLERT_4);
    END_SCREEN();
  }
#endif

void menu_tune();
void menu_motion();
void menu_temperature();
void menu_configuration();
void menu_user();
void menu_temp_e0_filament_change();
void menu_change_filament();
void menu_info();
void menu_led();

#if ENABLED(COLOR_MIXING_EXTRUDER)
  void menu_mixer();
#endif

void menu_main() {
  START_MENU();
  MENU_BACK(MSG_WATCH);

  const bool busy = printer.isPrinting();

  if (busy) {
    MENU_ITEM(function, MSG_PAUSE_PRINT, lcdui.pause_print);
    MENU_ITEM(submenu, MSG_STOP_PRINT, menu_stop_print);
    MENU_ITEM(submenu, MSG_TUNE, menu_tune);
  }
  else {
    #if !HAS_ENCODER_WHEEL && HAS_SD_SUPPORT
      //
      // Autostart
      //
      #if ENABLED(MENU_ADDAUTOSTART)
        if (!busy) MENU_ITEM(function, MSG_AUTOSTART, card.beginautostart);
      #endif

      if (card.isDetected()) {
        if (!card.isFileOpen()) {
          MENU_ITEM(submenu, MSG_CARD_MENU, menu_sdcard);
          #if !PIN_EXISTS(SD_DETECT)
            MENU_ITEM(gcode, MSG_CHANGE_SDCARD, PSTR("M21"));  // SD-card changed by user
          #endif
        }
      }
      else {
        #if !PIN_EXISTS(SD_DETECT)
          MENU_ITEM(gcode, MSG_INIT_SDCARD, PSTR("M21")); // Manually initialize the SD-card via user interface
        #endif
        MENU_ITEM(function, MSG_NO_CARD, NULL);
      }
    #endif // !HAS_ENCODER_WHEEL && HAS_SD_SUPPORT

    if (printer.isPaused())
      MENU_ITEM(function, MSG_RESUME_PRINT, lcdui.resume_print);

    MENU_ITEM(submenu, MSG_MOTION, menu_motion);
    if (printer.mode == PRINTER_MODE_FFF)
      MENU_ITEM(submenu, MSG_TEMPERATURE, menu_temperature);
  }

  MENU_ITEM(submenu, MSG_CONFIGURATION, menu_configuration);

  #if ENABLED(CUSTOM_USER_MENUS)
    MENU_ITEM(submenu, MSG_USER_MENU, menu_user);
  #endif

  if (printer.mode == PRINTER_MODE_FFF) {
    #if ENABLED(COLOR_MIXING_EXTRUDER)
      MENU_ITEM(submenu, MSG_MIXER, menu_mixer);
    #endif
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      #if DRIVER_EXTRUDERS == 1 && DISABLED(FILAMENT_LOAD_UNLOAD_GCODES)
        if (thermalManager.targetHotEnoughToExtrude(tools.active_extruder))
          MENU_ITEM(gcode, MSG_FILAMENTCHANGE, PSTR("M600 B0"));
        else
          MENU_ITEM(submenu, MSG_FILAMENTCHANGE, menu_temp_e0_filament_change);
      #else
        MENU_ITEM(submenu, MSG_FILAMENTCHANGE, menu_change_filament);
      #endif
    #endif
  }

  #if ENABLED(LCD_INFO_MENU)
    MENU_ITEM(submenu, MSG_INFO_MENU, menu_info);
  #endif

  #if ENABLED(LED_CONTROL_MENU)
    MENU_ITEM(submenu, MSG_LED_CONTROL, menu_led);
  #endif

  //
  // Switch power on/off
  //
  #if HAS_POWER_SWITCH
    if (powerManager.is_on())
      MENU_ITEM(gcode, MSG_SWITCH_PS_OFF, PSTR("M81"));
    else
      MENU_ITEM(gcode, MSG_SWITCH_PS_ON, PSTR("M80"));
  #endif

  #if HAS_ENCODER_WHEEL && HAS_SD_SUPPORT
    //
    // Autostart
    //
    #if ENABLED(MENU_ADDAUTOSTART)
      if (!busy) MENU_ITEM(function, MSG_AUTOSTART, card.beginautostart);
    #endif

    if (card.isDetected()) {
      if (!card.isFileOpen()) {
        MENU_ITEM(submenu, MSG_CARD_MENU, menu_sdcard);
        #if !PIN_EXISTS(SD_DETECT)
          MENU_ITEM(gcode, MSG_CHANGE_SDCARD, PSTR("M21"));  // SD-card changed by user
        #endif
      }
    }
    else {
      #if !PIN_EXISTS(SD_DETECT)
        MENU_ITEM(gcode, MSG_INIT_SDCARD, PSTR("M21")); // Manually initialize the SD-card via user interface
      #endif
      MENU_ITEM(function, MSG_NO_CARD, NULL);
    }
  #endif // HAS_ENCODER_WHEEL && HAS_SD_SUPPORT

  END_MENU();
}

#endif // HAS_LCD_MENU
