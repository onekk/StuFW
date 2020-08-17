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

//
// Configuration Menu
//

#include "../../../StuFW.h"

#if HAS_LCD_MENU

#define HAS_DEBUG_MENU ENABLED(LCD_PROGRESS_BAR_TEST)

void menu_advanced_settings();
void menu_delta_calibrate();

#if HAS_LCD_CONTRAST
  void lcd_callback_set_contrast() { lcdui.set_contrast(lcdui.contrast); }
#endif

static void lcd_reset_settings() { eeprom.reset(); }

#if ENABLED(LCD_PROGRESS_BAR_TEST)

  static void progress_bar_test() {
    static int8_t bar_percent = 0;
    if (lcdui.use_click()) {
      lcdui.goto_previous_screen();
      LCD_SET_CHARSET(CHARSET_MENU);
      return;
    }
    bar_percent += (int8_t)lcdui.encoderPosition;
    bar_percent = constrain(bar_percent, 0, 100);
    lcdui.encoderPosition = 0;
    draw_menu_item_static(0, PSTR(MSG_PROGRESS_BAR_TEST), true, true);
    lcd_moveto((LCD_WIDTH) / 2 - 2, LCD_HEIGHT - 2);
    lcd_put_u8str(int(bar_percent)); lcd_put_wchar('%');
    lcd_moveto(0, LCD_HEIGHT - 1); lcd_draw_progress_bar(bar_percent);
  }

  void _progress_bar_test() {
    lcdui.goto_screen(progress_bar_test);
    lcdui.set_custom_characters(CHARSET_INFO);
  }

#endif // LCD_PROGRESS_BAR_TEST

#if HAS_DEBUG_MENU

  void menu_debug() {
    START_MENU();

    MENU_BACK(MSG_MAIN);

    #if ENABLED(LCD_PROGRESS_BAR_TEST)
      MENU_ITEM(submenu, MSG_PROGRESS_BAR_TEST, _progress_bar_test);
    #endif

    END_MENU();
  }

#endif

#if ENABLED(DUAL_X_CARRIAGE)

  void _recalc_DXC_settings() {
    if (tools.active_extruder) {                // For the 2nd extruder re-home so the next tool-change gets the new offsets.
      commands.enqueue_and_echo_P(PSTR("G28")); // In future, we can babystep the 2nd extruder (if active), making homing unnecessary.
      tools.active_extruder = 0;
    }
  }

  void menu_DXC() {
    START_MENU();
    MENU_BACK(MSG_MAIN);

    MENU_ITEM(gcode, MSG_DXC_MODE_AUTOPARK, PSTR("M605 S1\nG28 X\nG1 X100"));
    const bool need_g28 = !(mechanics.home_flag.YHomed && mechanics.home_flag.ZHomed);
    MENU_ITEM(gcode, MSG_DXC_MODE_DUPLICATE, need_g28
      ? PSTR("M605 S1\nT0\nG28\nM605 S2 X200\nG28 X\nG1 X100")                // If Y or Z is not homed, do a full G28 first
      : PSTR("M605 S1\nT0\nM605 S2 X200\nG28 X\nG1 X100")
    );
    MENU_ITEM(gcode, MSG_DXC_MODE_FULL_CTRL, PSTR("M605 S0\nG28 X"));
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float52, MSG_DXC_X_OFFSET , &tools.hotend_offset[X_AXIS][1], MIN(X2_HOME_POS, X2_MAX_POS) - 25.0, MAX(X2_HOME_POS, X2_MAX_POS) + 25.0, _recalc_DXC_settings);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float52, MSG_DXC_Y_OFFSET , &tools.hotend_offset[Y_AXIS][1], -10.0, 10.0, _recalc_DXC_settings);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float52, MSG_DXC_Z_OFFSET , &tools.hotend_offset[Z_AXIS][1], -10.0, 10.0, _recalc_DXC_settings);
    MENU_ITEM(gcode, MSG_DXC_SAVE_OFFSETS, PSTR("M500"));
    END_MENU();
  }

#endif

#if ENABLED(BLTOUCH)

  void menu_bltouch() {
    START_MENU();
    MENU_BACK(MSG_MAIN);
    MENU_ITEM(gcode, MSG_BLTOUCH_RESET, PSTR("M280 P" STRINGIFY(Z_PROBE_SERVO_NR) " S" STRINGIFY(BLTOUCH_RESET)));
    MENU_ITEM(gcode, MSG_BLTOUCH_SELFTEST, PSTR("M280 P" STRINGIFY(Z_PROBE_SERVO_NR) " S" STRINGIFY(BLTOUCH_SELFTEST)));
    MENU_ITEM(gcode, MSG_BLTOUCH_DEPLOY, PSTR("M280 P" STRINGIFY(Z_PROBE_SERVO_NR) " S" STRINGIFY(BLTOUCH_DEPLOY)));
    MENU_ITEM(gcode, MSG_BLTOUCH_STOW, PSTR("M280 P" STRINGIFY(Z_PROBE_SERVO_NR) " S" STRINGIFY(BLTOUCH_STOW)));
    END_MENU();
  }

#endif

#if HAS_CASE_LIGHT

  void menu_case_light() {
    START_MENU();
    MENU_BACK(MSG_MAIN);
    MENU_ITEM_EDIT_CALLBACK(uint8, MSG_CASE_LIGHT_BRIGHTNESS, &caselight.brightness, 0, 255, caselight.update, true);
    MENU_ITEM_EDIT_CALLBACK(bool, MSG_CASE_LIGHT, (bool*)&caselight.status, caselight.update);
    END_MENU();
  }

#endif

#if ENABLED(FWRETRACT)

  void menu_config_retract() {
    START_MENU();
    MENU_BACK(MSG_CONTROL);
    MENU_ITEM_EDIT_CALLBACK(bool, MSG_AUTORETRACT, &fwretract.autoretract_enabled, fwretract.refresh_autoretract);
    MENU_ITEM_EDIT(float52sign, MSG_CONTROL_RETRACT, &fwretract.data.retract_length, 0, 100);
    #if EXTRUDERS > 1
      MENU_ITEM_EDIT(float52sign, MSG_CONTROL_RETRACT_SWAP, &fwretract.data.swap_retract_length, 0, 100);
    #endif
    MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACTF, &fwretract.data.retract_feedrate_mm_s, 1, 999);
    MENU_ITEM_EDIT(float52sign, MSG_CONTROL_RETRACT_ZHOP, &fwretract.data.retract_zlift, 0, 999);
    MENU_ITEM_EDIT(float52sign, MSG_CONTROL_RETRACT_RECOVER, &fwretract.data.retract_recover_length, -100, 100);
    #if EXTRUDERS > 1
      MENU_ITEM_EDIT(float52sign, MSG_CONTROL_RETRACT_RECOVER_SWAP, &fwretract.data.swap_retract_recover_length, -100, 100);
    #endif
    MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACT_RECOVERF, &fwretract.data.retract_recover_feedrate_mm_s, 1, 999);
    #if EXTRUDERS > 1
      MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACT_RECOVER_SWAPF, &fwretract.data.swap_retract_recover_feedrate_mm_s, 1, 999);
    #endif
    END_MENU();
  }

#endif


#if DISABLED(SLIM_LCD_MENUS)

  void _menu_configuration_preheat_settings(const uint8_t material) {
    #if HOTENDS > 3
      #define MINTEMP_ALL MIN(HEATER_0_MINTEMP, HEATER_1_MINTEMP, HEATER_2_MINTEMP, HEATER_3_MINTEMP)
      #define MAXTEMP_ALL MAX(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP, HEATER_2_MAXTEMP, HEATER_3_MAXTEMP)
    #elif HOTENDS > 2
      #define MINTEMP_ALL MIN(HEATER_0_MINTEMP, HEATER_1_MINTEMP, HEATER_2_MINTEMP)
      #define MAXTEMP_ALL MAX(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP, HEATER_2_MAXTEMP)
    #elif HOTENDS > 1
      #define MINTEMP_ALL MIN(HEATER_0_MINTEMP, HEATER_1_MINTEMP)
      #define MAXTEMP_ALL MAX(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP)
    #elif HOTENDS > 0
      #define MINTEMP_ALL HEATER_0_MINTEMP
      #define MAXTEMP_ALL HEATER_0_MAXTEMP
    #endif
    START_MENU();
    MENU_BACK(MSG_CONFIGURATION);
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &lcdui.preheat_fan_speed[material], 0, 255);
    #if HAS_TEMP_0
      MENU_ITEM_EDIT(int3, MSG_NOZZLE, &lcdui.preheat_hotend_temp[material], MINTEMP_ALL, MAXTEMP_ALL - 15);
    #endif
    #if HAS_TEMP_BED
      MENU_ITEM_EDIT(int3, MSG_BED, &lcdui.preheat_bed_temp[material], BED_MINTEMP, BED_MAXTEMP - 15);
    #endif
    #if ENABLED(EEPROM_SETTINGS)
      MENU_ITEM(function, MSG_STORE_EEPROM, lcd_store_settings);
    #endif
    END_MENU();
  }

  void menu_preheat_material1_settings() { _menu_configuration_preheat_settings(0); }
  void menu_preheat_material2_settings() { _menu_configuration_preheat_settings(1); }
  void menu_preheat_material3_settings() { _menu_configuration_preheat_settings(2); }

#endif

void menu_configuration() {
  START_MENU();
  MENU_BACK(MSG_MAIN);

  //
  // Debug Menu when certain options are enabled
  //
  #if HAS_DEBUG_MENU
    MENU_ITEM(submenu, MSG_DEBUG_MENU, menu_debug);
  #endif

  MENU_ITEM(submenu, MSG_ADVANCED_SETTINGS, menu_advanced_settings);

  const bool busy = printer.isPrinting();
  if (!busy) {

    #if ENABLED(DUAL_X_CARRIAGE)
      MENU_ITEM(submenu, MSG_DXC_MENU, menu_DXC);
    #endif

    #if ENABLED(BLTOUCH)
      MENU_ITEM(submenu, MSG_BLTOUCH, menu_bltouch);
    #endif
  }

  //
  // Set Case light on/off/brightness
  //
  #if HAS_CASE_LIGHT
    if (USEABLE_HARDWARE_PWM(CASE_LIGHT_PIN))
      MENU_ITEM(submenu, MSG_CASE_LIGHT, menu_case_light);
    else
      MENU_ITEM_EDIT_CALLBACK(bool, MSG_CASE_LIGHT, (bool*)&caselight.status, caselight.update);
  #endif

  #if HAS_LCD_CONTRAST
    MENU_ITEM_EDIT_CALLBACK(uint8, MSG_CONTRAST, &lcdui.contrast, LCD_CONTRAST_MIN, LCD_CONTRAST_MAX, lcd_callback_set_contrast, true);
  #endif

  if (printer.mode == PRINTER_MODE_FFF) {
    #if ENABLED(IDLE_OOZING_PREVENT)
      MENU_ITEM_EDIT(bool, MSG_IDLEOOZING, &printer.IDLE_OOZING_enabled);
    #endif

    #if ENABLED(FWRETRACT)
      MENU_ITEM(submenu, MSG_RETRACT, menu_config_retract);
    #endif

    #if DISABLED(SLIM_LCD_MENUS)
      // Preheat configurations
      MENU_ITEM(submenu, MSG_PREHEAT_1_SETTINGS, menu_preheat_material1_settings);
      MENU_ITEM(submenu, MSG_PREHEAT_2_SETTINGS, menu_preheat_material2_settings);
      MENU_ITEM(submenu, MSG_PREHEAT_3_SETTINGS, menu_preheat_material3_settings);
    #endif
  }

  #if HAS_SD_RESTART
    MENU_ITEM_EDIT_CALLBACK(bool, MSG_RESTART, &restart.enabled, restart.changed);
  #endif

  #if DISABLED(SLIM_LCD_MENUS)

    switch(sound.mode) {
      case SOUND_MODE_ON:
        MENU_ITEM(function, MSG_SOUND_MODE_ON, sound.cycleState);
        break;
      case SOUND_MODE_SILENT:
        MENU_ITEM(function, MSG_SOUND_MODE_SILENT, sound.cycleState);
        break;
      case SOUND_MODE_MUTE:
        MENU_ITEM(function, MSG_SOUND_MODE_MUTE, sound.cycleState);
        break;
      default:
        MENU_ITEM(function, MSG_SOUND_MODE_ON, sound.cycleState);
    }

  #endif

  #if ENABLED(EEPROM_SETTINGS)
    MENU_ITEM(function, MSG_STORE_EEPROM, lcd_store_settings);
    if (!busy)
      MENU_ITEM(function, MSG_LOAD_EEPROM, lcd_load_settings);
  #endif

  if (!busy)
    MENU_ITEM(function, MSG_RESTORE_FAILSAFE, lcd_reset_settings);

  END_MENU();
}

#endif // HAS_LCD_MENU
