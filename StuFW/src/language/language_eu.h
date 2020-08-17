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
 * Basque-Euskera
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_1
#define NOT_EXTENDED_ISO10646_1_5X7

#define WELCOME_MSG                         MACHINE_NAME _UxGT(" prest.")
#define MSG_BACK                            _UxGT("Atzera")
#define MSG_SD_INSERTED                     _UxGT("Txartela sartuta")
#define MSG_SD_REMOVED                      _UxGT("Txartela kenduta")
#define MSG_LCD_ENDSTOPS                    _UxGT("Endstops") // Max length 8 characters
#define MSG_MAIN                            _UxGT("Menu nagusia")
#define MSG_AUTOSTART                       _UxGT("Auto hasiera")
#define MSG_DISABLE_STEPPERS                _UxGT("Itzali motoreak")
#define MSG_DEBUG_MENU                      _UxGT("Arazketa Menua")
#define MSG_PROGRESS_BAR_TEST               _UxGT("Prog. Barra Proba")
#define MSG_AUTO_HOME                       _UxGT("Hasierara joan")
#define MSG_AUTO_HOME_X                     _UxGT("X jatorrira")
#define MSG_AUTO_HOME_Y                     _UxGT("Y jatorrira")
#define MSG_AUTO_HOME_Z                     _UxGT("Z jatorrira")
#define MSG_TMC_Z_CALIBRATION               _UxGT("Kalibratu Z")
#define MSG_LEVEL_BED_HOMING                _UxGT("XYZ hasieraratzen")
#define MSG_LEVEL_BED_WAITING               _UxGT("Klik egin hasteko")
#define MSG_LEVEL_BED_NEXT_POINT            _UxGT("Hurrengo Puntua")
#define MSG_LEVEL_BED_DONE                  _UxGT("Berdintzea eginda")
//#define MSG_Z_FADE_HEIGHT                 _UxGT("Fade Height")
#define MSG_SET_HOME_OFFSETS                _UxGT("Etxe. offset eza.")
#define MSG_HOME_OFFSETS_APPLIED            _UxGT("Offsetak ezarrita")
#define MSG_SET_ORIGIN                      _UxGT("Hasiera ipini")
#define MSG_PREHEAT_1                       _UxGT("Berotu " PREHEAT_1_LABEL)
#define MSG_PREHEAT_1_N                     MSG_PREHEAT_1 _UxGT(" ")
#define MSG_PREHEAT_1_ALL                   MSG_PREHEAT_1 _UxGT(" Guztia")
#define MSG_PREHEAT_1_END                   MSG_PREHEAT_1 _UxGT(" Amaia")
#define MSG_PREHEAT_1_BEDONLY               MSG_PREHEAT_1 _UxGT(" Ohea")
#define MSG_PREHEAT_1_SETTINGS              MSG_PREHEAT_1 _UxGT(" Ezarp.")
#define MSG_PREHEAT_2                       _UxGT("Berotu " PREHEAT_2_LABEL)
#define MSG_PREHEAT_2_N                     MSG_PREHEAT_1 _UxGT(" ")
#define MSG_PREHEAT_2_ALL                   MSG_PREHEAT_1 _UxGT(" Guztia")
#define MSG_PREHEAT_2_END                   MSG_PREHEAT_2 _UxGT(" Amaia")
#define MSG_PREHEAT_2_BEDONLY               MSG_PREHEAT_1 _UxGT(" Ohea")
#define MSG_PREHEAT_2_SETTINGS              MSG_PREHEAT_1 _UxGT(" Ezarp.")
#define MSG_COOLDOWN                        _UxGT("Hoztu")
#define MSG_SWITCH_PS_ON                    _UxGT("Energia piztu")
#define MSG_SWITCH_PS_OFF                   _UxGT("Energia itzali")
#define MSG_EXTRUDE                         _UxGT("Estruitu")
#define MSG_RETRACT                         _UxGT("Atzera eragin")
#define MSG_MOVE_AXIS                       _UxGT("Ardatzak mugitu")
#define MSG_BED_LEVELING                    _UxGT("Ohe berdinketa")
#define MSG_LEVEL_BED                       _UxGT("Ohea berdindu")
#define MSG_LEVEL_CORNERS                   _UxGT("Ertzak berdindu")
#define MSG_NEXT_CORNER                     _UxGT("Hurrengo ertza")
//#define MSG_EDITING_STOPPED               _UxGT("Mesh Editing Stopped")
//#define MSG_USER_MENU                     _UxGT("Custom Commands")

#define MSG_UBL_DOING_G29                   _UxGT("G29 exekutatzen")
#define MSG_UBL_UNHOMED                     _UxGT("XYZ etxeratu lehenengo")
#define MSG_UBL_TOOLS                       _UxGT("UBL Tresnak")
#define MSG_UBL_LEVEL_BED                   _UxGT("Unified Bed Leveling")
#define MSG_UBL_MANUAL_MESH                 _UxGT("Sarea eskuz sortu")
//#define MSG_UBL_BC_INSERT                 _UxGT("Place shim & measure")
#define MSG_UBL_BC_INSERT2                  _UxGT("Neurtu")
//#define MSG_UBL_BC_REMOVE                 _UxGT("Remove & measure bed")
//#define MSG_UBL_MOVING_TO_NEXT            _UxGT("Moving to next")
#define MSG_UBL_ACTIVATE_MESH               _UxGT("UBL aktibatu")
#define MSG_UBL_DEACTIVATE_MESH             _UxGT("UBL desaktibatu")
#define MSG_UBL_SET_TEMP_BED                _UxGT("Ohearen tenperatura")
#define MSG_UBL_BED_TEMP_CUSTOM             MSG_UBL_SET_TEMP_BED
#define MSG_UBL_SET_TEMP_HOTEND             _UxGT("Mutur beroaren tenp.")
#define MSG_UBL_HOTEND_TEMP_CUSTOM          MSG_UBL_SET_TEMP_HOTEND
#define MSG_UBL_MESH_EDIT                   _UxGT("Sarea editatu")
//#define MSG_UBL_EDIT_CUSTOM_MESH          _UxGT("Edit Custom Mesh")
//#define MSG_UBL_FINE_TUNE_MESH            _UxGT("Fine Tuning Mesh")
#define MSG_UBL_DONE_EDITING_MESH           _UxGT("Sarea editatzea eginda")
//#define MSG_UBL_BUILD_CUSTOM_MESH         _UxGT("Build Custom Mesh")
#define MSG_UBL_BUILD_MESH_MENU             _UxGT("Sarea sortu")
#define MSG_UBL_BUILD_MESH_M1               _UxGT(PREHEAT_1_LABEL " sarea sortu")
#define MSG_UBL_BUILD_MESH_M2               _UxGT(PREHEAT_2_LABEL " sarea sortu")
#define MSG_UBL_BUILD_COLD_MESH             _UxGT("Sare hotza sortu")
#define MSG_UBL_MESH_HEIGHT_ADJUST          _UxGT("Sarearen altuera doitu")
//#define MSG_UBL_MESH_HEIGHT_AMOUNT        _UxGT("Height Amount")
#define MSG_UBL_VALIDATE_MESH_MENU          _UxGT("Sarea balioetsi")
#define MSG_UBL_VALIDATE_MESH_M1            _UxGT(PREHEAT_1_LABEL " sarea balioetsi")
#define MSG_UBL_VALIDATE_MESH_M2            _UxGT(PREHEAT_2_LABEL " sarea balioetsi")
//#define MSG_UBL_VALIDATE_CUSTOM_MESH      _UxGT("Validate Custom Mesh")
#define MSG_UBL_CONTINUE_MESH               _UxGT("Ohe sarea balioetsi")
#define MSG_UBL_MESH_LEVELING               _UxGT("Sare berdinketa")
#define MSG_UBL_3POINT_MESH_LEVELING        _UxGT("3 puntuko berdinketa")
#define MSG_UBL_GRID_MESH_LEVELING          _UxGT("Lauki-sare berdinketa")
#define MSG_UBL_MESH_LEVEL                  _UxGT("Sarea berdindu")
//#define MSG_UBL_SIDE_POINTS               _UxGT("Side Points")
#define MSG_UBL_MAP_TYPE                    _UxGT("Mapa mota")
//#define MSG_UBL_OUTPUT_MAP                _UxGT("Output Mesh Map")
//#define MSG_UBL_OUTPUT_MAP_HOST           _UxGT("Output for Host")
//#define MSG_UBL_OUTPUT_MAP_CSV            _UxGT("Output for CSV")
//#define MSG_UBL_OUTPUT_MAP_BACKUP         _UxGT("Off Printer Backup")
//#define MSG_UBL_INFO_UBL                  _UxGT("Output UBL Info")
#define MSG_EDIT_MESH                       _UxGT("Sarea editatu")
//#define MSG_UBL_FILLIN_AMOUNT             _UxGT("Fill-in Amount")
//#define MSG_UBL_MANUAL_FILLIN             _UxGT("Manual Fill-in")
//#define MSG_UBL_SMART_FILLIN              _UxGT("Smart Fill-in")
//#define MSG_UBL_FILLIN_MESH               _UxGT("Fill-in Mesh")
//#define MSG_UBL_INVALIDATE_ALL            _UxGT("Invalidate All")
//#define MSG_UBL_INVALIDATE_CLOSEST        _UxGT("Invalidate Closest")
//#define MSG_UBL_FINE_TUNE_ALL             _UxGT("Fine Tune All")
//#define MSG_UBL_FINE_TUNE_CLOSEST         _UxGT("Fine Tune Closest")
//#define MSG_UBL_STORAGE_MESH_MENU         _UxGT("Mesh Storage")
//#define MSG_UBL_STORAGE_SLOT              _UxGT("Memory Slot")
//#define MSG_UBL_LOAD_MESH                 _UxGT("Load Bed Mesh")
//#define MSG_UBL_SAVE_MESH                 _UxGT("Save Bed Mesh")
//#define MSG_MESH_LOADED                   _UxGT("Mesh %i loaded")
//#define MSG_MESH_SAVED                    _UxGT("Mesh %i saved")
//#define MSG_NO_STORAGE                    _UxGT("No storage")
//#define MSG_UBL_SAVE_ERROR                _UxGT("Err: UBL Save")
//#define MSG_UBL_RESTORE_ERROR             _UxGT("Err: UBL Restore")
//#define MSG_UBL_Z_OFFSET_STOPPED          _UxGT("Z-Offset Stopped")
//#define MSG_UBL_STEP_BY_STEP_MENU         _UxGT("Step-By-Step UBL")
#define MSG_LED_CONTROL                     _UxGT("LED ezarpenak")
#define MSG_LEDS                            _UxGT("Argiak")
#define MSG_LED_PRESETS                     _UxGT("Argi aurrehautaketak")
#define MSG_SET_LEDS_RED                    _UxGT("Gorria")
#define MSG_SET_LEDS_ORANGE                 _UxGT("Laranja")
#define MSG_SET_LEDS_YELLOW                 _UxGT("Horia")
#define MSG_SET_LEDS_GREEN                  _UxGT("Berdea")
#define MSG_SET_LEDS_BLUE                   _UxGT("Urdina")
#define MSG_SET_LEDS_INDIGO                 _UxGT("Indigo")
#define MSG_SET_LEDS_VIOLET                 _UxGT("Bioleta")
#define MSG_SET_LEDS_WHITE                  _UxGT("Zuria")
#define MSG_SET_LEDS_DEFAULT                _UxGT("Lehenetsia")
#define MSG_CUSTOM_LEDS                     _UxGT("Argi pertsonalizatuak")
#define MSG_INTENSITY_R                     _UxGT("Intentsitate gorria")
#define MSG_INTENSITY_G                     _UxGT("Intentsitate berdea")
#define MSG_INTENSITY_B                     _UxGT("Intentsitate urdina")
#define MSG_INTENSITY_W                     _UxGT("Intentsitate zuria")
#define MSG_LED_BRIGHTNESS                  _UxGT("Distira")

#define MSG_MOVING                          _UxGT("Mugitzen...")
#define MSG_FREE_XY                         _UxGT("Askatu XY")
#define MSG_MOVE_X                          _UxGT("Mugitu X")
#define MSG_MOVE_Y                          _UxGT("Mugitu Y")
#define MSG_MOVE_Z                          _UxGT("Mugitu Z")
#define MSG_MOVE_E                          _UxGT("Estrusorea")
#define MSG_MOVE_01MM                       _UxGT("Mugitu 0.1mm")
#define MSG_MOVE_1MM                        _UxGT("Mugitu 1mm")
#define MSG_MOVE_10MM                       _UxGT("Mugitu 10mm")
#define MSG_SPEED                           _UxGT("Abiadura")
#define MSG_BED_Z                           _UxGT("Z Ohea")
#define MSG_NOZZLE                          _UxGT("Pita")
#define MSG_BED                             _UxGT("Ohea")
#define MSG_FAN_SPEED                       _UxGT("Haizagailu abiadura")
#define MSG_EXTRA_FAN_SPEED                 _UxGT("Haiz.gehig. abiadura")
#define MSG_FLOW                            _UxGT("Fluxua")
#define MSG_CONTROL                         _UxGT("Kontrola")
#define MSG_MIN                             _UxGT(" ") LCD_STR_THERMOMETER _UxGT(" Min")
#define MSG_MAX                             _UxGT(" ") LCD_STR_THERMOMETER _UxGT(" Max")
#define MSG_FACTOR                          _UxGT(" ") LCD_STR_THERMOMETER _UxGT(" Fakt")
#define MSG_AUTOTEMP                        _UxGT("Auto tenperatura")
#define MSG_ON                              _UxGT("On ")
#define MSG_OFF                             _UxGT("Off")
#define MSG_PID_P                           _UxGT("PID-P")
#define MSG_PID_I                           _UxGT("PID-I")
#define MSG_PID_D                           _UxGT("PID-D")
#define MSG_PID_C                           _UxGT("PID-C")
#define MSG_SELECT                          _UxGT("Aukeratu")
#define MSG_ACC                             _UxGT("Azelerazioa")
#define MSG_JERK                            _UxGT("Astindua")
#if IS_KINEMATIC
  #define MSG_VA_JERK                       _UxGT("Va-astindua")
  #define MSG_VB_JERK                       _UxGT("Vb-astindua")
  #define MSG_VC_JERK                       _UxGT("Vc-astindua")
#else
  #define MSG_VA_JERK                       _UxGT("Vx-astindua")
  #define MSG_VB_JERK                       _UxGT("Vy-astindua")
  #define MSG_VC_JERK                       _UxGT("Vz-astindua")
#endif
#define MSG_VE_JERK                         _UxGT("Ve-astindua")
//#define MSG_VELOCITY                      _UxGT("Velocity")
#define MSG_VMAX                            _UxGT("Vmax ")
#define MSG_VMIN                            _UxGT("Vmin")
#define MSG_VTRAV_MIN                       _UxGT("VBidaia min")
#define MSG_ACCELERATION                    MSG_ACC
#define MSG_AMAX                            _UxGT("Amax ")
#define MSG_A_RETRACT                       _UxGT("A-retrakt")
#define MSG_A_TRAVEL                        _UxGT("A-bidaia")
#define MSG_STEPS_PER_MM                    _UxGT("Pausoak/mm")
#if IS_KINEMATIC
  #define MSG_ASTEPS                        _UxGT("A pausoak/mm")
  #define MSG_BSTEPS                        _UxGT("B pausoak/mm")
  #define MSG_CSTEPS                        _UxGT("C pausoak/mm")
#else
  #define MSG_ASTEPS                        _UxGT("X pausoak/mm")
  #define MSG_BSTEPS                        _UxGT("Y pausoak/mm")
  #define MSG_CSTEPS                        _UxGT("Z pausoak/mm")
#endif
#define MSG_ESTEPS                          _UxGT("E pausoak/mm")
#define MSG_E1STEPS                         _UxGT("E1 pausoak/mm")
#define MSG_E2STEPS                         _UxGT("E2 pausoak/mm")
#define MSG_E3STEPS                         _UxGT("E3 pausoak/mm")
#define MSG_E4STEPS                         _UxGT("E4 pausoak/mm")
#define MSG_E5STEPS                         _UxGT("E5 pausoak/mm")
#define MSG_E6STEPS                         _UxGT("E6 pausoak/mm")
#define MSG_TEMPERATURE                     _UxGT("Tenperatura")
#define MSG_MOTION                          _UxGT("Mugimendua")
#define MSG_FILAMENT                        _UxGT("Harizpia")
#define MSG_VOLUMETRIC_ENABLED              _UxGT("E mm3-tan")
#define MSG_FILAMENT_DIAM                   _UxGT("Hariz. Dia.")
#define MSG_FILAMENT_UNLOAD                 _UxGT("Deskargatu mm")
#define MSG_FILAMENT_LOAD                   _UxGT("Kargatu mm")
#define MSG_ADVANCE_K                       _UxGT("K Aurrerapena")
#define MSG_CONTRAST                        _UxGT("LCD kontrastea")
#define MSG_STORE_EEPROM                    _UxGT("Gorde memoria")
#define MSG_LOAD_EEPROM                     _UxGT("Kargatu memoria")
#define MSG_RESTORE_FAILSAFE                _UxGT("Larri. berriz.")
#define MSG_INIT_EEPROM                     _UxGT("EEPROM-a hasieratu")
#define MSG_REFRESH                         _UxGT("Berriz kargatu")
#define MSG_WATCH                           _UxGT("Pantaila info")
#define MSG_PREPARE                         _UxGT("Prestatu")
#define MSG_TUNE                            _UxGT("Doitu")
#define MSG_PAUSE_PRINT                     _UxGT("Pausatu inprimak.")
#define MSG_RESUME_PRINT                    _UxGT("Jarraitu inprima.")
#define MSG_STOP_PRINT                      _UxGT("Gelditu inprima.")
#define MSG_CARD_MENU                       _UxGT("SD-tik inprimatu")
#define MSG_NO_CARD                         _UxGT("Ez dago SD-rik")
#define MSG_DWELL                           _UxGT("Lo egin...")
#define MSG_USERWAIT                        _UxGT("Aginduak zain...")
#define MSG_PRINT_PAUSED                    _UxGT("Inprim. geldi.")
#define MSG_PRINT_ABORTED                   _UxGT("Inprim. deusezta.")
#define MSG_NO_MOVE                         _UxGT("Mugimendu gabe.")
#define MSG_KILLED                          _UxGT("AKABATUTA. ")
#define MSG_STOPPED                         _UxGT("GELDITUTA. ")
#define MSG_CONTROL_RETRACT                 _UxGT("Atzera egin mm")
#define MSG_CONTROL_RETRACT_SWAP            _UxGT("Swap Atzera mm")
#define MSG_CONTROL_RETRACTF                _UxGT("Atzera egin V")
#define MSG_CONTROL_RETRACT_ZHOP            _UxGT("Igo mm")
#define MSG_CONTROL_RETRACT_RECOVER         _UxGT("Atzera egin mm")
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    _UxGT("Swap Atzera mm")
#define MSG_CONTROL_RETRACT_RECOVERF        _UxGT("Atzera egin V")
//#define MSG_CONTROL_RETRACT_RECOVER_SWAPF _UxGT("S UnRet V")
#define MSG_AUTORETRACT                     _UxGT("Atzera egin")
#define MSG_FILAMENTCHANGE                  _UxGT("Aldatu harizpia")
#define MSG_FILAMENTLOAD                    _UxGT("Harizpia kargatu")
#define MSG_FILAMENTUNLOAD                  _UxGT("Harizpia deskargatu")
#define MSG_FILAMENTUNLOAD_ALL              _UxGT("Erabat deskargatu")
#define MSG_INIT_SDCARD                     _UxGT("Hasieratu SD-a")
#define MSG_CHANGE_SDCARD                   _UxGT("Aldatu txartela")
#define MSG_ZPROBE_OUT                      _UxGT("Z zunda kanpora")
#define MSG_SKEW_FACTOR                     _UxGT("Okertze faktorea")
#define MSG_BLTOUCH                         _UxGT("BLTouch")
#define MSG_BLTOUCH_SELFTEST                _UxGT("BLTouch AutoProba")
#define MSG_BLTOUCH_RESET                   _UxGT("BLTouch berrabia.")
#define MSG_BLTOUCH_DEPLOY                  _UxGT("BLTouch jaitsi/luzatu")
#define MSG_BLTOUCH_STOW                    _UxGT("BLTouch igo/jaso")
#define MSG_HOME                            _UxGT("Etxera")  // Used as MSG_HOME " " MSG_X MSG_Y MSG_Z " " MSG_FIRST
#define MSG_FIRST                           _UxGT("lehenengo")
#define MSG_ZPROBE_ZOFFSET                  _UxGT("Z Konpentsatu")
#define MSG_BABYSTEP_X                      _UxGT("Mikro-urratsa X")
#define MSG_BABYSTEP_Y                      _UxGT("Mikro-urratsa Y")
#define MSG_BABYSTEP_Z                      _UxGT("Mikro-urratsa Z")
#define MSG_ENDSTOP_ABORT                   _UxGT("Endstop deusezta.")
#define MSG_HEATING_FAILED_LCD              _UxGT("Err: Beroketa")
#define MSG_ERR_REDUNDANT_TEMP              _UxGT("Err: Tenperatura")
#define MSG_THERMAL_RUNAWAY                 _UxGT("TENP. KONTROL EZA")
#define MSG_ERR_MAXTEMP                     _UxGT("Err: Tenp Maximoa")
#define MSG_ERR_MINTEMP                     _UxGT("Err: Tenp Minimoa")
#define MSG_ERR_MAXTEMP_BED                 _UxGT("Err: Ohe Tenp Max")
#define MSG_ERR_MINTEMP_BED                 _UxGT("Err: Ohe Tenp Min")
#define MSG_ERR_Z_HOMING                    MSG_HOME _UxGT(" ") MSG_X MSG_Y _UxGT(" ") MSG_FIRST
#define MSG_HALTED                          _UxGT("INPRIMA. GELDIRIK")
#define MSG_PLEASE_RESET                    _UxGT("Berrabia. Mesedez")
#define MSG_SHORT_DAY                       _UxGT("d") // One character only
#define MSG_SHORT_HOUR                      _UxGT("h") // One character only
#define MSG_SHORT_MINUTE                    _UxGT("m") // One character only
#define MSG_HEATING                         _UxGT("Berotzen...")
#define MSG_BED_HEATING                     _UxGT("Ohea Berotzen...")
#define MSG_DELTA_CALIBRATE                 _UxGT("Delta Kalibraketa")
#define MSG_DELTA_CALIBRATE_X               _UxGT("Kalibratu X")
#define MSG_DELTA_CALIBRATE_Y               _UxGT("Kalibratu Y")
#define MSG_DELTA_CALIBRATE_Z               _UxGT("Kalibratu Z")
#define MSG_DELTA_CALIBRATE_CENTER          _UxGT("Kalibratu Zentrua")
#define MSG_DELTA_SETTINGS                  _UxGT("Delta ezarpenak")
#define MSG_DELTA_AUTO_CALIBRATE            _UxGT("Auto Kalibraketa")
#define MSG_DELTA_HEIGHT_CALIBRATE          _UxGT("Delta Alt. Ezar.")
#define MSG_DELTA_DIAG_ROD                  _UxGT("Barra diagonala")
#define MSG_DELTA_HEIGHT                    _UxGT("Altuera")
#define MSG_DELTA_RADIUS                    _UxGT("Erradioa")
#define MSG_INFO_MENU                       _UxGT("Inprimagailu Inf.")
#define MSG_INFO_PRINTER_MENU               _UxGT("Inprimagailu Inf.")
#define MSG_3POINT_LEVELING                 _UxGT("3 puntuko berdinketa")
#define MSG_LINEAR_LEVELING                 _UxGT("Berdinketa lineala")
#define MSG_BILINEAR_LEVELING               _UxGT("Berdinketa bilinearra")
#define MSG_UBL_LEVELING                    _UxGT("Unified Bed Leveling")
#define MSG_MESH_LEVELING                   _UxGT("Sare berdinketa")
#define MSG_INFO_STATS_MENU                 _UxGT("Inprima. estatis.")
#define MSG_INFO_BOARD_MENU                 _UxGT("Txartelaren Info.")
#define MSG_INFO_THERMISTOR_MENU            _UxGT("Termistoreak")
#define MSG_INFO_EXTRUDERS                  _UxGT("Estrusoreak")
#define MSG_INFO_BAUDRATE                   _UxGT("Baudioak")
#define MSG_INFO_PROTOCOL                   _UxGT("Protokoloa")
#define MSG_CASE_LIGHT                      _UxGT("Kabina Argia")
//#define MSG_CASE_LIGHT_BRIGHTNESS
#if LCD_WIDTH >= 20
  #define MSG_INFO_PRINT_COUNT              _UxGT("Inprim. Zenbaketa")
  #define MSG_INFO_COMPLETED_PRINTS         _UxGT("Burututa")
  #define MSG_INFO_PRINT_TIME               _UxGT("Inprim. denbora")
  #define MSG_INFO_PRINT_LONGEST            _UxGT("Imprimatze luzeena")
  #define MSG_INFO_PRINT_FILAMENT           _UxGT("Estruituta guztira")
#else
  #define MSG_INFO_PRINT_COUNT              _UxGT("Inprimatze")
  #define MSG_INFO_COMPLETED_PRINTS         _UxGT("Burututa")
  #define MSG_INFO_PRINT_TIME               _UxGT("Guztira")
  #define MSG_INFO_PRINT_LONGEST            _UxGT("Luzeena")
  #define MSG_INFO_PRINT_FILAMENT           _UxGT("Estrusio")
#endif
#define MSG_INFO_MIN_TEMP                   _UxGT("Tenp. Minimoa")
#define MSG_INFO_MAX_TEMP                   _UxGT("Tenp. Maximoa")
#define MSG_INFO_PSU                        _UxGT("Elikadura-iturria")
#define MSG_DRIVE_STRENGTH                  _UxGT("Driver-aren potentzia")
#define MSG_DAC_PERCENT                     _UxGT("Driver %")
#define MSG_DAC_EEPROM_WRITE                _UxGT("Idatzi DAC EEPROM")
#define MSG_FILAMENT_CHANGE_HEADER_PAUSE    _UxGT("HARIZPIA ALDATU")
#define MSG_FILAMENT_CHANGE_HEADER_LOAD     _UxGT("HARIZPIA KARGATU")
#define MSG_FILAMENT_CHANGE_HEADER_UNLOAD   _UxGT("HARIZPIA DESKARGATU")
#define MSG_FILAMENT_CHANGE_OPTION_HEADER   _UxGT("ALDAKETA AUKERAK:")
//#define MSG_FILAMENT_CHANGE_OPTION_PURGE  _UxGT("Purge more")
#define MSG_FILAMENT_CHANGE_OPTION_RESUME   _UxGT("Inprima. jarraitu")
#define MSG_FILAMENT_CHANGE_NOZZLE          _UxGT("  Pita: ")
#define MSG_ERR_HOMING_FAILED               _UxGT("Hasi. huts egin du")
#define MSG_ERR_PROBING_FAILED              _UxGT("Neurketak huts egin du")
#define MSG_M600_TOO_COLD                   _UxGT("M600: hotzegi")
//
// Filament Change screens show up to 3 lines on a 4-line display
//                        ...or up to 2 lines on a 3-line display
//
#if LCD_HEIGHT >= 4
  //#define MSG_FILAMENT_CHANGE_INIT_1        _UxGT("Wait for start")
  //#define MSG_FILAMENT_CHANGE_INIT_2        _UxGT("of the filament")
  //#define MSG_FILAMENT_CHANGE_INIT_3        _UxGT("change")
  //#define MSG_FILAMENT_CHANGE_UNLOAD_1      _UxGT("Wait for")
  //#define MSG_FILAMENT_CHANGE_UNLOAD_2      _UxGT("filament unload")
  //#define MSG_FILAMENT_CHANGE_INSERT_1      _UxGT("Insert filament")
  //#define MSG_FILAMENT_CHANGE_INSERT_2      _UxGT("and press button")
  //#define MSG_FILAMENT_CHANGE_INSERT_3      _UxGT("to continue...")
  //#define MSG_FILAMENT_CHANGE_HEAT_1        _UxGT("Press button to")
  //#define MSG_FILAMENT_CHANGE_HEAT_2        _UxGT("heat nozzle.")
  //#define MSG_FILAMENT_CHANGE_HEATING_1     _UxGT("Heating nozzle")
  //#define MSG_FILAMENT_CHANGE_HEATING_2     _UxGT("Please wait...")
  //#define MSG_FILAMENT_CHANGE_LOAD_1        _UxGT("Wait for")
  //#define MSG_FILAMENT_CHANGE_LOAD_2        _UxGT("filament load")
  //#define MSG_FILAMENT_CHANGE_PURGE_1       _UxGT("Wait for")
  //#define MSG_FILAMENT_CHANGE_PURGE_2       _UxGT("filament purge")
  //#define MSG_FILAMENT_CHANGE_RESUME_1      _UxGT("Wait for print")
  //#define MSG_FILAMENT_CHANGE_RESUME_2      _UxGT("to resume")
#else // LCD_HEIGHT < 4
  #define MSG_FILAMENT_CHANGE_INIT_1          _UxGT("Mesedez, itxaron...")
  #define MSG_FILAMENT_CHANGE_UNLOAD_1        _UxGT("Deskargatzen...")
  #define MSG_FILAMENT_CHANGE_INSERT_1        _UxGT("Sartu eta click egin")
  #define MSG_FILAMENT_CHANGE_HEATING_1       _UxGT("Berotzen...")
  #define MSG_FILAMENT_CHANGE_LOAD_1          _UxGT("Kargatzen...")
  //#define MSG_FILAMENT_CHANGE_PURGE_1       _UxGT("Purging...")
  //#define MSG_FILAMENT_CHANGE_RESUME_1      _UxGT("Resuming...")
#endif // LCD_HEIGHT < 4
