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

#line 31

//
// Analog Pin Assignments
//
#if ENABLED(EXT_AUX_A0) && EXT_AUX_A0 >= 0 && EXT_AUX_A0 < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, EXT_AUX_A0)
#endif
#if ENABLED(EXT_AUX_A1) && EXT_AUX_A1 >= 0 && EXT_AUX_A1 < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, EXT_AUX_A1)
#endif
#if ENABLED(EXT_AUX_A2) && EXT_AUX_A2 >= 0 && EXT_AUX_A2 < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, EXT_AUX_A2)
#endif
#if ENABLED(EXT_AUX_A3) && EXT_AUX_A3 >= 0 && EXT_AUX_A3 < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, EXT_AUX_A3)
#endif
#if ENABLED(EXT_AUX_A4) && EXT_AUX_A4 >= 0 && EXT_AUX_A4 < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, EXT_AUX_A4)
#endif
#if PIN_EXISTS(FILWIDTH) && FILWIDTH_PIN < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, FILWIDTH_PIN)
#endif
#if PIN_EXISTS(MAIN_VOLTAGE_MEASURE) && MAIN_VOLTAGE_MEASURE_PIN < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, MAIN_VOLTAGE_MEASURE_PIN)
#endif
#if PIN_EXISTS(POWER_CONSUMPTION) && POWER_CONSUMPTION_PIN < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, POWER_CONSUMPTION_PIN)
#endif
#if DISABLED(ARDUINO_ARCH_SAM)  //TC1 & TC2 are macros in the SAM tool chain
  #if ENABLED(TC1) && TC1 >= 0 && TC1 < NUM_ANALOG_INPUTS
    REPORT_NAME_ANALOG(__LINE__, TC1)
  #endif
  #if ENABLED(TC2) && TC2 >= 0 && TC2 < NUM_ANALOG_INPUTS
    REPORT_NAME_ANALOG(__LINE__, TC2)
  #endif
#endif
#if PIN_EXISTS(TEMP_0) && TEMP_0_PIN < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, TEMP_0_PIN)
#endif
#if PIN_EXISTS(TEMP_1) && TEMP_1_PIN < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, TEMP_1_PIN)
#endif
#if PIN_EXISTS(TEMP_2) && TEMP_2_PIN < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, TEMP_2_PIN)
#endif
#if PIN_EXISTS(TEMP_3) && TEMP_3_PIN < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, TEMP_3_PIN)
#endif
#if PIN_EXISTS(TEMP_BED) && TEMP_BED_PIN < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, TEMP_BED_PIN)
#endif
#if PIN_EXISTS(TEMP_CHAMBER) && TEMP_CHAMBER_PIN < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, TEMP_CHAMBER_PIN)
#endif
#if PIN_EXISTS(TEMP_COOLER) && TEMP_COOLER_PIN < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, TEMP_CHAMBER_PIN)
#endif
#if PIN_EXISTS(TEMP_X) && TEMP_X_PIN < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, TEMP_X_PIN)
#endif
#if PIN_EXISTS(ADC_KEYPAD) && ADC_KEYPAD_PIN < NUM_ANALOG_INPUTS
  REPORT_NAME_ANALOG(__LINE__, ADC_KEYPAD_PIN)
#endif

//
// Digital Pin Assignments
//
#if ENABLED(__FD) && __FD >= 0
  REPORT_NAME_DIGITAL(__LINE__, __FD)
#endif
#if ENABLED(__FS) && __FS >= 0
  REPORT_NAME_DIGITAL(__LINE__, __FS)
#endif
#if ENABLED(__GD) && __GD >= 0
  REPORT_NAME_DIGITAL(__LINE__, __GD)
#endif
#if ENABLED(__GS) && __GS >= 0
  REPORT_NAME_DIGITAL(__LINE__, __GS)
#endif
#if PIN_EXISTS(AVR_MISO)
  REPORT_NAME_DIGITAL(__LINE__, AVR_MISO_PIN)
#endif
#if PIN_EXISTS(AVR_MOSI)
  REPORT_NAME_DIGITAL(__LINE__, AVR_MOSI_PIN)
#endif
#if PIN_EXISTS(AVR_SCK)
  REPORT_NAME_DIGITAL(__LINE__, AVR_SCK_PIN)
#endif
#if PIN_EXISTS(ALARM)
  REPORT_NAME_DIGITAL(__LINE__, ALARM_PIN)
#endif
#if PIN_EXISTS(AVR_SS)
  REPORT_NAME_DIGITAL(__LINE__, AVR_SS_PIN)
#endif
#if PIN_EXISTS(BEEPER)
  REPORT_NAME_DIGITAL(__LINE__, BEEPER_PIN)
#endif
#if ENABLED(BTN_BACK) && BTN_BACK >= 0
  REPORT_NAME_DIGITAL(__LINE__, BTN_BACK)
#endif
#if ENABLED(BTN_CENTER) && BTN_CENTER >= 0
  REPORT_NAME_DIGITAL(__LINE__, BTN_CENTER)
#endif
#if ENABLED(BTN_DOWN) && BTN_DOWN >= 0
  REPORT_NAME_DIGITAL(__LINE__, BTN_DOWN)
#endif
#if ENABLED(BTN_DWN) && BTN_DWN >= 0
  REPORT_NAME_DIGITAL(__LINE__, BTN_DWN)
#endif
#if ENABLED(BTN_EN1) && BTN_EN1 >= 0
  REPORT_NAME_DIGITAL(__LINE__, BTN_EN1)
#endif
#if ENABLED(BTN_EN2) && BTN_EN2 >= 0
  REPORT_NAME_DIGITAL(__LINE__, BTN_EN2)
#endif
#if ENABLED(BTN_ENC) && BTN_ENC >= 0
  REPORT_NAME_DIGITAL(__LINE__, BTN_ENC)
#endif
#if ENABLED(BTN_HOME) && BTN_HOME >= 0
  REPORT_NAME_DIGITAL(__LINE__, BTN_HOME)
#endif
#if ENABLED(BTN_LEFT) && BTN_LEFT >= 0
  REPORT_NAME_DIGITAL(__LINE__, BTN_LEFT)
#endif
#if ENABLED(BTN_LFT) && BTN_LFT >= 0
  REPORT_NAME_DIGITAL(__LINE__, BTN_LFT)
#endif
#if ENABLED(BTN_RIGHT) && BTN_RIGHT >= 0
  REPORT_NAME_DIGITAL(__LINE__, BTN_RIGHT)
#endif
#if ENABLED(BTN_RT) && BTN_RT >= 0
  REPORT_NAME_DIGITAL(__LINE__, BTN_RT)
#endif
#if ENABLED(BTN_UP) && BTN_UP >= 0
  REPORT_NAME_DIGITAL(__LINE__, BTN_UP)
#endif
#if PIN_EXISTS(CASE_LIGHT)
  REPORT_NAME_DIGITAL(__LINE__, CASE_LIGHT_PIN)
#endif
#if PIN_EXISTS(CUTOFF_RESET)
  REPORT_NAME_DIGITAL(__LINE__, CUTOFF_RESET_PIN)
#endif
#if PIN_EXISTS(CUTOFF_TEST)
  REPORT_NAME_DIGITAL(__LINE__, CUTOFF_TEST_PIN)
#endif
#if ENABLED(D57) && D57 >= 0
  REPORT_NAME_DIGITAL(__LINE__, D57)
#endif
#if ENABLED(D58) && D58 >= 0
  REPORT_NAME_DIGITAL(__LINE__, D58)
#endif
#if PIN_EXISTS(DAC_DISABLE)
  REPORT_NAME_DIGITAL(__LINE__, DAC_DISABLE_PIN)
#endif
#if ENABLED(DAC0_SYNC) && DAC0_SYNC >= 0
  REPORT_NAME_DIGITAL(__LINE__, DAC0_SYNC)
#endif
#if ENABLED(DAC1_SYNC) && DAC1_SYNC >= 0
  REPORT_NAME_DIGITAL(__LINE__, DAC1_SYNC)
#endif
#if PIN_EXISTS(DEBUG)
  REPORT_NAME_DIGITAL(__LINE__, DEBUG_PIN)
#endif
#if ENABLED(DIGIPOTS_I2C_SCL) && DIGIPOTS_I2C_SCL >= 0
  REPORT_NAME_DIGITAL(__LINE__, DIGIPOTS_I2C_SCL)
#endif
#if ENABLED(DIGIPOTS_I2C_SDA_E0) && DIGIPOTS_I2C_SDA_E0 >= 0
  REPORT_NAME_DIGITAL(__LINE__, DIGIPOTS_I2C_SDA_E0)
#endif
#if ENABLED(DIGIPOTS_I2C_SDA_E1) && DIGIPOTS_I2C_SDA_E1 >= 0
  REPORT_NAME_DIGITAL(__LINE__, DIGIPOTS_I2C_SDA_E1)
#endif
#if ENABLED(DIGIPOTS_I2C_SDA_X) && DIGIPOTS_I2C_SDA_X >= 0
  REPORT_NAME_DIGITAL(__LINE__, DIGIPOTS_I2C_SDA_X)
#endif
#if ENABLED(DIGIPOTS_I2C_SDA_Y) && DIGIPOTS_I2C_SDA_Y >= 0
  REPORT_NAME_DIGITAL(__LINE__, DIGIPOTS_I2C_SDA_Y)
#endif
#if ENABLED(DIGIPOTS_I2C_SDA_Z) && DIGIPOTS_I2C_SDA_Z >= 0
  REPORT_NAME_DIGITAL(__LINE__, DIGIPOTS_I2C_SDA_Z)
#endif
#if PIN_EXISTS(DIGIPOTSS)
  REPORT_NAME_DIGITAL(__LINE__, DIGIPOTSS_PIN)
#endif
#if ENABLED(DOGLCD_A0) && DOGLCD_A0 >= 0
  REPORT_NAME_DIGITAL(__LINE__, DOGLCD_A0)
#endif
#if ENABLED(DOGLCD_CS) && DOGLCD_CS >= 0
  REPORT_NAME_DIGITAL(__LINE__, DOGLCD_CS)
#endif
#if ENABLED(DOGLCD_MOSI) && DOGLCD_MOSI >= 0
  REPORT_NAME_DIGITAL(__LINE__, DOGLCD_MOSI)
#endif
#if ENABLED(DOGLCD_SCK) && DOGLCD_SCK >= 0
  REPORT_NAME_DIGITAL(__LINE__, DOGLCD_SCK)
#endif
#if PIN_EXISTS(E_MUX0)
  REPORT_NAME_DIGITAL(__LINE__, E_MUX0_PIN)
#endif
#if PIN_EXISTS(E_MUX1)
  REPORT_NAME_DIGITAL(__LINE__, E_MUX1_PIN)
#endif
#if PIN_EXISTS(E_MUX2)
  REPORT_NAME_DIGITAL(__LINE__, E_MUX2_PIN)
#endif
#if PIN_EXISTS(E_STOP)
  REPORT_NAME_DIGITAL(__LINE__, E_STOP_PIN)
#endif
#if PIN_EXISTS(E0_ATT)
  REPORT_NAME_DIGITAL(__LINE__, E0_ATT_PIN)
#endif
#if PIN_EXISTS(E0_CS)
  REPORT_NAME_DIGITAL(__LINE__, E0_CS_PIN)
#endif
#if PIN_EXISTS(E0_DIR)
  REPORT_NAME_DIGITAL(__LINE__, E0_DIR_PIN)
#endif
#if PIN_EXISTS(E0_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, E0_ENABLE_PIN)
#endif
#if PIN_EXISTS(E0_MS1)
  REPORT_NAME_DIGITAL(__LINE__, E0_MS1_PIN)
#endif
#if PIN_EXISTS(E0_MS2)
  REPORT_NAME_DIGITAL(__LINE__, E0_MS2_PIN)
#endif
#if PIN_EXISTS(E0_STEP)
  REPORT_NAME_DIGITAL(__LINE__, E0_STEP_PIN)
#endif
#if PIN_EXISTS(E1_CS)
  REPORT_NAME_DIGITAL(__LINE__, E1_CS_PIN)
#endif
#if PIN_EXISTS(E1_DIR)
  REPORT_NAME_DIGITAL(__LINE__, E1_DIR_PIN)
#endif
#if PIN_EXISTS(E1_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, E1_ENABLE_PIN)
#endif
#if PIN_EXISTS(E1_MS1)
  REPORT_NAME_DIGITAL(__LINE__, E1_MS1_PIN)
#endif
#if PIN_EXISTS(E1_MS2)
  REPORT_NAME_DIGITAL(__LINE__, E1_MS2_PIN)
#endif
#if PIN_EXISTS(E1_STEP)
  REPORT_NAME_DIGITAL(__LINE__, E1_STEP_PIN)
#endif
#if PIN_EXISTS(E2_CS)
  REPORT_NAME_DIGITAL(__LINE__, E2_CS_PIN)
#endif
#if PIN_EXISTS(E2_DIR)
  REPORT_NAME_DIGITAL(__LINE__, E2_DIR_PIN)
#endif
#if PIN_EXISTS(E2_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, E2_ENABLE_PIN)
#endif
#if PIN_EXISTS(E2_MS1)
  REPORT_NAME_DIGITAL(__LINE__, E2_MS1_PIN)
#endif
#if PIN_EXISTS(E2_MS2)
  REPORT_NAME_DIGITAL(__LINE__, E2_MS2_PIN)
#endif
#if PIN_EXISTS(E2_STEP)
  REPORT_NAME_DIGITAL(__LINE__, E2_STEP_PIN)
#endif
#if PIN_EXISTS(E3_CS)
  REPORT_NAME_DIGITAL(__LINE__, E3_CS_PIN)
#endif
#if PIN_EXISTS(E3_DIR)
  REPORT_NAME_DIGITAL(__LINE__, E3_DIR_PIN)
#endif
#if PIN_EXISTS(E3_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, E3_ENABLE_PIN)
#endif
#if PIN_EXISTS(E3_MS1)
  REPORT_NAME_DIGITAL(__LINE__, E3_MS1_PIN)
#endif
#if PIN_EXISTS(E3_MS2)
  REPORT_NAME_DIGITAL(__LINE__, E3_MS2_PIN)
#endif
#if PIN_EXISTS(E3_MS3)
  REPORT_NAME_DIGITAL(__LINE__, E3_MS3_PIN)
#endif
#if PIN_EXISTS(E3_STEP)
  REPORT_NAME_DIGITAL(__LINE__, E3_STEP_PIN)
#endif
#if PIN_EXISTS(E4_CS)
  REPORT_NAME_DIGITAL(__LINE__, E4_CS_PIN)
#endif
#if PIN_EXISTS(E4_DIR)
  REPORT_NAME_DIGITAL(__LINE__, E4_DIR_PIN)
#endif
#if PIN_EXISTS(E4_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, E4_ENABLE_PIN)
#endif
#if PIN_EXISTS(E4_MS1)
  REPORT_NAME_DIGITAL(__LINE__, E4_MS1_PIN)
#endif
#if PIN_EXISTS(E4_MS2)
  REPORT_NAME_DIGITAL(__LINE__, E4_MS2_PIN)
#endif
#if PIN_EXISTS(E4_MS3)
  REPORT_NAME_DIGITAL(__LINE__, E4_MS3_PIN)
#endif
#if PIN_EXISTS(E4_STEP)
  REPORT_NAME_DIGITAL(__LINE__, E4_STEP_PIN)
#endif
#if PIN_EXISTS(E5_DIR)
  REPORT_NAME_DIGITAL(__LINE__, E5_DIR_PIN)
#endif
#if PIN_EXISTS(E5_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, E5_ENABLE_PIN)
#endif
#if PIN_EXISTS(E5_STEP)
  REPORT_NAME_DIGITAL(__LINE__, E5_STEP_PIN)
#endif
#if ENABLED(ENET_CRS) && ENET_CRS >= 0
  REPORT_NAME_DIGITAL(__LINE__, ENET_CRS)
#endif
#if ENABLED(ENET_MDIO) && ENET_MDIO >= 0
  REPORT_NAME_DIGITAL(__LINE__, ENET_MDIO)
#endif
#if ENABLED(ENET_MOC) && ENET_MOC >= 0
  REPORT_NAME_DIGITAL(__LINE__, ENET_MOC)
#endif
#if ENABLED(ENET_RX_ER) && ENET_RX_ER >= 0
  REPORT_NAME_DIGITAL(__LINE__, ENET_RX_ER)
#endif
#if ENABLED(ENET_RXD0) && ENET_RXD0 >= 0
  REPORT_NAME_DIGITAL(__LINE__, ENET_RXD0)
#endif
#if ENABLED(ENET_RXD1) && ENET_RXD1 >= 0
  REPORT_NAME_DIGITAL(__LINE__, ENET_RXD1)
#endif
#if ENABLED(ENET_TX_EN) && ENET_TX_EN >= 0
  REPORT_NAME_DIGITAL(__LINE__, ENET_TX_EN)
#endif
#if ENABLED(ENET_TXD0) && ENET_TXD0 >= 0
  REPORT_NAME_DIGITAL(__LINE__, ENET_TXD0)
#endif
#if ENABLED(ENET_TXD1) && ENET_TXD1 >= 0
  REPORT_NAME_DIGITAL(__LINE__, ENET_TXD1)
#endif
#if PIN_EXISTS(EXP_VOLTAGE_LEVEL)
  REPORT_NAME_DIGITAL(__LINE__, EXP_VOLTAGE_LEVEL_PIN)
#endif
#if ENABLED(EXT_AUX_A0_IO) && EXT_AUX_A0_IO >= 0
  REPORT_NAME_DIGITAL(__LINE__, EXT_AUX_A0_IO)
#endif
#if ENABLED(EXT_AUX_A1_IO) && EXT_AUX_A1_IO >= 0
  REPORT_NAME_DIGITAL(__LINE__, EXT_AUX_A1_IO)
#endif
#if ENABLED(EXT_AUX_A2_IO) && EXT_AUX_A2_IO >= 0
  REPORT_NAME_DIGITAL(__LINE__, EXT_AUX_A2_IO)
#endif
#if ENABLED(EXT_AUX_A3_IO) && EXT_AUX_A3_IO >= 0
  REPORT_NAME_DIGITAL(__LINE__, EXT_AUX_A3_IO)
#endif
#if ENABLED(EXT_AUX_A4_IO) && EXT_AUX_A4_IO >= 0
  REPORT_NAME_DIGITAL(__LINE__, EXT_AUX_A4_IO)
#endif
#if ENABLED(EXT_AUX_PWM_D24) && EXT_AUX_PWM_D24 >= 0
  REPORT_NAME_DIGITAL(__LINE__, EXT_AUX_PWM_D24)
#endif
#if ENABLED(EXT_AUX_RX1_D2) && EXT_AUX_RX1_D2 >= 0
  REPORT_NAME_DIGITAL(__LINE__, EXT_AUX_RX1_D2)
#endif
#if ENABLED(EXT_AUX_SCL_D0) && EXT_AUX_SCL_D0 >= 0
  REPORT_NAME_DIGITAL(__LINE__, EXT_AUX_SCL_D0)
#endif
#if ENABLED(EXT_AUX_SDA_D1) && EXT_AUX_SDA_D1 >= 0
  REPORT_NAME_DIGITAL(__LINE__, EXT_AUX_SDA_D1)
#endif
#if ENABLED(EXT_AUX_TX1_D3) && EXT_AUX_TX1_D3 >= 0
  REPORT_NAME_DIGITAL(__LINE__, EXT_AUX_TX1_D3)
#endif
#if PIN_EXISTS(FAN0)
  REPORT_NAME_DIGITAL(__LINE__, FAN0_PIN)
#endif
#if PIN_EXISTS(FAN1)
  REPORT_NAME_DIGITAL(__LINE__, FAN1_PIN)
#endif
#if PIN_EXISTS(FAN2)
  REPORT_NAME_DIGITAL(__LINE__, FAN2_PIN)
#endif
#if PIN_EXISTS(FAN3)
  REPORT_NAME_DIGITAL(__LINE__, FAN3_PIN)
#endif
#if PIN_EXISTS(FAN4)
  REPORT_NAME_DIGITAL(__LINE__, FAN4_PIN)
#endif
#if PIN_EXISTS(FAN5)
  REPORT_NAME_DIGITAL(__LINE__, FAN5_PIN)
#endif
#if PIN_EXISTS(FIL_RUNOUT0)
  REPORT_NAME_DIGITAL(__LINE__, FIL_RUNOUT_0_PIN)
#endif
#if PIN_EXISTS(FIL_RUNOUT1)
  REPORT_NAME_DIGITAL(__LINE__, FIL_RUNOUT_1_PIN)
#endif
#if PIN_EXISTS(FIL_RUNOUT2)
  REPORT_NAME_DIGITAL(__LINE__, FIL_RUNOUT_2_PIN)
#endif
#if PIN_EXISTS(FIL_RUNOUT3)
  REPORT_NAME_DIGITAL(__LINE__, FIL_RUNOUT_3_PIN)
#endif
#if PIN_EXISTS(FIL_RUNOUT4)
  REPORT_NAME_DIGITAL(__LINE__, FIL_RUNOUT_4_PIN)
#endif
#if PIN_EXISTS(FIL_RUNOUT5)
  REPORT_NAME_DIGITAL(__LINE__, FIL_RUNOUT_5_PIN)
#endif
#if PIN_EXISTS(HEATER_0)
  REPORT_NAME_DIGITAL(__LINE__, HEATER_0_PIN)
#endif
#if PIN_EXISTS(HEATER_1)
  REPORT_NAME_DIGITAL(__LINE__, HEATER_1_PIN)
#endif
#if PIN_EXISTS(HEATER_2)
  REPORT_NAME_DIGITAL(__LINE__, HEATER_2_PIN)
#endif
#if PIN_EXISTS(HEATER_3)
  REPORT_NAME_DIGITAL(__LINE__, HEATER_3_PIN)
#endif
#if PIN_EXISTS(HEATER_4)
  REPORT_NAME_DIGITAL(__LINE__, HEATER_4_PIN)
#endif
#if PIN_EXISTS(HEATER_5)
  REPORT_NAME_DIGITAL(__LINE__, HEATER_5_PIN)
#endif
#if PIN_EXISTS(HEATER_6)
  REPORT_NAME_DIGITAL(__LINE__, HEATER_6_PIN)
#endif
#if PIN_EXISTS(HEATER_7)
  REPORT_NAME_DIGITAL(__LINE__, HEATER_7_PIN)
#endif
#if PIN_EXISTS(HEATER_BED)
  REPORT_NAME_DIGITAL(__LINE__, HEATER_BED_PIN)
#endif
#if PIN_EXISTS(HEATER_CHAMBER)
  REPORT_NAME_DIGITAL(__LINE__, HEATER_CHAMBER_PIN)
#endif
#if PIN_EXISTS(HEATER_COOLER)
  REPORT_NAME_DIGITAL(__LINE__, HEATER_COOLER_PIN)
#endif
#if PIN_EXISTS(HOME)
  REPORT_NAME_DIGITAL(__LINE__, HOME_PIN)
#endif
#if PIN_EXISTS(I2C_SCL)
  REPORT_NAME_DIGITAL(__LINE__, I2C_SCL_PIN)
#endif
#if PIN_EXISTS(I2C_SDA)
  REPORT_NAME_DIGITAL(__LINE__, I2C_SDA_PIN)
#endif
#if PIN_EXISTS(KILL)
  REPORT_NAME_DIGITAL(__LINE__, KILL_PIN)
#endif
#if PIN_EXISTS(LCD_BACKLIGHT)
  REPORT_NAME_DIGITAL(__LINE__, LCD_BACKLIGHT_PIN)
#endif
#if ENABLED(LCD_PINS_D4) && LCD_PINS_D4 >= 0
  REPORT_NAME_DIGITAL(__LINE__, LCD_PINS_D4)
#endif
#if ENABLED(LCD_PINS_D5) && LCD_PINS_D5 >= 0
  REPORT_NAME_DIGITAL(__LINE__, LCD_PINS_D5)
#endif
#if ENABLED(LCD_PINS_D6) && LCD_PINS_D6 >= 0
  REPORT_NAME_DIGITAL(__LINE__, LCD_PINS_D6)
#endif
#if ENABLED(LCD_PINS_D7) && LCD_PINS_D7 >= 0
  REPORT_NAME_DIGITAL(__LINE__, LCD_PINS_D7)
#endif
#if ENABLED(LCD_PINS_ENABLE) && LCD_PINS_ENABLE >= 0
  REPORT_NAME_DIGITAL(__LINE__, LCD_PINS_ENABLE)
#endif
#if ENABLED(LCD_PINS_RS) && LCD_PINS_RS >= 0
  REPORT_NAME_DIGITAL(__LINE__, LCD_PINS_RS)
#endif
#if ENABLED(LCD_SDSS) && LCD_SDSS >= 0
  REPORT_NAME_DIGITAL(__LINE__, LCD_SDSS)
#endif
#if PIN_EXISTS(LED_GREEN)
  REPORT_NAME_DIGITAL(__LINE__, LED_GREEN_PIN)
#endif
#if PIN_EXISTS(LED)
  REPORT_NAME_DIGITAL(__LINE__, LED_PIN)
#endif
#if PIN_EXISTS(LED_RED)
  REPORT_NAME_DIGITAL(__LINE__, LED_RED_PIN)
#endif
#if PIN_EXISTS(MAX6675_SS)
  REPORT_NAME_DIGITAL(__LINE__, MAX6675_SS_PIN)
#endif
#if PIN_EXISTS(MISO)
  REPORT_NAME_DIGITAL(__LINE__, MISO_PIN)
#endif
#if PIN_EXISTS(MOSI)
  REPORT_NAME_DIGITAL(__LINE__, MOSI_PIN)
#endif
#if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
  REPORT_NAME_DIGITAL(__LINE__, MOTOR_CURRENT_PWM_E_PIN)
#endif
#if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
  REPORT_NAME_DIGITAL(__LINE__, MOTOR_CURRENT_PWM_XY_PIN)
#endif
#if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
  REPORT_NAME_DIGITAL(__LINE__, MOTOR_CURRENT_PWM_Z_PIN)
#endif
#if PIN_EXISTS(MOTOR_FAULT)
  REPORT_NAME_DIGITAL(__LINE__, MOTOR_FAULT_PIN)
#endif
#if PIN_EXISTS(NUM_DIGITAL)
  REPORT_NAME_DIGITAL(__LINE__, NUM_DIGITAL_PINS)
#endif
#if PIN_EXISTS(PHOTOGRAPH)
  REPORT_NAME_DIGITAL(__LINE__, PHOTOGRAPH_PIN)
#endif
#if PIN_EXISTS(PS_ON)
  REPORT_NAME_DIGITAL(__LINE__, PS_ON_PIN)
#endif
#if PIN_EXISTS(PWM_1)
  REPORT_NAME_DIGITAL(__LINE__, PWM_1_PIN)
#endif
#if PIN_EXISTS(PWM_2)
  REPORT_NAME_DIGITAL(__LINE__, PWM_2_PIN)
#endif
#if ENABLED(REF_CLK) && REF_CLK >= 0
  REPORT_NAME_DIGITAL(__LINE__, REF_CLK)
#endif
#if PIN_EXISTS(RGB_LED_R)
  REPORT_NAME_DIGITAL(__LINE__, RGB_LED_R_PIN)
#endif
#if PIN_EXISTS(RGB_LED_G)
  REPORT_NAME_DIGITAL(__LINE__, RGB_LED_G_PIN)
#endif
#if PIN_EXISTS(RGB_LED_B)
  REPORT_NAME_DIGITAL(__LINE__, RGB_LED_B_PIN)
#endif
#if PIN_EXISTS(RGB_LED_W)
  REPORT_NAME_DIGITAL(__LINE__, RGB_LED_W_PIN)
#endif
#if PIN_EXISTS(RX_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, RX_ENABLE_PIN)
#endif
#if PIN_EXISTS(SAFETY_TRIGGERED)
  REPORT_NAME_DIGITAL(__LINE__, SAFETY_TRIGGERED_PIN)
#endif
#if PIN_EXISTS(SCK)
  REPORT_NAME_DIGITAL(__LINE__, SCK_PIN)
#endif
//#if ENABLED(SCL) && SCL >= 0
//  REPORT_NAME_DIGITAL(__LINE__, SCL_PIN)
//#endif
#if PIN_EXISTS(SD_DETECT)
  REPORT_NAME_DIGITAL(__LINE__, SD_DETECT_PIN)
#endif
//#if ENABLED(SDA) && SDA >= 0
//  REPORT_NAME_DIGITAL(__LINE__, SDA)
//#endif
#if ENABLED(SDPOWER) && SDPOWER >= 0
  REPORT_NAME_DIGITAL(__LINE__, SDPOWER)
#endif
#if ENABLED(SDSS) && SDSS >= 0
  REPORT_NAME_DIGITAL(__LINE__, SDSS)
#endif
#if PIN_EXISTS(SERVO0)
  REPORT_NAME_DIGITAL(__LINE__, SERVO0_PIN)
#endif
#if PIN_EXISTS(SERVO1)
  REPORT_NAME_DIGITAL(__LINE__, SERVO1_PIN)
#endif
#if PIN_EXISTS(SERVO2)
  REPORT_NAME_DIGITAL(__LINE__, SERVO2_PIN)
#endif
#if PIN_EXISTS(SERVO3)
  REPORT_NAME_DIGITAL(__LINE__, SERVO3_PIN)
#endif
#if ENABLED(SHIFT_CLK) && SHIFT_CLK >= 0
  REPORT_NAME_DIGITAL(__LINE__, SHIFT_CLK)
#endif
#if ENABLED(SHIFT_EN) && SHIFT_EN >= 0
  REPORT_NAME_DIGITAL(__LINE__, SHIFT_EN)
#endif
#if ENABLED(SHIFT_LD) && SHIFT_LD >= 0
  REPORT_NAME_DIGITAL(__LINE__, SHIFT_LD)
#endif
#if ENABLED(SHIFT_OUT) && SHIFT_OUT >= 0
  REPORT_NAME_DIGITAL(__LINE__, SHIFT_OUT)
#endif
#if PIN_EXISTS(SLED)
  REPORT_NAME_DIGITAL(__LINE__, SLED_PIN)
#endif
#if PIN_EXISTS(SLEEP_WAKE)
  REPORT_NAME_DIGITAL(__LINE__, SLEEP_WAKE_PIN)
#endif
#if PIN_EXISTS(SOL0)
  REPORT_NAME_DIGITAL(__LINE__, SOL0_PIN)
#endif
#if PIN_EXISTS(SOL1)
  REPORT_NAME_DIGITAL(__LINE__, SOL1_PIN)
#endif
#if PIN_EXISTS(SOL2)
  REPORT_NAME_DIGITAL(__LINE__, SOL2_PIN)
#endif
#if PIN_EXISTS(SOL3)
  REPORT_NAME_DIGITAL(__LINE__, SOL3_PIN)
#endif
#if PIN_EXISTS(SOL4)
  REPORT_NAME_DIGITAL(__LINE__, SOL4_PIN)
#endif
#if ENABLED(SPARE_IO) && SPARE_IO >= 0
  REPORT_NAME_DIGITAL(__LINE__, SPARE_IO)
#endif
#if ENABLED(SPI_CHAN_DAC) && SPI_CHAN_DAC >= 0
  REPORT_NAME_DIGITAL(__LINE__, SPI_CHAN_DAC)
#endif
#if ENABLED(SPI_CHAN_EEPROM1) && SPI_CHAN_EEPROM1 >= 0
  REPORT_NAME_DIGITAL(__LINE__, SPI_CHAN_EEPROM1)
#endif
#if ENABLED(SPI_EEPROM) && SPI_EEPROM >= 0
  REPORT_NAME_DIGITAL(__LINE__, SPI_EEPROM)
#endif
#if ENABLED(SPI_EEPROM1_CS) && SPI_EEPROM1_CS >= 0
  REPORT_NAME_DIGITAL(__LINE__, SPI_EEPROM1_CS)
#endif
#if ENABLED(SPI_EEPROM2_CS) && SPI_EEPROM2_CS >= 0
  REPORT_NAME_DIGITAL(__LINE__, SPI_EEPROM2_CS)
#endif
#if ENABLED(SPI_FLASH_CS) && SPI_FLASH_CS >= 0
  REPORT_NAME_DIGITAL(__LINE__, SPI_FLASH_CS)
#endif
#if PIN_EXISTS(SPINDLE_DIR)
  REPORT_NAME_DIGITAL(__LINE__, SPINDLE_DIR_PIN)
#endif
#if PIN_EXISTS(SPINDLE_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, SPINDLE_ENABLE_PIN)
#endif
#if PIN_EXISTS(SR_CLK)
  REPORT_NAME_DIGITAL(__LINE__, SR_CLK_PIN)
#endif
#if PIN_EXISTS(SR_DATA)
  REPORT_NAME_DIGITAL(__LINE__, SR_DATA_PIN)
#endif
#if PIN_EXISTS(SR_STROBE)
  REPORT_NAME_DIGITAL(__LINE__, SR_STROBE_PIN)
#endif
#if PIN_EXISTS(STAT_LED_BLUE)
  REPORT_NAME_DIGITAL(__LINE__, STAT_LED_BLUE_PIN)
#endif
#if PIN_EXISTS(STAT_LED_RED)
  REPORT_NAME_DIGITAL(__LINE__, STAT_LED_RED_PIN)
#endif
#if PIN_EXISTS(STEPPER_RESET)
  REPORT_NAME_DIGITAL(__LINE__, STEPPER_RESET_PIN)
#endif
#if PIN_EXISTS(SUICIDE)
  REPORT_NAME_DIGITAL(__LINE__, SUICIDE_PIN)
#endif
#if PIN_EXISTS(X_CS)
  REPORT_NAME_DIGITAL(__LINE__, X_CS_PIN)
#endif
#if PIN_EXISTS(X_DIR)
  REPORT_NAME_DIGITAL(__LINE__, X_DIR_PIN)
#endif
#if PIN_EXISTS(X_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, X_ENABLE_PIN)
#endif
#if PIN_EXISTS(X_MAX)
  REPORT_NAME_DIGITAL(__LINE__, X_MAX_PIN)
#endif
#if PIN_EXISTS(X_MIN)
  REPORT_NAME_DIGITAL(__LINE__, X_MIN_PIN)
#endif
#if PIN_EXISTS(X_MS1)
  REPORT_NAME_DIGITAL(__LINE__, X_MS1_PIN)
#endif
#if PIN_EXISTS(X_MS2)
  REPORT_NAME_DIGITAL(__LINE__, X_MS2_PIN)
#endif
#if PIN_EXISTS(X_STEP)
  REPORT_NAME_DIGITAL(__LINE__, X_STEP_PIN)
#endif
#if PIN_EXISTS(X_STOP)
  REPORT_NAME_DIGITAL(__LINE__, X_STOP_PIN)
#endif
#if PIN_EXISTS(X2_DIR)
  REPORT_NAME_DIGITAL(__LINE__, X2_DIR_PIN)
#endif
#if PIN_EXISTS(X2_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, X2_ENABLE_PIN)
#endif
#if PIN_EXISTS(X2_STEP)
  REPORT_NAME_DIGITAL(__LINE__, X2_STEP_PIN)
#endif
#if PIN_EXISTS(Y_ATT)
  REPORT_NAME_DIGITAL(__LINE__, Y_ATT_PIN)
#endif
#if PIN_EXISTS(Y_CS)
  REPORT_NAME_DIGITAL(__LINE__, Y_CS_PIN)
#endif
#if PIN_EXISTS(Y_DIR)
  REPORT_NAME_DIGITAL(__LINE__, Y_DIR_PIN)
#endif
#if PIN_EXISTS(Y_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, Y_ENABLE_PIN)
#endif
#if PIN_EXISTS(Y_MAX)
  REPORT_NAME_DIGITAL(__LINE__, Y_MAX_PIN)
#endif
#if PIN_EXISTS(Y_MIN)
  REPORT_NAME_DIGITAL(__LINE__, Y_MIN_PIN)
#endif
#if PIN_EXISTS(Y_MS1)
  REPORT_NAME_DIGITAL(__LINE__, Y_MS1_PIN)
#endif
#if PIN_EXISTS(Y_MS2)
  REPORT_NAME_DIGITAL(__LINE__, Y_MS2_PIN)
#endif
#if PIN_EXISTS(Y_STEP)
  REPORT_NAME_DIGITAL(__LINE__, Y_STEP_PIN)
#endif
#if PIN_EXISTS(Y_STOP)
  REPORT_NAME_DIGITAL(__LINE__, Y_STOP_PIN)
#endif
#if PIN_EXISTS(Y2_DIR)
  REPORT_NAME_DIGITAL(__LINE__, Y2_DIR_PIN)
#endif
#if PIN_EXISTS(Y2_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, Y2_ENABLE_PIN)
#endif
#if PIN_EXISTS(Y2_STEP)
  REPORT_NAME_DIGITAL(__LINE__, Y2_STEP_PIN)
#endif
#if PIN_EXISTS(Z_ATT)
  REPORT_NAME_DIGITAL(__LINE__, Z_ATT_PIN)
#endif
#if PIN_EXISTS(Z_CS)
  REPORT_NAME_DIGITAL(__LINE__, Z_CS_PIN)
#endif
#if PIN_EXISTS(Z_DIR)
  REPORT_NAME_DIGITAL(__LINE__, Z_DIR_PIN)
#endif
#if PIN_EXISTS(Z_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, Z_ENABLE_PIN)
#endif
#if PIN_EXISTS(Z_MAX)
  REPORT_NAME_DIGITAL(__LINE__, Z_MAX_PIN)
#endif
#if PIN_EXISTS(Z_MIN)
  REPORT_NAME_DIGITAL(__LINE__, Z_MIN_PIN)
#endif
#if PIN_EXISTS(Z_MIN_PROBE)
  REPORT_NAME_DIGITAL(__LINE__, Z_MIN_PROBE_PIN)
#endif
#if PIN_EXISTS(Z_MS1)
  REPORT_NAME_DIGITAL(__LINE__, Z_MS1_PIN)
#endif
#if PIN_EXISTS(Z_MS2)
  REPORT_NAME_DIGITAL(__LINE__, Z_MS2_PIN)
#endif
#if PIN_EXISTS(Z_PROBE)
  REPORT_NAME_DIGITAL(__LINE__, Z_PROBE_PIN)
#endif
#if PIN_EXISTS(Z_STEP)
  REPORT_NAME_DIGITAL(__LINE__, Z_STEP_PIN)
#endif
#if PIN_EXISTS(Z_STOP)
  REPORT_NAME_DIGITAL(__LINE__, Z_STOP_PIN)
#endif
#if PIN_EXISTS(Z2_CS)
  REPORT_NAME_DIGITAL(__LINE__, Z2_CS_PIN)
#endif
#if PIN_EXISTS(Z2_DIR)
  REPORT_NAME_DIGITAL(__LINE__, Z2_DIR_PIN)
#endif
#if PIN_EXISTS(Z2_ENABLE)
  REPORT_NAME_DIGITAL(__LINE__, Z2_ENABLE_PIN)
#endif
#if PIN_EXISTS(Z2_MS1)
  REPORT_NAME_DIGITAL(__LINE__, Z2_MS1_PIN)
#endif
#if PIN_EXISTS(Z2_MS2)
  REPORT_NAME_DIGITAL(__LINE__, Z2_MS2_PIN)
#endif
#if PIN_EXISTS(Z2_MS3)
  REPORT_NAME_DIGITAL(__LINE__, Z2_MS3_PIN)
#endif
#if PIN_EXISTS(Z2_STEP)
  REPORT_NAME_DIGITAL(__LINE__, Z2_STEP_PIN)
#endif
#if PIN_EXISTS(X_SERIAL_TX)
  REPORT_NAME_DIGITAL(__LINE__, X_SERIAL_TX_PIN)
#endif
#if PIN_EXISTS(X_SERIAL_RX)
  REPORT_NAME_DIGITAL(__LINE__, X_SERIAL_RX_PIN)
#endif
#if PIN_EXISTS(X2_SERIAL_TX)
  REPORT_NAME_DIGITAL(__LINE__, X2_SERIAL_TX_PIN)
#endif
#if PIN_EXISTS(X2_SERIAL_RX)
  REPORT_NAME_DIGITAL(__LINE__, X2_SERIAL_RX_PIN)
#endif
#if PIN_EXISTS(Y_SERIAL_TX)
  REPORT_NAME_DIGITAL(__LINE__, Y_SERIAL_TX_PIN)
#endif
#if PIN_EXISTS(Y_SERIAL_RX)
  REPORT_NAME_DIGITAL(__LINE__, Y_SERIAL_RX_PIN)
#endif
#if PIN_EXISTS(Y2_SERIAL_TX)
  REPORT_NAME_DIGITAL(__LINE__, Y2_SERIAL_TX_PIN)
#endif
#if PIN_EXISTS(Y2_SERIAL_RX)
  REPORT_NAME_DIGITAL(__LINE__, Y2_SERIAL_RX_PIN)
#endif
#if PIN_EXISTS(Z_SERIAL_TX)
  REPORT_NAME_DIGITAL(__LINE__, Z_SERIAL_TX_PIN)
#endif
#if PIN_EXISTS(Z_SERIAL_RX)
  REPORT_NAME_DIGITAL(__LINE__, Z_SERIAL_RX_PIN)
#endif
#if PIN_EXISTS(Z2_SERIAL_TX)
  REPORT_NAME_DIGITAL(__LINE__, Z2_SERIAL_TX_PIN)
#endif
#if PIN_EXISTS(Z2_SERIAL_RX)
  REPORT_NAME_DIGITAL(__LINE__, Z2_SERIAL_RX_PIN)
#endif
#if PIN_EXISTS(E0_SERIAL_TX)
  REPORT_NAME_DIGITAL(__LINE__, E0_SERIAL_TX_PIN)
#endif
#if PIN_EXISTS(E0_SERIAL_RX)
  REPORT_NAME_DIGITAL(__LINE__, E0_SERIAL_RX_PIN)
#endif
#if PIN_EXISTS(E1_SERIAL_TX)
  REPORT_NAME_DIGITAL(__LINE__, E1_SERIAL_TX_PIN)
#endif
#if PIN_EXISTS(E1_SERIAL_RX)
  REPORT_NAME_DIGITAL(__LINE__, E1_SERIAL_RX_PIN)
#endif
#if PIN_EXISTS(E2_SERIAL_TX)
  REPORT_NAME_DIGITAL(__LINE__, E2_SERIAL_TX_PIN)
#endif
#if PIN_EXISTS(E2_SERIAL_RX)
  REPORT_NAME_DIGITAL(__LINE__, E2_SERIAL_RX_PIN)
#endif
#if PIN_EXISTS(E3_SERIAL_TX)
  REPORT_NAME_DIGITAL(__LINE__, E3_SERIAL_TX_PIN)
#endif
#if PIN_EXISTS(E3_SERIAL_RX)
  REPORT_NAME_DIGITAL(__LINE__, E3_SERIAL_RX_PIN)
#endif
#if PIN_EXISTS(E4_SERIAL_TX)
  REPORT_NAME_DIGITAL(__LINE__, E4_SERIAL_TX_PIN)
#endif
#if PIN_EXISTS(E4_SERIAL_RX)
  REPORT_NAME_DIGITAL(__LINE__, E4_SERIAL_RX_PIN)
#endif
