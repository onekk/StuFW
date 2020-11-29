/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
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

#ifdef ARDUINO_ARCH_SAM

#include "../../../StuFW.h"

#if HAS_EEPROM

MemoryStore memorystore;

extern void eeprom_flush(void);

#if HAS_EEPROM_SD
  char MemoryStore::eeprom_data[EEPROM_SIZE];
#endif

bool MemoryStore::access_read() {
  #if HAS_EEPROM_SD
    card.open_eeprom_sd(true);
    size_t bytes_read = card.read_eeprom_data(eeprom_data, EEPROM_SIZE);
    if (bytes_read != EEPROM_SIZE) SERIAL_STR(ER);
    else SERIAL_STR(ECHO);
    SERIAL_EMV("SD EEPROM bytes read: ", (int)bytes_read);
    card.close_eeprom_sd();
    return (bytes_read != EEPROM_SIZE);
  #else
    return false;
  #endif

}

bool MemoryStore::access_write() {
  #if HAS_EEPROM_FLASH
    eeprom_flush();
    return false;
  #elif HAS_EEPROM_SD
    card.open_eeprom_sd(false);
    size_t bytes_written = card.write_eeprom_data(eeprom_data, EEPROM_SIZE);
    const bool error_read = bytes_written != EEPROM_SIZE;
    if (error_read) SERIAL_STR(ER);
    else SERIAL_STR(ECHO);
    SERIAL_EMV("SD EEPROM bytes written: ", (int)bytes_written);
    card.close_eeprom_sd();
    return error_read;
  #else
    return false;
  #endif
}

bool MemoryStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc) {

  while(size--) {
    uint8_t v = *value;
    #if HAS_EEPROM_SD
      eeprom_data[pos] = v;
    #else
      uint8_t * const p = (uint8_t * const)pos;
      // EEPROM has only ~100,000 write cycles,
      // so only write bytes that have changed!
      if (v != eeprom_read_byte(p)) {
        eeprom_write_byte(p, v);
        if (eeprom_read_byte(p) != v) {
          SERIAL_LM(ECHO, MSG_ERR_EEPROM_WRITE);
          return true;
        }
      }
    #endif
    crc16(crc, &v, 1);
    pos++;
    value++;
  };

  return false;
}

bool MemoryStore::read_data(int &pos, uint8_t *value, size_t size, uint16_t *crc, const bool writing/*=true*/) {

  while(size--) {
    #if HAS_EEPROM_SD
      uint8_t c = eeprom_data[pos];
    #else
      uint8_t c = eeprom_read_byte((uint8_t*)pos);
    #endif
    if (writing) *value = c;
    crc16(crc, &c, 1);
    pos++;
    value++;
  };

  return false;
}

size_t MemoryStore::capacity() { return EEPROM_SIZE + 1; }

#endif // HAS_EEPROM

#endif // ARDUINO_ARCH_SAM
