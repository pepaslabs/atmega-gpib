/*
  serial.h
  Copyright 2017 Jason Pepas
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/

#ifndef SERIAL_T_H
#define SERIAL_T_H

typedef struct _serial_t {
    HardwareSerial *hard;
    SoftwareSerial *soft;
} serial_t;

void serial_setup(serial_t *serial, HardwareSerial *hard, SoftwareSerial *soft);
void serial_begin(serial_t *serial, baud_rate_t baud_rate);

uint8_t serial_available(serial_t *serial);
uint8_t serial_read(serial_t *serial);

uint8_t serial_write_available(serial_t *serial);
size_t serial_write_byte(serial_t *serial, uint8_t data);
size_t serial_write_str(serial_t *serial, const char *str);

#endif
