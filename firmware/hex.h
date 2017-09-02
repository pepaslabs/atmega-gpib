/*
  hex.h
  Copyright 2017 Jason Pepas
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/

#ifndef _HEX_H_
#define _HEX_H_

#include <stdint.h>
#include <stdbool.h>

bool hex_to_nibble(char ch, uint8_t *out);
bool hex_to_byte(char high, char low, uint8_t *out);
void byte_to_hex(uint8_t data, char *buff);

#endif
