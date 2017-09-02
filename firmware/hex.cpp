/*
  hex.cpp
  Copyright 2017 Jason Pepas
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/


#include "hex.h"


bool hex_to_nibble(char ch, uint8_t *out) {
  switch (ch) {
      case '0':
          *out = 0x0;
          return true;
      case '1':
          *out = 0x1;
          return true;
      case '2':
          *out = 0x2;
          return true;
      case '3':
          *out = 0x3;
          return true;
      case '4':
          *out = 0x4;
          return true;
      case '5':
          *out = 0x5;
          return true;
      case '6':
          *out = 0x6;
          return true;
      case '7':
          *out = 0x7;
          return true;
      case '8':
          *out = 0x8;
          return true;
      case '9':
          *out = 0x9;
          return true;
      case 'a':
      case 'A':
          *out = 0xA;
          return true;
      case 'b':
      case 'B':
          *out = 0xB;
          return true;
      case 'c':
      case 'C':
          *out = 0xC;
          return true;
      case 'd':
      case 'D':
          *out = 0xD;
          return true;
      case 'e':
      case 'E':
          *out = 0xE;
          return true;
      case 'f':
      case 'F':
          *out = 0xF;
          return true;
      default:
          return false;
  }
}


bool hex_to_byte(char high, char low, uint8_t *out) {
  // originally inspired by http://stackoverflow.com/a/12839870/558735

  uint8_t high_bin = 0x0;
  uint8_t low_bin = 0x0;
  bool ret;
  
  ret = hex_to_nibble(high, &high_bin);
  if (ret == false) {
      return ret;
  }

  ret = hex_to_nibble(low, &low_bin);
  if (ret == false) {
      return ret;
  }

  *out = (high_bin << 4) | low_bin;
  return true;
}


void byte_to_hex(uint8_t data, char *buff) {
  // originally inspired by http://stackoverflow.com/a/12839870/558735

  char map[16+1] = "0123456789ABCDEF";

  uint8_t high_nibble = (data & 0xF0) >> 4;
  *buff = map[high_nibble];
  buff++;

  uint8_t low_nibble = data & 0x0F;
  *buff = map[low_nibble];
  buff++;

  *buff = '\0';
}
