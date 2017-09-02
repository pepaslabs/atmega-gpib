/*
  parse.cpp
  Copyright 2017 Jason Pepas
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/


#include "parse.h"

#include <stddef.h>

bool is_digit(char ch) {
    return (ch >= '0' && ch <= '9');
}

bool parse_digit(char ch, uint8_t *out) {
    switch (ch) {
        case '0':
            *out = 0;
            return true;
        case '1':
            *out = 1;
            return true;
        case '2':
            *out = 2;
            return true;
        case '3':
            *out = 3;
            return true;
        case '4':
            *out = 4;
            return true;
        case '5':
            *out = 5;
            return true;
        case '6':
            *out = 6;
            return true;
        case '7':
            *out = 7;
            return true;
        case '8':
            *out = 8;
            return true;
        case '9':
            *out = 9;
            return true;
        default:
            return false;
    }
}

bool scan_for_uint(char *str, char **first, char **last) {
    bool ret = false;
    *first = NULL;
    for (uint8_t i = 0; i < UINT8_MAX; i++) {
        if (str[i] == '\0') { break; }
        if (is_digit(str[i])) {
            if (*first == NULL) {
                *first = str + i;
            }
            *last = str + i;
            ret = true;
        } else {
            break;
        }
    }
    return ret;
}

// Warning: Does not detect overflow!  Don't parse anything over 255;
bool parse_uint8(char *str, uint8_t *out) {
    char *first;
    char *last;
    bool ret = scan_for_uint(str, &first, &last);
    if (ret == false) { return ret; }

    uint8_t accumulator = 0;
    uint8_t scale = 1;
    char *ch = last;
    while (ch >= first) {
        uint8_t digit = 0;
        parse_digit(*ch, &digit);
        accumulator += scale * digit;
        ch -= 1;
        scale *= 10;
    }
    *out = accumulator;
    return true;            
}

// Warning: Does not detect overflow!  Don't parse anything over 65535;
bool parse_uint16(char *str, uint16_t *out) {
    char *first;
    char *last;
    bool ret = scan_for_uint(str, &first, &last);
    if (ret == false) { return ret; }

    uint16_t accumulator = 0;
    uint16_t scale = 1;
    char *ch = last;
    while (ch >= first) {
        uint8_t digit = 0;
        parse_digit(*ch, &digit);
        accumulator += scale * digit;
        ch -= 1;
        scale *= 10;
    }
    *out = accumulator;
    return true;            
}

// Warning: Does not detect overflow!  Don't parse anything over 4294967295;
bool parse_uint32(char *str, uint32_t *out) {
    char *first;
    char *last;
    bool ret = scan_for_uint(str, &first, &last);
    if (ret == false) { return ret; }

    uint32_t accumulator = 0;
    uint32_t scale = 1;
    char *ch = last;
    while (ch >= first) {
        uint8_t digit = 0;
        parse_digit(*ch, &digit);
        accumulator += scale * digit;
        ch -= 1;
        scale *= 10;
    }
    *out = accumulator;
    return true;            
}
