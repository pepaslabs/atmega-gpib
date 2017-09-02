/*
  buffer.h
  Copyright 2017 Jason Pepas
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/

#ifndef _BUFFER_H_
#define _BUFFER_H_

#define __STDC_LIMIT_MACROS
#include <stdint.h>


struct _buffer_t
{
    uint8_t len;
    union {
        char *str;
        uint8_t *bytes;  
    };
};
typedef _buffer_t buffer_t;


void clear_buffer(buffer_t *buff);


#endif
