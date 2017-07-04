/*
  buffer.h
  Copyright Jason Pepas (Pepas Labs, LLC)
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/

#ifndef _BUFFER_H_
#define _BUFFER_H_

#define __STDC_LIMIT_MACROS
#include <stdint.h>


struct _char_buffer_t
{
  uint8_t len;
  char *bytes;
};
typedef _char_buffer_t char_buffer_t;


struct _uint8_buffer_t
{
  uint8_t len;
  uint8_t *bytes;
};
typedef _uint8_buffer_t uint8_buffer_t;


void clear_char_buffer(char_buffer_t *buff);
void clear_uint8_buffer(uint8_buffer_t *buff);


#endif

