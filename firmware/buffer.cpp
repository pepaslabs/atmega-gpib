/*
  buffer.cpp
  Copyright Jason Pepas (Pepas Labs, LLC)
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/


#include "buffer.h"

#include <string.h>


void clear_char_buffer(char_buffer_t *buff)
{
  memset(buff->bytes, 0, buff->len);
}


void clear_uint8_buffer(uint8_buffer_t *buff)
{
  memset(buff->bytes, 0, buff->len);
}


