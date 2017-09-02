/*
  buffer.cpp
  Copyright 2017 Jason Pepas
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/


#include "buffer.h"

#include <string.h>


void clear_buffer(buffer_t *buff)
{
    memset(buff->bytes, 0, buff->len);
}

