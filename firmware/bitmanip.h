/*
  bitmanip.h
  Copyright 2017 Jason Pepas
  Released under the terms of the MIT License.  See http://opensource.org/licenses/MIT
*/

#ifndef _BITMANIP_H_
#define _BITMANIP_H_


#define SET_BIT(ADDRESS, BITNUM)   (ADDRESS |= (1<<BITNUM))
#define CLEAR_BIT(ADDRESS, BITNUM) (ADDRESS &= ~(1<<BITNUM))
#define FLIP_BIT(ADDRESS, BITNUM)  (ADDRESS ^= (1<<BITNUM))
// note: this returns a byte with the desired bit in its original position
#define GET_BIT(ADDRESS, BITNUM)   (ADDRESS & (1<<BITNUM))


#endif
