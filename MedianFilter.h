/*
  MedianFilter.h - Median Filter for the Arduino platform.
  Copyright (c) 2013 Phillip Schmidt.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/* 
	A median filter object is created by by passing the desired filter window size on object creation.
	The window size should be an odd number between 3 and 255.  
  
	New data is added to the median filter by passing the data through the write() method.
	The new data will over-write the oldest data point, then be shifted in the array to place it in the correct location. 
  
	The current median value is returned by the read() method.
 
	!!! All data must be type INT.  !!!
 */

#ifndef MedianFilter_h
#define MedianFilter_h

#include "Arduino.h"

#ifdef  __cplusplus
class MedianFilter
{
public:
  MedianFilter(byte size, int16_t seed);
  int16_t in(int16_t value);
  int16_t out();
  //void printData();		// used for debugging

private:
  int16_t filterShift;
  byte medFilterWin;		// number of samples in sliding median filter window - usually odd #
  byte medDataPointer;	// mid point of window
  int16_t * sortedData;		// array pointer for data sorted by size
  byte * historyMap;		// array pointer for locations of history data in sorted list
  byte * locationMap;		// array pointer for data locations in history map
  int16_t ODP;				// oldest data point in accelRawHistory
  int16_t tempData;			// temp data storage while swapping data locations
  byte tempMap;			// temp map storage while swapping data locations
  boolean dataMoved;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif
typedef void* median_filter_t;

// Need an explicit constructor and destructor.
median_filter_t median_filter_new(byte size, int16_t seed);
int16_t median_filter_in(median_filter_t filter, int16_t value);
int16_t median_filter_out(median_filter_t filter);
#ifdef __cplusplus
}
#endif

#endif

