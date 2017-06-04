#ifndef CxCircularBuffer_h
#define CxCircularBuffer_h

/*
 * based on: http://playground.arduino.cc/Main/runningAverage
 */

#define CXCIRCULARBUFFER_LIB_VERSION "0.1"

#include "Arduino.h"

class CxCircularBuffer
{
public:
    CxCircularBuffer(void);
    CxCircularBuffer(int);
    ~CxCircularBuffer();

    void clear();
    void addValue(int);
    void fillValue(int, int);

    int getElement(uint8_t idx);
    uint8_t getSize() { return _size; }
    uint8_t getCount() { return _cnt; }
    uint8_t getCurrentIdx() { return _idx; }
		int getPreviousElement(uint8_t number);
		
protected:
    uint8_t _size;
    uint8_t _cnt;
    uint8_t _idx;
    int * _ar;
};

#endif
