#include "CxCircularBuffer.h"
#include <stdlib.h>


CxCircularBuffer::CxCircularBuffer(int n)
{
    _size = n;
    _ar = (int*) malloc(_size * sizeof(int));
    if (_ar == NULL) _size = 0;
    clear();
}

CxCircularBuffer::~CxCircularBuffer()
{
    if (_ar != NULL) free(_ar);
}

// resets all counters
void CxCircularBuffer::clear()
{
    _cnt = 0;
    _idx = 0;
    for (int i = 0; i< _size; i++) _ar[i] = 0;  // needed to keep addValue simple
}

// adds a new value to the data-set
void CxCircularBuffer::addValue(int f)
{
    if (_ar == NULL) return;
    _ar[_idx] = f;
    _idx++;
    if (_idx == _size) _idx = 0;  // faster than %
    if (_cnt < _size) _cnt++;
}

// returns the value of an element if exist, 0 otherwise
int CxCircularBuffer::getElement(uint8_t idx)
{
    if (idx >=_cnt ) return NAN;
    return _ar[idx];
}

int CxCircularBuffer::getPreviousElement(uint8_t number)
{
	int i = _idx-number;
	if(i<0){
		i = _size - abs(i);
	}
	return _ar[i];
}


