#ifndef HITSTATEMACHINE_H
#define HITSTATEMACHINE_H

#include "CxCircularBuffer.h"
#include "CaxixiConfig.h"

typedef enum HitEvent{
	HITEVENT_NONE,
	HITEVENT_FORWARD,
	HITEVENT_BACKWARD,
	HITEVENT_ROTATED
};

typedef enum HitState{
	HITSTATE_STILL,
	HITSTATE_MOVING_FORWARD,
	HITSTATE_MOVING_BACKWARD,
	HITSTATE_HIT_FORWARD,
	HITSTATE_HIT_BACKWARD,
	HITSTATE_ROTATED
};

class HitStateMachine
{
public:
	HitStateMachine();
	~HitStateMachine();

	HitEvent update(CxCircularBuffer * AccelX, CxCircularBuffer * AccelY);

protected:
	HitEvent updateStopped(CxCircularBuffer * AccelXBuffer, CxCircularBuffer * AccelYBuffer);
	HitEvent updateMovingForward(CxCircularBuffer * AccelXBuffer, CxCircularBuffer * AccelYBuffer);
	HitEvent updateMovingBackward(CxCircularBuffer * AccelXBuffer, CxCircularBuffer * AccelYBuffer);
	boolean isForwardThreshold(CxCircularBuffer * AccelXBuffer, CxCircularBuffer * AccelYBuffer);
	boolean isBackwardThreshold(CxCircularBuffer * AccelXBuffer, CxCircularBuffer * AccelYBuffer);


	HitState curState;
};

#endif