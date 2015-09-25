#include "HitStateMachine.h"

HitStateMachine::HitStateMachine()
{
	curState = HITSTATE_STILL;
}

HitStateMachine::~HitStateMachine()
{
}

HitEvent HitStateMachine::update(CxCircularBuffer * AccelXBuffer, CxCircularBuffer * AccelYBuffer)
{
	HitEvent ev = HITEVENT_NONE;
	switch(curState){
		case HITSTATE_STILL:
			ev = updateStopped(AccelXBuffer, AccelYBuffer);
			break;
		case HITSTATE_MOVING_FORWARD:
			ev = updateMovingForward(AccelXBuffer, AccelYBuffer);
			break;
		case HITSTATE_MOVING_BACKWARD:
			ev = updateMovingBackward(AccelXBuffer, AccelYBuffer);
			break;
		default:
			break;
	}
	return ev;
}

HitEvent HitStateMachine::updateStopped(CxCircularBuffer * AccelXBuffer, CxCircularBuffer * AccelYBuffer)
{
	HitEvent ev = HITEVENT_NONE;
	// Check if it's moving up or down
	// and update it's status
	if(isForwardThreshold(AccelXBuffer, AccelYBuffer)){
		curState = HITSTATE_MOVING_FORWARD;
	}
	if(isBackwardThreshold(AccelXBuffer, AccelYBuffer)){
		curState = HITSTATE_MOVING_BACKWARD;
	}
	return ev;
}

boolean HitStateMachine::isForwardThreshold(CxCircularBuffer * AccelXBuffer, CxCircularBuffer * AccelYBuffer)
{
	int currentAccel = AccelXBuffer->getPreviousElement(1);
	if(currentAccel > 400){
		return true;
	}else{
		return false;
	}
}

boolean HitStateMachine::isBackwardThreshold(CxCircularBuffer * AccelXBuffer, CxCircularBuffer * AccelYBuffer)
{
	int currentAccel = AccelXBuffer->getPreviousElement(1);
	if(currentAccel < -200){
		return true;
	}else{
		return false;
	}
}

HitEvent HitStateMachine::updateMovingForward(CxCircularBuffer * AccelXBuffer, CxCircularBuffer * AccelYBuffer)
{
	HitEvent ev = HITEVENT_NONE;
	// Check if it's moving up or down
	// and update it's status
	int currentAccel = AccelXBuffer->getPreviousElement(1);
	if(currentAccel < -300){
		ev = HITEVENT_BACKWARD;
		curState = HITSTATE_MOVING_BACKWARD;
	}else{
		int prevAccel = AccelXBuffer->getPreviousElement(4);
		if(abs(currentAccel - prevAccel) < 20 && abs(currentAccel) < 200){
			curState = HITSTATE_STILL;
		}
	}
	return ev;
}

HitEvent HitStateMachine::updateMovingBackward(CxCircularBuffer * AccelXBuffer, CxCircularBuffer * AccelYBuffer)
{
	HitEvent ev = HITEVENT_NONE;
	// Check if it's moving up or down
	// and update it's status
	int currentAccel = AccelXBuffer->getPreviousElement(1);
	if(currentAccel > 400){
		ev = HITEVENT_FORWARD;
		curState = HITSTATE_MOVING_FORWARD;
	}else{
		int prevAccel = AccelXBuffer->getPreviousElement(4);
		if(abs(currentAccel - prevAccel) < 20 && abs(currentAccel) < 200){
			curState = HITSTATE_STILL;
		}
	}
	return ev;
}
