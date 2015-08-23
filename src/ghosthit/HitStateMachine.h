#ifndef HITSTATEMACHINE_H
#define HITSTATEMACHINE_H


#include "CxCircularBuffer.h"

typedef enum HitEvent{
        HITEVENT_NONE,
        HITEVENT_UP,
        HITEVENT_DOWN,
        HITEVENT_ROTATED
};

typedef enum HitState{
        HITSTATE_STOPPED,
        HITSTATE_MOVING_UP,
        HITSTATE_MOVING_DOWN,
        HITSTATE_ROTATED
};

class HitStateMachine
{
public:
        HitStateMachine();
        ~HitStateMachine();

        HitEvent update(CxCircularBuffer * fifo);

protected:
        HitEvent updateStopped(CxCircularBuffer * fifo);
        HitEvent updateMovingUp(CxCircularBuffer * fifo);
        HitEvent updateMovingDown(CxCircularBuffer * fifo);
        HitEvent updateRotated(CxCircularBuffer * fifo);

        HitState curState;
};

#endif
