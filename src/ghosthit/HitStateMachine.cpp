#include "HitStateMachine.h"

HitStateMachine::HitStateMachine()
{
        curState = HITSTATE_STOPPED;
}

HitStateMachine::~HitStateMachine()
{
}

HitEvent HitStateMachine::update(CxCircularBuffer * fifo)
{
        HitEvent ev = HITEVENT_NONE;
        switch(curState){
        case HITSTATE_STOPPED:
                ev = updateStopped(fifo);
                break;
        case HITSTATE_MOVING_UP:
                ev = updateMovingUp(fifo);
                break;
        case HITSTATE_MOVING_DOWN:
                ev = updateMovingDown(fifo);
                break;
        case HITSTATE_ROTATED:
                ev = updateRotated(fifo);
                break;
        default:
                break;
        }

        return ev;
}

HitEvent HitStateMachine::updateStopped(CxCircularBuffer * fifo)
{
        HitEvent ev = HITEVENT_NONE;
        return ev;
}

HitEvent HitStateMachine::updateMovingUp(CxCircularBuffer * fifo)
{
        HitEvent ev = HITEVENT_NONE;
        return ev;
}

HitEvent HitStateMachine::updateMovingDown(CxCircularBuffer * fifo)
{
        HitEvent ev = HITEVENT_NONE;
        return ev;
}

HitEvent HitStateMachine::updateRotated(CxCircularBuffer * fifo)
{
        HitEvent ev = HITEVENT_NONE;
        return ev;
}
