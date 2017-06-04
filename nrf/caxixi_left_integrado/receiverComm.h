#ifndef RECEIVERCOMM_H
#define RECEIVERCOMM_H
void SendToReceiver(int msg);
void SendNoteOn(int note);
void SendNoteOff(int note);
void SendRecordStart();
void SendRecordStop();
void SendOctaveUp();
void SendOctaveDown();
void SendClear();
#endif