void SendToReceiver(int msg) 
{
  radio.write( &msg, sizeof(int));
}

void SendNoteOn(int note)
{
  SendToReceiver(note);
}

void SendNoteOff(int note)
{
  SendToReceiver(note);
}

void SendRecordStart()
{
  SendToReceiver(CAXIXI_RECORD_START);
}

void SendRecordStop()
{
  SendToReceiver(CAXIXI_RECORD_STOP);
}

void SendOctaveUp()
{
  SendToReceiver(CAXIXI_OCTAVE_UP);
}

void SendOctaveDown()
{
  SendToReceiver(CAXIXI_OCTAVE_DOWN);
}

void SendClear()
{
  SendToReceiver(CAXIXI_SAMPLER_CLEAR);
}