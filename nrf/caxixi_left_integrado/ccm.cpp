///////////////////
// CCM functions //
///////////////////

boolean isCCMBufferReady(){
  if(GyroXBuffer.getCount() < BUFFER_SIZE
  || GyroYBuffer.getCount() < BUFFER_SIZE
  || GyroZBuffer.getCount() < BUFFER_SIZE
  || accelYBuffer.getCount() < BUFFER_SIZE){
    return false;
  } else {
    CCMbufferReady = true;
    return true;
  }
}

void setIsRollingX(){
  int currentValue = GyroXBuffer.getPreviousElement(1);
  int firstValue = GyroXBuffer.getPreviousElement(3);
  int firstVariation = abs(firstValue - currentValue);
  if(firstVariation > isRollingXVariation){
        isRollingX = true;
       return;
  } else {
    isRollingX = false; }
}

void setIsRollingY(){
  int currentValue = GyroYBuffer.getPreviousElement(1);
  int firstValue = GyroYBuffer.getPreviousElement(3);
  int firstVariation = abs(firstValue - currentValue);
  if(firstVariation > isRollingYVariation){
    isRollingY = true;
    return;
  }else{
    isRollingY = false; }
}

void setIsRollingZ(){
  int currentValue = GyroZBuffer.getPreviousElement(1);
  int firstValue = GyroZBuffer.getPreviousElement(3);
  int firstVariation = abs(firstValue - currentValue);
  if(firstVariation > isRollingZVariation){
    isRollingZ = true;
    return;
  }else{
    isRollingZ = false; }
}

void processX()
{
  int x, controlvalueX;
  controlvalueX = abs(GyroXBuffer.getPreviousElement(1));
  if(controlvalueX>180){
     controlvalueX = 180;}      
  x = map(controlvalueX, 0, 180, 0, 127); // scale to midi range
  SendToReceiver(formatCCM(x,15));
  delay(2);
}

void processY()
{
  int y, controlvalueY;
  controlvalueY = GyroYBuffer.getPreviousElement(1);      
  y = map(controlvalueY, -90, 90, 0, 127); 
  SendToReceiver(formatCCM(y,16));
  delay(2);
}

void processZ()
{
  int z, controlvalueZ;
  controlvalueZ = GyroZBuffer.getPreviousElement(1);
  z = map(controlvalueZ, -90, 90, 0, 127); 
  SendToReceiver(formatCCM(z,17));
  delay(2);
}


void ccmNotes() {
  if (currentAccelY > NoteThresholdCCM & (!Bloqueo)){
    if (NoteState){
       SendToReceiver(CAXIXI_RIGHT_HIT_NOTEON);
       NoteState = false;
       Bloqueo = true;
       }
    else{
      SendToReceiver(CAXIXI_RIGHT_HIT_NOTEOFF);
      NoteState = true;
      Bloqueo = true;
      }
    }
  if (currentAccelY < NoteReleaseCCM){
    Bloqueo = false;
    }
} 

void areRolling() {
  setIsRollingX();
  setIsRollingY();
  setIsRollingZ();  
}

void ProcessCCM() {
  if(isRollingX){
    processX();
    }
  if(isRollingY){
    processY();
    }
  if(isRollingZ){
    processZ();
    }
}

void runCCM() {
  ButtonOctaveDown();
  currentAccelY = accelYBuffer.getPreviousElement(1);
  ccmNotes();
  areRolling();
  ProcessCCM();
  }

int formatCCM(int NUM, int CH) {
  msg = CH*1000 + NUM;
  return msg;
}
