void setCircularBuffer(){
  smoothAccelX = digitalSmooth(SensorRead[SENSOR_ACCEL_X], accelXSmooth);
  smoothAccelY = digitalSmooth(SensorRead[SENSOR_ACCEL_Y], accelYSmooth);
  accelXBuffer.addValue(smoothAccelX);
  accelYBuffer.addValue(smoothAccelY);
}

boolean isBufferReady(){
  if(accelXBuffer.getCount() < BUFFER_SIZE
  || accelYBuffer.getCount() < BUFFER_SIZE){
    return false;
  }else{
    bufferReady = true;
    return true;
  }
}

void setSlopeStill()
{
  if(abs(currentAccelX - accelXBuffer.getPreviousElement(8)) < SLOPE_STILL_X_RANGE
  && abs(currentAccelY - accelYBuffer.getPreviousElement(8)) < SLOPE_STILL_Y_RANGE){
    slopeStill = SLOPE_STILL_STATUS; 
  }else{
    slopeStill = SLOPE_MOVING_STATUS;
  }
}

void setAccelXForce()
{
  if(currentAccelX > FORCE_THRESHOLD_FORWARD || currentAccelY > NOTE_THRESHOLD_HIT){
    accelXForce = FORCE_FORWARD;
  }
  if(currentAccelX < FORCE_THRESHOLD_BACKWARD){
    accelXForce = FORCE_BACKWARD;
  }
  if(currentAccelX < FORCE_STILL_RANGE_FORWARD && currentAccelX > FORCE_STILL_RANGE_BACKWARD){
    accelXForce = FORCE_STILL;
  }
}

void setCircularBufferCCM(){
  smoothAccelY = digitalSmooth(SensorRead[SENSOR_ACCEL_Y], accelYSmooth);
  accelYBuffer.addValue(smoothAccelY);
  smoothGyroX = digitalSmooth(SensorRead[SENSOR_GYRO_X], GyroXSmooth);
  GyroXBuffer.addValue(smoothGyroX);
  smoothGyroY = digitalSmooth(SensorRead[SENSOR_GYRO_Y], GyroYSmooth);
  GyroYBuffer.addValue(smoothGyroY);
  smoothGyroZ = digitalSmooth(SensorRead[SENSOR_GYRO_Z], GyroZSmooth);
  GyroZBuffer.addValue(smoothGyroZ);
  }
  
void setState()
{
  if(slopeStill == SLOPE_STILL_STATUS && accelXForce == FORCE_STILL){
    state = STATE_STILL;
  }else if(slopeStill == SLOPE_MOVING_STATUS && accelXForce == FORCE_FORWARD){
    state = STATE_FORWARD;
  }else if(slopeStill == SLOPE_MOVING_STATUS && accelXForce == FORCE_BACKWARD){
    state = STATE_BACKWARD;
  }else{
    state = prevState;
  }
}