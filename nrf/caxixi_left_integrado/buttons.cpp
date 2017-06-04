//CAXIXI LEFT BUTTONS
///////BUTTON CLEAR////
void ButtonClear() {
  clearButtonState = digitalRead(SAMPLER_BUTTON_CLEAR_PIN);
  if (clearButtonState != lastClearButtonState) {// if the state has changed, increment the counter
    if (clearButtonState == HIGH) {// if the current state is HIGH then the button wend from off to on:
    SendClear();
    }
    lastClearButtonState = clearButtonState; // save the current state as the last state, for next time through the loop
  }
}

//////////////////////
//////BUTTON OCTAVE DOWN////
void ButtonOctaveDown() {
  octaveDownButtonState = digitalRead(OCTAVE_DOWN_BUTTON_PIN);
  if (octaveDownButtonState != octaveDownButtonLastState) {
    if (octaveDownButtonState == HIGH){
      firstTime = millis();
      if (!wantCCM) {
        //currentOctave = currentOctave-1; 
        SendOctaveDown();
      }
    }
        millis_held = (millis() - firstTime);
    if (octaveDownButtonState == LOW && millis_held > longPressThreshold) {
        //Serial.print("\nChange wantCCM");
        if (!wantCCM) {
        SendOctaveUp(); //Revisar, para compensar el octaveUp indeseado del if anterior
        }
        wantCCM = !wantCCM;
      }  
    octaveDownButtonLastState = octaveDownButtonState;
  }
}
//CAXIXI RIGHT BUTTONS////
//////BUTTON RECORD////
void ButtonRecord() {
  recordButtonState = digitalRead(SAMPLER_BUTTON_RECORD_PIN);
  if (recordButtonState != lastRecordButtonState) {// if the state has changed, increment the counter
    if (recordButtonState == HIGH) {// if the current state is HIGH then the button wend from off to on:
      if (record==true){
        record=false;
        SendRecordStop();
      } else {
        record=true;
        SendRecordStart();
      }
    }
    lastRecordButtonState = recordButtonState; // save the current state as the last state, for next time through the loop
  }
}
//////////////////////////
//////BUTTON OCTAVE UP////
void ButtonOctaveUp() {
  octaveUpButtonState = digitalRead(OCTAVE_UP_BUTTON_PIN);
  if (octaveUpButtonState != octaveUpButtonLastState) {
    if (octaveUpButtonState == HIGH){
      firstTime = millis();
      if (!wantCCM) {
        //currentOctave = currentOctave+1; 
        SendOctaveUp();
      }
    }
    millis_held = (millis() - firstTime);
    if (octaveUpButtonState == LOW && millis_held > longPressThreshold) {
        //Serial.print("\nChange wantCCM");
        if (!wantCCM) {
        SendOctaveDown(); //Revisar, para compensar el octaveUp indeseado del if anterior
        }
        wantCCM = !wantCCM;
      }      
    octaveUpButtonLastState = octaveUpButtonState;
  }
}




/*
void ButtonOctaveUp() {
  octaveUpButtonState = digitalRead(OCTAVE_UP_BUTTON_PIN);
  if (octaveUpButtonState != octaveUpButtonLastState) {
    if (octaveUpButtonState == HIGH){
      firstTime = millis();
      if (!wantCCM) {
        //currentOctave = currentOctave+1; 
        SendOctaveUp();
      }
    }
    millis_held = (millis() - firstTime);
    if (octaveUpButtonState == LOW && millis_held > longPressThreshold) {
        //Serial.print("\nChange wantCCM");
        if (!wantCCM) {
        SendOctaveDown(); //Revisar, para compensar el octaveUp indeseado del if anterior
        }
        wantCCM = !wantCCM;
      }      
    octaveUpButtonLastState = octaveUpButtonState;
  }
}*/

/////////////////////////
