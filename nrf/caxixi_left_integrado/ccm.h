#ifndef CCM_H
#define CCM_H
boolean isCCMBufferReady();
void setIsRollingX();
void setIsRollingY();
void setIsRollingZ();
void processX();
void processY();
void processZ();
void ccmNotes();
void areRolling();
void ProcessCCM();
void runCCM();
int formatCCM(int NUM, int CH);
int digitalSmooth(int rawIn, int *sensSmoothArray);
#endif
