#include "RCMegaReceiver.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Configure Pins A8-A15 to input and enable interrupt
  configureReceiver();
}

void loop() {
  // put your main code here, to run repeatedly:

  int16_t *rcDataTemp = getRcData();
  char message[45];

  snprintf(message, 45, "%d,%d,%d,%d,%d,%d,%d,%d", *(rcDataTemp+0), 
  *(rcDataTemp+1), *(rcDataTemp+2), *(rcDataTemp+3), *(rcDataTemp+4), 
  *(rcDataTemp+5), *(rcDataTemp+6), *(rcDataTemp+7));
  
  Serial.println(message);
}
