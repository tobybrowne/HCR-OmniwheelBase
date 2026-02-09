/**
 * Utility script to reassign motor IDs
 */

#include <SCServo.h>
SMS_STS st;

// the pins on the ESP32 used for communicating with the motor driver
#define S_RXD 20 
#define S_TXD 21

int ID_ChangeFrom = 2; // Change the original servo ID, and the factory default is 1
int ID_Changeto = 4; // new ID

void setup(){
  Serial.begin(115200);

  // initialise comms with the motor driver
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(2000);

  // reassign motor ID
  st.unLockEprom(ID_ChangeFrom);
  st.writeByte(ID_ChangeFrom, SMS_STS_ID, ID_Changeto);
  st.LockEprom(ID_Changeto);
  Serial.write("Finished Converting!");
}

void loop(){
}