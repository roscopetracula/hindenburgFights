/* DVR8830_test by Thomas Olson 20140716.1
DVR8830_test_thomasOlson
*/

#include <Wire.h>	
#include <RFduinoBLE.h>

#define Vref = 1.285

//Vmoter = 4 * Vref * (Vset + 1) / 64
//Vset = (Vmoter * 64 / (4 * Vref)) - 1;

#define MOTOR1 0x67
#define MOTOR2 0x64
#define MOTOR3 0x61
#define CONTROL 0x00
#define FAULT 0x01
int Vset;
int timeout=1000;
int lastPing;
int thisLoopMillis=0;
bool timeoutPossible=0;
int faultPin = 4;
int motors [3] = { MOTOR1, MOTOR2, MOTOR3 };

void setup(){
  pinMode(faultPin, INPUT);
  Wire.beginOnPins(5,6); //SCL pin, SDA pin //5,6 for slimstack board v1
  delay(20);
  Serial.begin(9600);
  
  Serial.println("\nDVR8830 Motor Controller");
  
  RFduinoBLE.deviceName = "RFduino Blimp"; //Sets the device name  
  //RFduinoBLE.advertisementData = "temp"; //data in the advertisement  
  RFduinoBLE.begin();  // start the BLE stack
  delay(25);
  
  //Wire.begin();
  Vset = 0x3F;
  lastPing = millis();
 
  Serial.println("Test motors"); 
  
  // clear faults
  runCommandOnAll('c');
  
  // startup check: forward, brake, reverse, brake
  char sequences [4] = { 'f', 'b', 'r', 'b' };
  for ( int j=sizeof(motors); j>=0; --j ){
    // not as efficient but more intuitive to keep it in the right order
    for ( int i=0; i<sizeof(sequences); i++ ){
      switch ( sequences[i] ){
        case 'f': setForward(motors[j]);
          break;
        case 'r': setReverse(motors[j]);
          break;
        case 'b': setBrake(motors[j]);
          break;
      }
      // allow the motors time to spin up / down
      delay( 100 );
    }
  }

  Serial.println("Ready to go!"); 
}

void loop(){
  // get the current time
  thisLoopMillis=millis();
  
  if ( digitalRead(faultPin)==LOW ){
    Serial.println(" ----FAULT----  ");
    // Clear faults on all motors
    runCommandOnAll('c');
  }
  
  // Brake if there is no communication for > 1s (timeout)
  if ( (thisLoopMillis-lastPing)>timeout && timeoutPossible==1 ){
    Serial.println(" ----TIMED OUT----  ");
    Serial.println("lastPing timeout:  "+lastPing);
    Serial.println("thisLoopMillis:    "+thisLoopMillis);
    // Brake all motors
    runCommandOnAll('b');
    //can only timeout once
    timeoutPossible=0; 
  }

  // switch to lower power mode
  RFduino_ULPDelay(INFINITE);
}

void runCommandOnAll( char command ){
  for ( int j=sizeof(motors); j>=0; --j ){
    switch (command){
      case 'f': setForward(motors[j]); break;
      case 'r': setReverse(motors[j]); break;
      case 'b': setBrake(motors[j]);   break;
      case 'c': clearFault(motors[j]); break;
    }
  }
}

void RFduinoBLE_onReceive(char *data, int len) {
  // Track the last ping every time we recieve data to detect timeouts
  lastPing=millis();
  Serial.println("lastPing set:    "+lastPing);
  
  // Clear faults on all motors
  runCommandOnAll('c');
  
  // each transmission should contain an RGB triple
  if (len >= 3){
    // see which motor bit is set and then use that character to do something on the motor
    for (int i=2; i>=0; i--){
      switch (data[i]){
        case 0x01: 
          setForward(motors[i]); break;
        case 0x02: 
          setReverse(motors[i]); break;
        default:
          setBrake(motors[i]);
      }
    }
  }
  timeoutPossible=1; // now that we've recieved data it's possible to timeout.
}

void RFduinoBLE_onConnect() {
  runCommandOnAll('f');
  delay(200);
  runCommandOnAll('b');
}

void RFduinoBLE_onDisconnect() {
  runCommandOnAll('r');
  delay(200);
  runCommandOnAll('b');
}

void getFault(int thisMotor){
  uint8_t RegisterFault;
  
  Wire.beginTransmission(thisMotor);
  Wire.write(FAULT);
  Serial.print("err: ");
  Serial.println(Wire.endTransmission());
  
  Wire.requestFrom(thisMotor,1);
  while(Wire.available()){
    RegisterFault = Wire.read();
    Serial.print("RegisterFault: ");
    Serial.println(RegisterFault, HEX);
    
    if(RegisterFault & 0x01) //fault bit
      Serial.print(" FAULT: ");
      
    if(RegisterFault & 0x02) //OCP event
      Serial.print(" OCP: ");
  
    if(RegisterFault & 0x04) //UVLO event
      Serial.print(" UVLO: ");
  
    if(RegisterFault & 0x08) //OTS event
      Serial.print(" OTS: ");
  
    if(RegisterFault & 0x10) //ILIMIT event
      Serial.print(" ILIMIT: ");
  
    Serial.println("");    
  }
}

void getControl(int thisMotor){
  uint8_t RegisterControl;
  
  Wire.beginTransmission(thisMotor);
  Wire.write(CONTROL);
  Serial.print("getControl: ");
  Serial.println(Wire.endTransmission());
  
  Wire.requestFrom(thisMotor,1);
  while(Wire.available()){
    RegisterControl = Wire.read();
    Serial.print("RegisterControl: ");
    Serial.println(RegisterControl, HEX);
  }
}

void setControl(byte thisMotor, uint8_t control){
  Wire.beginTransmission(thisMotor);
  Wire.write(CONTROL);
  Wire.write(control);
  Serial.print("setControl ");
  Serial.println(Wire.endTransmission());
}
  
void clearFault(byte thisMotor){
  Wire.beginTransmission(thisMotor);
  Wire.write(FAULT);
  Wire.write(0x80); // CLEAR bit clears faults
  Serial.print("clear fault ");
  Serial.println(Wire.endTransmission());
}

void setBrake(byte thisMotor){
  Wire.beginTransmission(thisMotor);
  Wire.write(CONTROL);
  Wire.write((Vset << 2) | 0x03); 
  Serial.print("brake ");
  Serial.println(Wire.endTransmission());
}

void setCoast(byte thisMotor){
  Wire.beginTransmission(thisMotor);
  Wire.write(CONTROL);
  Wire.write((Vset << 2) | 0x00); 
  Serial.print("coast ");
  Serial.println(Wire.endTransmission());
}

void setForward(byte thisMotor){
  Wire.beginTransmission(thisMotor);
  Wire.write(CONTROL);
  Wire.write((Vset << 2) | 0x01); 
  Serial.print("forward ");
  Serial.println(Wire.endTransmission());
}

void setReverse(byte thisMotor){
  Wire.beginTransmission(thisMotor);
  Wire.write(CONTROL);
  Wire.write((Vset << 2) | 0x02); 
  Serial.print("reverse ");
  Serial.println(Wire.endTransmission());
}
