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

void setup()
{
  pinMode(faultPin, INPUT);
  Wire.beginOnPins(5,6); //SCL pin, SDA pin //5,6 for slimstack board v1
  delay(20);
  Serial.begin(9600);
  Serial.println("\nDVR8830 Motor Controller");
  RFduinoBLE.deviceName = "RFduino Blimp"; //Sets the device name  
//  RFduinoBLE.advertisementData = "temp"; //data in the advertisement  
  RFduinoBLE.begin();  // start the BLE stack
//  Wire.begin();
  Vset = 0x3F;
  lastPing = millis();
 
  Serial.println("test motors"); 
  delay(25);
  
  clearFault(MOTOR1);
  clearFault(MOTOR2);
  clearFault(MOTOR3);
  
  setForward(MOTOR1);
//  setForward(MOTOR2);
//  setForward(MOTOR3);
  delay(200);
  setReverse(MOTOR1);
//  setReverse(MOTOR2);
//  setReverse(MOTOR3);
  delay(200);
  setBrake(MOTOR1);
  
//  setBrake(MOTOR2);
//  setBrake(MOTOR3);
//  delay(500);
//  setForward(MOTOR1);
//  setForward(MOTOR2);
//  setForward(MOTOR3);
//  delay(500);
//  
//  setBrake(MOTOR1);
//  setBrake(MOTOR2);
//  setBrake(MOTOR3);

  delay(25);
  setForward(MOTOR2);
  delay(200);
  setReverse(MOTOR2);
  delay(200);
  setBrake(MOTOR2);
  
  delay(25);
  setForward(MOTOR3);
  delay(200);
  setReverse(MOTOR3);
  delay(200);
  setBrake(MOTOR3);
  Serial.println("ready to go!"); 
}


void loop()
{
  thisLoopMillis=millis();
  
  if(digitalRead(faultPin)==LOW){
    Serial.println(" ----FAULT----  ");
    clearFault(MOTOR1);
    clearFault(MOTOR2);
    clearFault(MOTOR3);
  }
  
 if ( (thisLoopMillis-lastPing)>timeout && timeoutPossible==1){
   Serial.print("lastPing timeout:  ");
   Serial.println(lastPing);
   Serial.print("thisLoopMillis:    ");
   Serial.println(thisLoopMillis);
      setBrake(MOTOR1);
      setBrake(MOTOR2);
      setBrake(MOTOR3);
      timeoutPossible=0; //can only timeout once
      Serial.println(" TIMED OUT ");
    }
//  clearFault(MOTOR1);
//  clearFault(MOTOR2);
//  clearFault(MOTOR3);
}

void RFduinoBLE_onReceive(char *data, int len) {
  lastPing=millis();
   Serial.print("lastPing set:    ");
   Serial.println(lastPing);
  Serial.println( "-----------RX-----------");
  // each transmission should contain an RGB triple
  clearFault(MOTOR1);
  clearFault(MOTOR3);
  clearFault(MOTOR3);
  if (len >= 3)
  {
    // get the values
    uint8_t m1 = data[0];
    uint8_t m2 = data[1];
    uint8_t m3 = data[2];
    if (m1 == 0x01){  setForward(MOTOR1); }
    else if (m1 == 0x02) {   setReverse(MOTOR1);}
    else {  setBrake(MOTOR1);  }
    
    if (m2 == 0x01){  setForward(MOTOR2); }
    else if (m2 == 0x02) {   setReverse(MOTOR2);}
    else {  setBrake(MOTOR2);  }
    
    if (m3 == 0x01){  setForward(MOTOR3); }
    else if (m3 == 0x02) {   setReverse(MOTOR3);}
    else {  setBrake(MOTOR3);  }
  }
  timeoutPossible=1; // now that we've recieved data it's possible to timeout.
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



void RFduinoBLE_onConnect() {
  setForward(MOTOR1);
  setForward(MOTOR2);
  setForward(MOTOR3);
  delay(200);
  setBrake(MOTOR1);
  setBrake(MOTOR2);
  setBrake(MOTOR3);
}

void RFduinoBLE_onDisconnect() {
  setReverse(MOTOR1);
  setReverse(MOTOR2);
  setReverse(MOTOR3);
  delay(200);
  setBrake(MOTOR1);
  setBrake(MOTOR2);
  setBrake(MOTOR3);
}


