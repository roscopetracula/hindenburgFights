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

const int Vset = 0x3F;
const int timeout = 1000;
const int motors [3] = { 
  MOTOR1, 
  MOTOR2, 
  MOTOR3 
};

int lastPing;
int thisLoopMillis=0;
bool timeoutPossible=0;
int faultPin = 4;

void setup(){
  
  Serial.begin(9600);
  Serial.println("\nDVR8830 Motor Controller");
  
  pinMode(faultPin, INPUT);
  Wire.beginOnPins(5,6); //SCL pin, SDA pin //5,6 for slimstack board v1
  delay(20);
  
  RFduinoBLE.deviceName = "RFduino Blimp"; //Sets the device name  
  //RFduinoBLE.advertisementData = "temp"; //data in the advertisement  
  RFduinoBLE.begin();  // start the BLE stack
  delay(25);
  
  lastPing = millis();
 
  Serial.println("Test motors"); 
  
  // clear faults
  runCommandOnAll('c');
  
  // startup check: forward, brake, reverse, brake
  char sequences [4] = { 'f', 'b', 'r', 'b' };
  for ( int j=2; j>=0; --j ){
    // not as efficient but more intuitive to keep it in the right order
    for ( int i=0; i<4; i++ ){
      switch ( sequences[i] ){
        case 'f': setForward(motors[j]);
          break;
        case 'r': setReverse(motors[j]);
          break;
        case 'b': setBrake(motors[j]);
          break;
      }
      // allow the motors time to spin up / down
      delay( 200 );
    }
  }

  Serial.println("Ready to go!"); 
}

void loop(){
  // get the current time
  thisLoopMillis=millis();
  //Serial.print("looping at ");
  //Serial.println(thisLoopMillis);
  
  if ( digitalRead(faultPin)==LOW ){
    Serial.println("----FAULT----");
    // Clear faults on all motors
    runCommandOnAll('c');
  }
  
  // Brake if there is no communication for > 1s (timeout)
  if ( (thisLoopMillis-lastPing)>timeout && timeoutPossible==1 ){
    Serial.println("----TIMED OUT----");
    Serial.print("    lastPing timeout: ");
    Serial.println(lastPing);
    Serial.print("    thisLoopMillis:   ");
    Serial.println(thisLoopMillis);
    
    // Brake all motors
    runCommandOnAll('b');
    
    // can only timeout once
    timeoutPossible=0;
    
    // switch to lower power mode
    //RFduino_ULPDelay(INFINITE);
  }
  
  // start checking for a timeout  
  RFduino_ULPDelay(MILLISECONDS(50));
}

void runCommandOnAll( char command ){
  for ( int j=2; j>=0; --j ){
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
          setBrake(motors[i]); break;
      }
    }
  }
  
  // now that we've recieved data it's possible to timeout in loop()
  timeoutPossible=1; 
}

void RFduinoBLE_onConnect() {
  Serial.println("RFduinoBLE_onConnect");
  
  runCommandOnAll('f');
  delay(200);
  runCommandOnAll('b');
}

void RFduinoBLE_onDisconnect() {
  Serial.println("RFduinoBLE_onDisconnect");
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

void getControl(int motor){
  uint8_t RegisterControl;
  
  Wire.beginTransmission(motor);
  Wire.write(CONTROL);
  logMotorControl("getControl", motor, Wire.endTransmission());
  
  Wire.requestFrom(motor,1);
  while(Wire.available()){
    RegisterControl = Wire.read();
    Serial.print("RegisterControl: ");
    Serial.println(RegisterControl, HEX);
  }
}

void setControl(byte motor, uint8_t control){
  doMotorControl("setControl", motor, CONTROL, control);
}
  
void clearFault(byte motor){
  doMotorControl("clearFault", motor, FAULT, 0x80); // CLEAR bit clears faults
}
void setCoast(byte motor){
  doMotorControl("setCoast", motor, CONTROL, 0x00);
}

void setForward(byte motor){
  doMotorControl("setForward", motor, CONTROL, 0x01);
}

void setReverse(byte motor){
  doMotorControl("setReverse", motor, CONTROL, 0x02);
}

void setBrake(byte motor){
  doMotorControl("setBrake", motor, CONTROL, 0x03);
}

void doMotorControl(String action, byte motor, byte type, uint8_t arg){
  Wire.beginTransmission(motor);
  Wire.write(type); 
  Wire.write((Vset << 2) | arg); 
  logMotorControl(action, motor, Wire.endTransmission());
}  

void logMotorControl(String action, byte motor, int wireStatus){
  switch(motor){
    case 97: Serial.print("Back "); break;
    case 100: Serial.print("Front "); break;
    case 103: Serial.print("Bottom "); break;
  }
  Serial.print(action);
  Serial.print('(');
  Serial.print(motor);
  Serial.print(") wire status: ");
  Serial.println(wireStatus);
}
