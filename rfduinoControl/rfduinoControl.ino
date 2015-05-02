#include <Wire.h>	
#include <RFduinoBLE.h>

#define Vref = 1.285
#define MOTOR1 0x63
#define MOTOR2 0x64
#define MOTOR3 0x61
#define CONTROL 0x00
#define FAULT 0x01
#define PROTOCOL_VERSION 0

byte motorIndexes[3] = {MOTOR1,MOTOR2,MOTOR3};
byte motorStates[3][2] = {{0, 0}, {0, 0}, {0, 0}};
byte igniterState = 0;
byte expectedMsgCounter = 0xff;
int Vset;
int motorTimeout = 1000;
int igniterTimeout = 1000;
int lastPing;
int motorMillis = 0;
int igniterMillis = 0;
bool timeoutPossible = 0;

int sclPin = 6; // 5 for slimstack board
int sdaPin = 5; // 6 for slimstack board
int faultPin = 4;
int igniterPin = 3;
uint8_t wireAck;


void setup()
{
  Serial.println("Blimp booting...");
  pinMode(faultPin, INPUT);
  pinMode(igniterPin, OUTPUT);
  Wire.beginOnPins(sclPin, sdaPin);
  delay(20);
  RFduinoBLE.deviceName = "RFduino Blimp";
  RFduinoBLE.begin();
  delay(20);
  Serial.begin(9600);
  delay(20);
  Vset = 0x3F;
  lastPing = millis();

  // Do the motor dance.
  testMotors(0x3F, 200);
  Serial.println("ready to go!"); 
}

void loop()
{
  motorMillis = millis();

  if (digitalRead(faultPin) == LOW) {
    Serial.println(" ----FAULT----  ");
    getFault(MOTOR1);
    getFault(MOTOR2);
    getFault(MOTOR3);
    clearAllFaults();
  }

  if ((motorMillis - lastPing) > motorTimeout && timeoutPossible == 1){
    initDevices();
    timeoutPossible = 0; //can only timeout once
    Serial.println(" TIMED OUT ");
  }

  if (millis()-igniterMillis > igniterTimeout){
    Serial.println("Igniter timed out, turning it off.");
    igniterMillis = millis();
    updateIgniter(0x00);
  }
}







void RFduinoBLE_onConnect() {
  Serial.println("connected");
  testMotors(0x3F, 100);
}

void RFduinoBLE_onDisconnect() {
  Serial.println("disconnected");
  testMotors(0x3F, 50);
}



/*
radio messages should be two hearder bytes [protoVersion, msgCount]
followed by multiples of 3 bytes: ([channelNum,msg1,msg2] * n)
how msg1 ang msg2 bytes will be understood depends on the channel.
for motor channels (00-02) it will be [motorNum,motorDirectionCode,motorSpeed]
for igniter channel (03) it will be [channelNum,duration1,duration2]
*/
void RFduinoBLE_onReceive(char *data, int len) {
  lastPing=millis();
  
  // Receive protocol version and message counter, and adjust length and data start accordingly.
  char protoVersion = *data++;
  char msgCounter = *data++;
  len -= 2;
  
  char buf[256];
  sprintf(buf, "message received: [p:%02x] [c:%02x] [l:%d]\n", protoVersion, msgCounter, len);
  Serial.print(buf);

  // If there is a protocol version mismatch, ignore all messages.
  if (protoVersion != PROTOCOL_VERSION) {
    Serial.printf("VERSION MISMATCH: Expected %d, received %d.\n", 0, PROTOCOL_VERSION);
    return;
  }

  // Check for any lost messages.
  if (msgCounter == 0xff) {
    // 0xff means a new connection, and we start at 0 from there.
    Serial.println("New connection received.");
    expectedMsgCounter = 0x00;
  } else if (expectedMsgCounter == 0xff) {
    // If we received an ongoing counter but haven't seen a start message, just note it.
    Serial.printf("Unexpected counter %d with no start message.\n", msgCounter);
    expectedMsgCounter = (msgCounter + 1) % 255;
  } else if (msgCounter == expectedMsgCounter) {
    // All is proceeding as normal.
    expectedMsgCounter = (msgCounter + 1) % 255;
  } else {
    // We lost packets.
    char lostMessages = msgCounter - expectedMsgCounter;
    if (lostMessages < 0)
      lostMessages += 255;      
    Serial.printf("LOST %d MESSAGES. (Expected counter %d, received %d.)\n", 
      lostMessages, expectedMsgCounter, msgCounter);
    expectedMsgCounter = (msgCounter + 1) % 255;
  }
  
  // We are assuming that any full block of three bytes can be interpreted, 
  // and any remainder is ignored.
  for (int cmdStart=0; cmdStart+3 <= len; cmdStart+=3)
  {
    // If it's a motor command, parse it.  
    // If it's an igniter command and the state has changed, update the igniter.
    // State changes are identified in updateIgniter() and receiveMotorCommand().
    if (0x00<=data[cmdStart+0] && data[cmdStart+0]<=0x02) {
      receiveMotorCommand(data[cmdStart+0], motorIndexes[data[cmdStart+0]], data[cmdStart+1], data[cmdStart+2]);
    }
    else if (data[cmdStart+0] == 0x03) {
      updateIgniter(data[cmdStart+1]);
    }
  }
  // now that we've recieved valid data it's possible to timeout.
  timeoutPossible = 1;
}


void receiveMotorCommand(byte motorNum, byte motorIndex, uint8_t value, uint8_t speed) {
  
  // If the value has not changed, simply return.
  if (motorStates[motorNum][0] == value &&
      motorStates[motorNum][1] == speed)
      return;
      
  // Otherwise, update the motor setting and store the new values.
  if (value == 0x01) {
    setForward(motorIndex, speed);
  } else if (value == 0x02) {
    setReverse(motorIndex, speed);
  } else {
    setBrake(motorIndex);
  }
  motorStates[motorNum][0] = value;
  motorStates[motorNum][1] = speed;
}

void setIgniter(byte igniterCode) {
  if (igniterCode==0x01){
    digitalWrite(igniterPin,HIGH);
    igniterMillis=millis();
    } else {
      digitalWrite(igniterPin,LOW);
    }
  igniterState = igniterCode;
  Serial.printf("set igniter %d\n", igniterCode);
}

void updateIgniter(byte igniterCode) {
  if (igniterCode != igniterState)
    setIgniter(igniterState);
}
void sendMessage(byte motor, uint8_t value, char *msg) {
  Wire.beginTransmission(motor);
  Wire.write(CONTROL);
  Wire.write(value);
  wireAck = Wire.endTransmission();
  Serial.printf("set motor %d: %s\n", motor, msg);
}

void setCoast(byte thisMotor){
  int value = (Vset << 2) | 0x00;
  sendMessage(thisMotor, value, "coast"); 
}

void setForward(byte thisMotor, int speed){
  int value = (speed << 2) | 0x01;
  sendMessage(thisMotor, value, "forward"); 
}

void setReverse(byte thisMotor, int speed) {
  int value = (speed << 2) | 0x02;
  sendMessage(thisMotor, value, "reverse"); 
}

void setBrake(byte thisMotor){
  int value = (Vset << 2) | 0x03;
  sendMessage(thisMotor, value, "brake"); 
}

void getFault(int thisMotor){
  uint8_t RegisterFault;

  Wire.beginTransmission(thisMotor);
  Wire.write(FAULT);
  Serial.printf("Result of fault request for motor %d: ", thisMotor);
  Serial.println(Wire.endTransmission());

  Wire.requestFrom(thisMotor,1);
  while(Wire.available()){
    RegisterFault = Wire.read();
    Serial.print("RegisterFault: ");
    Serial.print(RegisterFault, HEX);
    Serial.print(" (");

    if(RegisterFault & 0x01) //fault bit
      Serial.print(" FAULT ");

    if(RegisterFault & 0x02) //OCP event
      Serial.print(" OCP ");

    if(RegisterFault & 0x04) //UVLO event
      Serial.print(" UVLO ");

    if(RegisterFault & 0x08) //OTS event
      Serial.print(" OTS ");

    if(RegisterFault & 0x10) //ILIMIT event
      Serial.print(" ILIMIT ");

    Serial.println(")");
  }
}

void clearFault(byte thisMotor){
  Wire.beginTransmission(thisMotor);
  Wire.write(FAULT);
  Wire.write(0x80); // CLEAR bit clears faults
  Serial.print("clear fault ");
  Serial.println(Wire.endTransmission());
}

void clearAllFaults() {
  clearFault(MOTOR1);
  clearFault(MOTOR2);
  clearFault(MOTOR3);
}

void initDevices(void) {
  // Make sure motors and igniter are off.
  setIgniter(0);
  setBrake(MOTOR1);
  setBrake(MOTOR2);
  setBrake(MOTOR3);
}

void testMotors(uint8_t velocity, int interval)
{
  initDevices();
  
  delay(25);
  clearAllFaults();
  delay(25);

  setForward(MOTOR1, velocity);
  delay(interval);
  setBrake(MOTOR1);
  delay(interval);

  setForward(MOTOR2, velocity);
  delay(interval);
  setBrake(MOTOR2);
  delay(interval);

  setForward(MOTOR3, velocity);
  delay(interval);
  setBrake(MOTOR3);
  delay(interval);
}
