#include <Wire.h>	
#include <RFduinoBLE.h>

// Motor addresses.
#define MOTOR1 0x63
#define MOTOR2 0x64
#define MOTOR3 0x61
#define CONTROL 0x00
#define FAULT 0x01
#define PROTOCOL_VERSION 0
#define MOTOR_TIMEOUT 1000
#define IGNITER_TIMEOUT 1000
#define V_SET 0x3f // 3f = 5.06, or ~20v max output.

// Pin assignments.
#define SCL_PIN     6 /* 5 for slimstack board */
#define SDA_PIN     5 /* 6 for slimstack board */
#define FAULT_PIN   4
#define TRIGGER_PIN 3
#define IGNITER_PIN 2

byte motorIndexes[3] = {MOTOR1,MOTOR2,MOTOR3};
byte motorStates[3][2] = {{0, 0}, {0, 0}, {0, 0}};
byte igniterState = 0;
byte blimpTriggerState = 0;
byte controllerTriggerState = 0;
byte expectedMsgCounter = 0xff;
int lastPing;
int motorMillis = 0;
int igniterMillis = 0;
bool timeoutPossible = 0;
bool isConnected = false;
uint8_t wireAck = 0;


void setup()
{
  Serial.println("Blimp booting...");
  pinMode(FAULT_PIN, INPUT);
  pinMode(IGNITER_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, INPUT);
  Wire.beginOnPins(SCL_PIN, SDA_PIN);
  delay(20);
  RFduinoBLE.deviceName = "RFduino Blimp";
  RFduinoBLE.begin();
  delay(20);
  Serial.begin(9600);
  delay(20);
  lastPing = millis();

  // Do the motor dance.
  testMotors(0x3F, 200);
  Serial.println("ready to go!"); 
}

void loop()
{
  motorMillis = millis();

  // Check for any faults from the motor controllers and clear the ones we find.
  if (digitalRead(FAULT_PIN) == LOW) {
    Serial.println(" ----FAULT----  ");
    getFault(MOTOR1, true);
    getFault(MOTOR2, true);
    getFault(MOTOR3, true);
  }
    
  // Check if the trigger has changed state, then check if the Igniter needs to be turned on or off.
  updateBlimpTrigger(digitalRead(TRIGGER_PIN) == HIGH ? 0x01 : 0x00);
  updateIgniterState();

  // Time out and shut everything down if we haven't heard from the transmitter in too long.
  if ((motorMillis - lastPing) > MOTOR_TIMEOUT && timeoutPossible == 1){
    initDevices();
    timeoutPossible = 0; //can only timeout once
    Serial.println(" TIMED OUT ");
  }
}







void RFduinoBLE_onConnect() {
  isConnected = true;
  Serial.println("connected");
  testMotors(0x3F, 100);
}

void RFduinoBLE_onDisconnect() {
  isConnected = false;
  Serial.println("disconnected, turning everything off");
  resetState();
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

  // Transmit debug ack message.  Currently disabled.
  // sprintf(buf, "ack %02x", msgCounter);
  // RFduinoBLE.send(buf, strlen(buf));
  
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
    // State changes are identified in updateControllerTrigger() and receiveMotorCommand().
    if (0x00<=data[cmdStart+0] && data[cmdStart+0]<=0x02) {
      receiveMotorCommand(data[cmdStart+0], motorIndexes[data[cmdStart+0]], data[cmdStart+1], data[cmdStart+2]);
    }
    else if (data[cmdStart+0] == 0x03) {
      updateControllerTrigger(data[cmdStart+1]);
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
    digitalWrite(IGNITER_PIN,HIGH);
    igniterMillis=millis();
    } else {
      digitalWrite(IGNITER_PIN,LOW);
    }
  igniterState = igniterCode;
  Serial.printf("set igniter %d\n", igniterCode);
}

void updateIgniterState(void) {
  // Treat the controller and trigger together.
  byte igniterTriggered = isConnected && (controllerTriggerState || blimpTriggerState);
  
  if (igniterTriggered) {
    igniterMillis = millis();
    
    if (!igniterState) {
      Serial.println("igniter turned on!");
      setIgniter(0x01);
    }
  } else if (igniterState && millis()-igniterMillis > IGNITER_TIMEOUT){
    // If the igniter has been on for long enough, turn it off.
    Serial.println("igniter timed out, turning it off.");
    setIgniter(0x00);
  } 
}

void updateControllerTrigger(byte igniterCode) {
  if (igniterCode != controllerTriggerState) {
    controllerTriggerState = igniterCode;
    Serial.printf("controller trigger state change: %d\n", igniterCode);
    updateIgniterState();
  }
}

void updateBlimpTrigger(byte igniterCode) {
  if (igniterCode != blimpTriggerState) {
    blimpTriggerState = igniterCode;
    Serial.printf("blimp trigger state change: %d\n", igniterCode);
    updateIgniterState();
  }
}

void sendMessage(byte motor, uint8_t value, char *msg) {
  Wire.beginTransmission(motor);
  Wire.write(CONTROL);
  Wire.write(value);
  wireAck = Wire.endTransmission();
  Serial.printf("set motor %d: %s\n", motor, msg);
}

void setCoast(byte thisMotor){
  int value = (V_SET << 2) | 0x00;
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
  int value = (V_SET << 2) | 0x03;
  sendMessage(thisMotor, value, "brake"); 
}

void getFault(int thisMotor, bool shouldClearFault){
  uint8_t RegisterFault;

  Wire.beginTransmission(thisMotor);
  Wire.write(FAULT);
  Serial.printf("Motor %d faults (err %d): ", thisMotor, Wire.endTransmission());

  Wire.requestFrom(thisMotor,1);
  while(Wire.available()){
    RegisterFault = Wire.read();
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

    Serial.print(") ");
  }
  Serial.println();

  if (shouldClearFault) {
    clearFault(thisMotor);
  }
}

void clearFault(byte thisMotor){
  Wire.beginTransmission(thisMotor);
  Wire.write(FAULT);
  Wire.write(0x80); // CLEAR bit clears faults
  Serial.printf("clear fault on motor %d: %d\n", thisMotor, Wire.endTransmission());
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

void resetState(void) {
  initDevices();
  for (int i=0; i<3; i++)
    for (int j=0; j<2; j++)
      motorStates[i][i] = 0;
  blimpTriggerState = 0;
  controllerTriggerState = 0;
  expectedMsgCounter = 0xff;
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
