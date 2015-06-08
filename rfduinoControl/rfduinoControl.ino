#include <Wire.h>	
#include <RFduinoBLE.h>
#include "rfduinoControl.h"

// Tuning and timeouts.
#define CONNECTION_TIMEOUT 1000 /* Time (ms) with no received messages before all motors are turned off. */
#define SERIAL_ENABLE      1    /* 0 to disable all serial communications. Watch out for loss of side effects in DBGPRINT* calls. */
#define DEBUG_RECEIVE_LONG 0    /* 1 to print extended receive messages; 0 to just print '@'. */

// Summary: After starting, the igniter will turn off IGNITER_RELEASE_TIME ms after all triggers are released, but in no case will it turn off before IGNITER_MINIMUM_TIME or stay on past IGNITER_MAXIMUM_TIME.
#define IGNITER_MINIMUM_TIME 1000 /* Time (ms) after release of trigger before the igniter is turned off.  */
#define IGNITER_RELEASE_TIME 500  /* Time (ms) that the igniter will stay on after the button has been released.  Will not exceed maximum. */
#define IGNITER_MAXIMUM_TIME 5000 /* Maximum time (ms) for the igniter to be continuously on. */ 
#define BLESEND RFduinoBLE.send

// Motor and other i2c addresses.
#define MOTOR1 0x63
#define MOTOR2 0x64
#define MOTOR3 0x61
#define CONTROL 0x00
#define FAULT 0x01
#define PROTOCOL_VERSION 0
#define V_SET 0x3f // 3f = 5.06, or ~20v max output.

// Pin assignments.
#define SCL_PIN     6 /* 5 for slimstack board */
#define SDA_PIN     5 /* 6 for slimstack board */
#define FAULT_PIN   4
#define TRIGGER_PIN 3
#define IGNITER_PIN 2

byte motorIndexes[3] = {MOTOR1,MOTOR2,MOTOR3};
byte curMotorStates[3][2] = {{0, 0}, {0, 0}, {0, 0}};
byte nextMotorStates[3][2] = {{0, 0}, {0, 0}, {0, 0}};
byte igniterStateByte = 0;
byte blimpTriggerState = 0;
byte curControllerTriggerState = 0;
byte nextControllerTriggerState = 0;
byte expectedMsgCounter = 0xff;
igniterStateEnum igniterState = IGNITER_STATE_LOCKED;
unsigned long lastPingMillis = 0;
unsigned long motorMillis = 0;
unsigned long igniterLastOn = 0;
unsigned long igniterTriggerReleased = 0;
bool timeoutPossible = 0;
bool isConnected = false;

#if SERIAL_ENABLE
#define DBGPRINT(...) Serial.print(__VA_ARGS__)
#define DBGPRINTLN(...) Serial.println(__VA_ARGS__)
#define DBGPRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBGPRINT(...) {}
#define DBGPRINTLN(...) {}
#define DBGPRINTF(...) {}
#endif

void setup()
{
#if SERIAL_ENABLE
  Serial.begin(9600);
  DBGPRINTLN("Blimp booting...");
#else
  Serial.end();
#endif
  pinMode(FAULT_PIN, INPUT);
  pinMode(IGNITER_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, INPUT);
  Wire.beginOnPins(SCL_PIN, SDA_PIN);
  delay(20);
  RFduinoBLE.deviceName = "RFduino Blimp";
  RFduinoBLE.begin();
  delay(20);
  delay(20);

  // Do the motor dance.
  testMotors(0x3F, 200);
  DBGPRINTLN("ready to go!"); 
}

void loop()
{
  motorMillis = millis();
  
  // Check for any faults from the motor controllers and clear the ones we find.
  if (digitalRead(FAULT_PIN) == LOW) {
    DBGPRINTLN(" ----FAULT----  ");
    getFault(MOTOR1, true);
    getFault(MOTOR2, true);
    getFault(MOTOR3, true);
  }

  // Time out and shut everything down if we haven't heard from the transmitter in too long.  Note that motorMillis can actually be less than lastPingMillis, as receives are asynchronous. 
  if (motorMillis - lastPingMillis > CONNECTION_TIMEOUT && motorMillis > lastPingMillis && timeoutPossible == 1) {
    DBGPRINTLN(" TIMED OUT ");
    timeoutPossible = 0; //can only timeout once
    initDevices();
  }

  // Process any motor updates.
  processAllMotorUpdates();
  
  // Check if the triggers have changed state, then check if the Igniter needs to be turned on or off.
  updateBlimpTrigger(digitalRead(TRIGGER_PIN) == HIGH ? 0x01 : 0x00);
  updateControllerTrigger(nextControllerTriggerState);
  updateIgniterState();
}







void RFduinoBLE_onConnect() {
  isConnected = true;
  DBGPRINTLN("connected");
  testMotors(0x3F, 100);
}

void RFduinoBLE_onDisconnect() {
  isConnected = false;
  DBGPRINTLN("disconnected, turning everything off");
  resetState();
  testMotors(0x3F, 50);
}

void sendDouble(char *inBuf, int len) {
  char outBuf[256];
  for (int i=0; i<128 && i < len; i++) {
    outBuf[i*2] = inBuf[i];
    outBuf[i*2+1] = inBuf[i];
  }
  RFduinoBLE.send(outBuf, len*2);
}

/*
radio messages should be two hearder bytes [protoVersion, msgCount]
followed by multiples of 3 bytes: ([channelNum,msg1,msg2] * n)
how msg1 ang msg2 bytes will be understood depends on the channel.
for motor channels (00-02) it will be [motorNum,motorDirectionCode,motorSpeed]
for igniter channel (03) it will be [channelNum,duration1,duration2]
*/
void RFduinoBLE_onReceive(char *data, int len) {
  unsigned long previousPingMillis = lastPingMillis;
  lastPingMillis=millis();
  
  if (len < 2) {
    DBGPRINTF("Malformed message, length %d < 2.\n", len);
  }

  // Receive protocol version and message counter, and adjust length and data start accordingly.
  char protoVersion = *data++;
  char msgCounter = *data++;
  len -= 2;
  
#if DEBUG_RECEIVE_LONG
  char buf[256];
  sprintf(buf, "|message received [%d]: [p:%02x] [c:%02x] [l:%d]|\n", lastPingMillis - previousPingMillis, protoVersion, msgCounter, len);
  DBGPRINT(buf);
#else
  DBGPRINT("@");
#endif

  // Transmit debug ack message.  Currently disabled.
  char buf[256];
  sprintf(buf, "ack %02x", msgCounter);
  BLESEND(buf, strlen(buf));
  
  // If there is a protocol version mismatch, ignore all messages.
  if (protoVersion != PROTOCOL_VERSION) {
    DBGPRINTF("VERSION MISMATCH: Expected %d, received %d.\n", 0, PROTOCOL_VERSION);
    return;
  }

  // Check for any lost messages.
  if (msgCounter == 0xff) {
    // 0xff means a new connection, and we start at 0 from there.
    DBGPRINTLN("New connection received.");
    expectedMsgCounter = 0x00;
  } else if (expectedMsgCounter == 0xff) {
    // If we received an ongoing counter but haven't seen a start message, just note it.
    DBGPRINTF("Unexpected counter %d with no start message.\n", msgCounter);
    expectedMsgCounter = (msgCounter + 1) % 255;
  } else if (msgCounter == expectedMsgCounter) {
    // All is proceeding as normal.
    expectedMsgCounter = (msgCounter + 1) % 255;
  } else {
    // We lost packets.
    char lostMessages = msgCounter - expectedMsgCounter;
    if (lostMessages < 0)
      lostMessages += 255;      
    DBGPRINTF("LOST %d MESSAGES. (Expected counter %d, received %d.)\n", 
      lostMessages, expectedMsgCounter, msgCounter);
    expectedMsgCounter = (msgCounter + 1) % 255;
  }
  
  // We are assuming that any full block of three bytes can be interpreted, 
  // and any remainder is ignored.
  for (int cmdStart=0; cmdStart+3 <= len; cmdStart+=3)
  {
    // Store motor or trigger states in the next state; the devices will be updated in the main loop.      
    if (0x00<=data[cmdStart+0] && data[cmdStart+0]<=0x02) {
      nextMotorStates[data[cmdStart+0]][0] = data[cmdStart+1];
      nextMotorStates[data[cmdStart+0]][1] = data[cmdStart+2];
    }
    else if (data[cmdStart+0] == 0x03) {
      nextControllerTriggerState = data[cmdStart+1];
    }
  }
  // now that we've recieved valid data it's possible to timeout.
  timeoutPossible = 1;
}

inline byte motorNumFromIndex(byte motorIndex) {
  for (int i=0; i<3; i++)
    if (motorIndexes[i] == motorIndex)
      return i;
  return -1;
}

void processMotorUpdate(byte motorNum, byte motorIndex, uint8_t value, uint8_t speed) {
  
  // If the value has not changed, simply return.
  if (curMotorStates[motorNum][0] == value &&
      curMotorStates[motorNum][1] == speed)
      return;
      
  // Otherwise, update the motor setting and store the new values.
  if (value == 0x01) {
    setForward(motorIndex, speed);
  } else if (value == 0x02) {
    setReverse(motorIndex, speed);
  } else {
    setBrake(motorIndex);
  }
  curMotorStates[motorNum][0] = value;
  curMotorStates[motorNum][1] = speed;
}

// Try to update all motor states.  State differencing is done in processMotorUpdate.
void processAllMotorUpdates(void) {
  for (int i=0; i<3; i++) {
    processMotorUpdate(i, motorIndexes[i], nextMotorStates[i][0], nextMotorStates[i][1]);
  }
}

void setIgniter(byte igniterCode) {
  if (igniterCode==0x01){
      digitalWrite(IGNITER_PIN,HIGH);
      DBGPRINTLN("IGNITER ON");
    } else {
      digitalWrite(IGNITER_PIN,LOW);
      DBGPRINTLN("IGNITER OFF");
    }
  igniterStateByte = igniterCode;
}

void updateIgniter(byte igniterCode) {
  if (igniterCode == igniterStateByte)
    return;
  else
    setIgniter(igniterCode);
}

void updateIgniterState(void) {
  // Treat the controller and trigger as a single combined trigger.
  bool igniterTriggered = curControllerTriggerState || blimpTriggerState;
  unsigned int curMillis = millis();
  
  if (!isConnected && igniterState != IGNITER_STATE_LOCKED) {
    DBGPRINTLN("we're disconnected, locking the igniter");
    igniterState = IGNITER_STATE_LOCKED;
  }  
 
  switch (igniterState) {
    case IGNITER_STATE_LOCKED:
      if (isConnected) {
        // We're connected, unlock the igniter.
        igniterState = IGNITER_STATE_OFF;
      } else if (igniterState) {
        // We're locked but the igniter is on, turn it off.
        DBGPRINTLN("igniter is locked but on, turning it off");
        setIgniter(0x00);
      }
      break;
    case IGNITER_STATE_OFF:
      if (igniterTriggered) {
        DBGPRINTLN("igniter triggered, turning it on");
        setIgniter(0x01);     
        igniterLastOn = curMillis;
        igniterState = IGNITER_STATE_ON;
      }
      break;
    case IGNITER_STATE_ON:
      if (igniterTriggered) {
        // THE TRIGGER IS ON. If it's been on too long, turn it off.
        if (curMillis > igniterLastOn + IGNITER_MAXIMUM_TIME) {
          DBGPRINTF("igniter has been on too long [%d ms on], moving to cooldown\n", curMillis - igniterLastOn);
          setIgniter(0x00);
          igniterState = IGNITER_STATE_COOLDOWN;
        }
        // Otherwise, just leave it on.
      } else {
        // THE TRIGGER IS OFF. Move to the turning off state.
        DBGPRINTF("trigger released, starting turn-off process [%d ms on]\n", curMillis - igniterLastOn);
        igniterState = IGNITER_STATE_TURNING_OFF;
        igniterTriggerReleased = curMillis;
      }
      break;
    case IGNITER_STATE_TURNING_OFF:     
      if (igniterTriggered) {
        // If we're re-triggerd, just go back to the ON state and continue as normal.
        DBGPRINTLN("igniter re-triggered, returning to ON state");
        igniterState = IGNITER_STATE_ON;
      } else if (curMillis > igniterLastOn + IGNITER_MAXIMUM_TIME) {
        // If we're past the maximum time, jump right to off.
        DBGPRINTF("igniter has been on too long [%d ms on] and trigger is released, turning off\n", curMillis - igniterLastOn);
        igniterState = IGNITER_STATE_OFF;
        setIgniter(0x00);
      } else if (curMillis > igniterLastOn + IGNITER_MINIMUM_TIME && 
                 curMillis > igniterTriggerReleased + IGNITER_RELEASE_TIME) {
         // If we're past the minimum on time and the release time, turn the igniter off.
         DBGPRINTF("trigger released for sufficient time [%d ms on], turning off\n", curMillis - igniterLastOn);
         setIgniter(0x00);
         igniterState = IGNITER_STATE_OFF;
      }
      break;
    case IGNITER_STATE_COOLDOWN:
      if (!igniterTriggered) {
        // Trigger has been released.  Just transition to off state.
        DBGPRINTLN("trigger released, moving from cooldown to off");
        igniterState = IGNITER_STATE_OFF;
      }
      break;
    default:
      DBGPRINTF("*** UNKNOWN IGNITER STATE: %d\n", igniterState);
      break;
  }
}

void updateControllerTrigger(byte triggerCode) {
  if (triggerCode != curControllerTriggerState) {
    curControllerTriggerState = triggerCode;
    DBGPRINTF("controller trigger state change: %d\n", triggerCode);
  }
}

void updateBlimpTrigger(byte triggerCode) {
  if (triggerCode != blimpTriggerState) {
    blimpTriggerState = triggerCode;
    DBGPRINTF("blimp trigger state change: %d%s\n", triggerCode, (triggerCode == 1 && igniterState == IGNITER_STATE_LOCKED) ? " (but IGNITER_STATE_LOCKED)" : "");
  }
}

void sendMessage(byte motor, uint8_t value, char *msg) {
  Wire.beginTransmission(motor);
  Wire.write(CONTROL);
  Wire.write(value);
  byte wireAck = Wire.endTransmission();
  DBGPRINTF("motor %d %s\n", motorNumFromIndex(motor), msg);
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

void getFault(int thisMotor, bool shouldClearFault) {
  uint8_t registerFault;
  uint8_t totalRegisterFaults = 0;
  
  Wire.beginTransmission(thisMotor);
  Wire.write(FAULT);
  byte wireAck = Wire.endTransmission();

  Wire.requestFrom(thisMotor,1);
  while(Wire.available()) {
    registerFault = Wire.read();
    totalRegisterFaults |= registerFault;
    if (registerFault != 0) {
      DBGPRINTF("Motor %d faults (err %d): ", motorNumFromIndex(thisMotor), wireAck);
      DBGPRINT(registerFault, HEX);
      DBGPRINT(" (");

      if(registerFault & 0x01) //fault bit
        DBGPRINT(" FAULT ");

      if(registerFault & 0x02) //OCP event
        DBGPRINT(" OCP ");

      if(registerFault & 0x04) //UVLO event
        DBGPRINT(" UVLO ");

      if(registerFault & 0x08) //OTS event
        DBGPRINT(" OTS ");

      if(registerFault & 0x10) //ILIMIT event
        DBGPRINT(" ILIMIT ");

      DBGPRINTLN(") ");
    }
  }
  // If we had any faults and we are supposed to clear them, do so.
  if (shouldClearFault && totalRegisterFaults != 0)
    clearFault(thisMotor);
}

void clearFault(byte thisMotor){
  Wire.beginTransmission(thisMotor);
  Wire.write(FAULT);
  Wire.write(0x80); // CLEAR bit clears faults
  byte wireAck = Wire.endTransmission();
  DBGPRINTF("clear fault on motor %d: %d\n", motorNumFromIndex(thisMotor), wireAck);
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
      curMotorStates[i][i] = 0;
  blimpTriggerState = 0;
  curControllerTriggerState = 0;
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
