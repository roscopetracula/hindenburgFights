#include <Wire.h>
#include <RFduinoBLE.h>
#include "rfduinoControl.h"

// General timeouts/intevals.
#define CONNECTION_TIMEOUT   1000 /* Time (ms) with no received messages before all motors are turned off. */
#define UPDATE_INTERVAL      1000 /* Interval (ms) between status updates.  Undefine to not send updates. Note that this is a lower bound. */
#define FAST_UPDATE_INTERVAL  100 /* Interval (ms) between fast status updates.  If fastUpdate is set, it will update one time at this faster interval; can be 0. */


// Igniter-related timeouts.
// Summary: After starting, the igniter will turn off IGNITER_RELEASE_TIME ms after all triggers are released, but in no case will it turn off before IGNITER_MINIMUM_TIME or stay on past IGNITER_MAXIMUM_TIME.
#define IGNITER_MINIMUM_TIME 1000 /* Minimum time (ms) after activation of trigger, before the igniter is turned off.  */
#define IGNITER_RELEASE_TIME 500  /* Time (ms) that the igniter will stay on after the trigger has been released.  Will not exceed maximum. */
#define IGNITER_MAXIMUM_TIME 5000 /* Maximum time (ms) for the igniter to be continuously on. */

// Flags
#undef  SERIAL_ENABLE            /* Enable serial communications. Watch out for loss of side effects in calls. */
#undef  DEBUG_RECEIVE_LONG       /* Define to print extended receive messages; 0 to just print '@'. */
#undef  TRANSMIT_ACK             /* Transmit an acknowledgement after each packet. */
#undef  TRANSMIT_FAULT_STRING    /* Transmit a string version of the fault in addition to the coded version. */
#undef  TRANSMIT_FAULT_IMMEDIATE /* Transmit fault messages immediately upon receiving fault. Disabled by default to reduce wireless spam (as it now comes with updates). */
#undef  TEST_MOTORS_ON_CONNECT   /* Define to do the "motor dance" when BLE connects or disconnects. */
#undef  EXPANDER                 /* new board - i2c expander */

//#define SERIAL_ENABLE
//#define DEBUG_RECEIVE_LONG
//#define EXPANDER

// Motor and other i2c addresses.
#define MOTOR1 0x63
#define MOTOR2 0x64
#define MOTOR3 0x61
#define CONTROL 0x00
#define FAULT 0x01
#define PROTOCOL_VERSION 1
#define V_SET 0x3f // 3f = 5.06, or ~20v max output.

#ifdef EXPANDER
// new board with i2c expander and LEDs
#define EXP_I2C 0x20
#define EXP_PIN_IGNITER 3
#define EXP_PIN_REDLIGHT 4
#define EXP_PIN_GREENLIGHT 5
#define EXP_PIN_FAULT 6
// expander interrupt signal to rfduino
#define EXP_INT_PIN     4
#else
// rfduino pin assignments (for board without expander)
#define FAULT_PIN   4
#define IGNITER_PIN 2
#endif

// enable battery level detection from pin
//#define BATT_PIN    2

#ifdef BATT_PIN
#define BATT_V_SCALE (2 * 3.6 / 1023.0)
#define BATT_V_CUTOFF 3.0
#undef IGNITER_PIN
#else
// if using VDD reference
#define BATT_V_SCALE (3.6 / 1023.0)
#define BATT_V_CUTOFF 2.6
#endif

//

// rfduino pin assignments.
#define SCL_PIN     6 /* 5 for slimstack board */
#define SDA_PIN     5 /* 6 for slimstack board */
#define TRIGGER_PIN 3

byte motorIndexes[3] = {MOTOR1, MOTOR2, MOTOR3};
byte curMotorStates[3][2] = {{0, 0}, {0, 0}, {0, 0}};
byte nextMotorStates[3][2] = {{0, 0}, {0, 0}, {0, 0}};
byte faultCollectors[3] = {0, 0, 0};
byte igniterStateByte = 0;
byte blimpTriggerState = 0;
byte curControllerTriggerState = 0;
byte nextControllerTriggerState = 0;
byte expectedMsgCounter = 0xff;
igniterStateEnum igniterState = IGNITER_STATE_LOCKED;
int curRSSI = 0;
unsigned long lastPingMillis = 0;
unsigned long igniterLastOn = 0;
unsigned long igniterTriggerReleased = 0;
bool timeoutPossible = 0;
bool isConnected = false;
bool fastUpdate = false;
#if defined(UPDATE_INTERVAL) || defined(FAST_UPDATE_INTERVAL)
unsigned long lastUpdateMillis = 0;
#endif

#ifdef SERIAL_ENABLE
#define DBGPRINT(...) Serial.print(__VA_ARGS__)
#define DBGPRINTLN(...) Serial.println(__VA_ARGS__)
#define DBGPRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBGPRINT(...) {}
#define DBGPRINTLN(...) {}
#define DBGPRINTF(...) {}
#endif

void setup_io() {
  // all boards: start i2c
  Wire.beginOnPins(SCL_PIN, SDA_PIN);
  delay(20);

  pinMode(TRIGGER_PIN, INPUT);
  // TODO: interrupt on trigger pin

#ifdef EXPANDER
  // with i2c io expander
  // TODO 1: set up pin modes and initial states
  // TODO 2: enable interrupt for i2c expander
  // motor fault, igniter, red LED, green LED
#else
  pinMode(FAULT_PIN, INPUT);
#ifdef IGNITER_PIN
  pinMode(IGNITER_PIN, OUTPUT);
#endif
#endif

  analogReference(VBG); // Reference 1.2V band gap
#ifdef BATT_PIN
  pinMode(BATT_PIN, INPUT);
  analogSelection(AIN_1_3_PS);  // selected analog pin input, 1/3 prescaling as the analog source
#else
  analogSelection(VDD_1_3_PS);  // VDD input, 1/3 prescaling as the analog source
#endif
}

void setup()
{
#ifdef SERIAL_ENABLE
  Serial.begin(9600);
  DBGPRINTLN("Blimp booting...");
#else
  Serial.end();
#endif

  setup_io();

  // start bluetooth
  RFduinoBLE.deviceName = "RFduino Blimp";
  RFduinoBLE.begin();
  delay(20);
  delay(20);

  // Do the motor dance.
  testMotors(0x3F, 250);
  DBGPRINTLN("ready to go!");
}

// Send data, even if it is too long for a block.
inline void bleSendData(const char *data, int len) {
  RFduinoBLE.send(data, len);
}

// Send all of a string (including null terminator in a block,
// splitting it into 19-byte segments where necessary.
void bleSendString(const char *str) {
  char sendData[20] = {RETURN_MSG_STRING};
  int len = strlen(str) + 1;
  while (len > 0) {
    if (len <= 19) {
      // Last block.
      memcpy(sendData + 1, str, len);
      bleSendData(sendData, len + 1);
      len = 0;
    } else {
      // Not last block.
      memcpy(sendData + 1, str, 19);
      bleSendData(sendData, 20);
      str += 19;
      len -= 19;
    }
  }
}

// Send a printf-style formatted string.  This is currently a bit of a hack, 
// using a fixed-size temporary buffer.  It should be easy to port asprintf; 
// we may want to do that.
void bleNPrintf(int maxLen, const char *format, ...)
{
  char buf[maxLen];
  va_list myargs;
  va_start(myargs, format);
  snprintf(buf, maxLen, format, myargs);
  va_end(myargs);  
  bleSendString(buf);
}

// Store any updates to rssi.
void RFduinoBLE_onRSSI(int rssi) {
  curRSSI = rssi;
}

boolean checkMotorFault()
{
#ifdef EXPANDER
  // TODO: read motor fault from i2c
  return 0;
#else
  return (digitalRead(FAULT_PIN) == LOW);
#endif
}

void loop()
{
  unsigned long loopMillis = millis();
  
#ifdef BATT_PIN
    float batteryVoltage = BATT_V_SCALE * analogRead(BATT_PIN);
#else
    float batteryVoltage = BATT_V_SCALE * analogRead(1); // pin doesn't matter, reading from VDD src
#endif
  if (batteryVoltage <= BATT_V_CUTOFF) {
    DBGPRINTF(" ----LOW VOLTAGE %fV----\n",batteryVoltage);
    // TODO: disable motors and igniter, set battery light to blink
  } else {
    // enable motors and igniter and normal lights
  }

  // Check for any faults from the motor controllers and clear the ones we find.
  if (checkMotorFault()) {
    DBGPRINTLN(" ----FAULT----  ");
    getFault(MOTOR1, true);
    getFault(MOTOR2, true);
    getFault(MOTOR3, true);
  }

  // Time out and shut everything down if we haven't heard from the transmitter in too long.  Note that loopMillis can actually be less than lastPingMillis, as receives are asynchronous.
  if (loopMillis - lastPingMillis > CONNECTION_TIMEOUT && loopMillis > lastPingMillis && timeoutPossible == 1) {
    DBGPRINTLN(" TIMED OUT ");
    bleSendString(" TIMED OUT ");
    timeoutPossible = 0; //can only timeout once
    initDevices();
  }

  // Process any motor updates.
  processAllMotorUpdates();

  // Check if the triggers have changed state, then check if the Igniter needs to be turned on or off.
  updateBlimpTrigger(digitalRead(TRIGGER_PIN) == HIGH ? 0x01 : 0x00);
  updateControllerTrigger(nextControllerTriggerState);
  updateIgniterState();

#if defined(UPDATE_INTERVAL) || defined(FAST_UPDATE_INTERVAL)
  // Check if it's time to send an update.
  if (loopMillis - lastUpdateMillis >= UPDATE_INTERVAL ||
      (fastUpdate && loopMillis - lastUpdateMillis >= FAST_UPDATE_INTERVAL)) {
    float curTemp = RFduino_temperature(FAHRENHEIT);
    unsigned short batteryVoltageX100 = (100.0 * batteryVoltage); // convert value to hundredths of a volt

    int bufLen = 1 + sizeof(curRSSI) + sizeof(curTemp) + 5 + sizeof(batteryVoltageX100);
    char buf[bufLen];
    buf[0] = RETURN_MSG_UPDATE;
    memcpy(buf + 1, &curRSSI, sizeof(curRSSI));
    memcpy(buf + 1 + sizeof(curRSSI), &curTemp, sizeof(curTemp));
    for (int i = 0; i < 3; i++) {
      buf[1 + sizeof(curRSSI) + sizeof(curTemp) + i] = faultCollectors[i];
      faultCollectors[i] = 0;
    }

    buf[1 + sizeof(curRSSI) + sizeof(curTemp) + 3] = igniterStateByte;
    buf[1 + sizeof(curRSSI) + sizeof(curTemp) + 4] = blimpTriggerState;
    memcpy(&buf[1 + sizeof(curRSSI) + sizeof(curTemp) + 5], &batteryVoltageX100, sizeof(batteryVoltageX100));

    bleSendData(buf, bufLen);
    lastUpdateMillis = millis();
    fastUpdate = false;
  }
#endif
}







void RFduinoBLE_onConnect() {
  isConnected = true;
  DBGPRINTLN("connected");
#ifdef TEST_MOTORS_ON_CONNECT
  testMotors(0x3F, 100);
#endif
}

void RFduinoBLE_onDisconnect() {
  isConnected = false;
  DBGPRINTLN("disconnected, turning everything off");
  resetState();
#ifdef TEST_MOTORS_ON_CONNECT
  testMotors(0x3F, 50);
#endif
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
  lastPingMillis = millis();

  if (len < 2) {
    DBGPRINTF("Malformed message, length %d < 2.\n", len);
  }

  // Receive protocol version and message counter, and adjust length and data start accordingly.
  char protoVersion = *data++;
  char msgCounter = *data++;
  len -= 2;

#ifdef DEBUG_RECEIVE_LONG
  char buf[256];
  snprintf(buf, 256, "|message received [%d]: [p:%02x] [c:%02x] [l:%d]|\n", lastPingMillis - previousPingMillis, protoVersion, msgCounter, len);
  DBGPRINT(buf);
#else
  DBGPRINT("@");
#endif

#ifdef TRANSMIT_ACK
  // Transmit debug ack message.
  char buf[256];
  int curTemp = (int)(RFduino_temperature(FAHRENHEIT) * 10.0);
  snprintf(buf, 256, "ack %02x (%d dBm) (%d.%d\u00b0F)", msgCounter, curRSSI, curTemp / 10, curTemp % 10);
  bleSendString(buf);
  char td[2] = {0x00, 'X'};
  bleSendData(td, 2);
#endif

  // If there is a protocol version mismatch, ignore all messages.
  if (protoVersion != PROTOCOL_VERSION) {
    DBGPRINTF("VERSION MISMATCH: Expected %d, received %d.\n", PROTOCOL_VERSION, protoVersion);
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
  for (int cmdStart = 0; cmdStart + 3 <= len; cmdStart += 3)
  {
    // Store motor or trigger states in the next state; the devices will be updated in the main loop.
    if (0x00 <= data[cmdStart + 0] && data[cmdStart + 0] <= 0x02) {
      nextMotorStates[data[cmdStart + 0]][0] = data[cmdStart + 1];
      nextMotorStates[data[cmdStart + 0]][1] = data[cmdStart + 2];
    } else if (data[cmdStart + 0] == 0x03) {
      nextControllerTriggerState = data[cmdStart + 1];
    } else if (data[cmdStart + 0] == 0x04) {
      RFduino_systemReset();
    } else {
      // We don't know this command.
      char buf[32];
      snprintf(buf, 32, "Unknown command number %d.\n", data[cmdStart + 0]);
      bleSendString(buf);
      DBGPRINT(buf);
    }
  }
  // now that we've recieved valid data it's possible to timeout.
  timeoutPossible = 1;
}

inline byte motorNumFromIndex(byte motorIndex) {
  for (int i = 0; i < 3; i++)
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
  for (int i = 0; i < 3; i++) {
    processMotorUpdate(i, motorIndexes[i], nextMotorStates[i][0], nextMotorStates[i][1]);
  }
}

void setIgniter(byte igniterCode) {
  // Set the igniter and then do a quick update.
#ifdef EXPANDER
#else
#ifdef IGNITER_PIN
  if (igniterCode == 0x01) {
    digitalWrite(IGNITER_PIN, HIGH);
    DBGPRINTLN("IGNITER ON");
  } else {
    digitalWrite(IGNITER_PIN, LOW);
    DBGPRINTLN("IGNITER OFF");
  }
#endif
#endif
  igniterStateByte = igniterCode;
  fastUpdate = true;
}

void updateIgniter(byte igniterCode) {
  if (igniterCode != igniterStateByte)
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
    fastUpdate = true;
  }
}

void sendMessage(byte motor, uint8_t value, char *msg) {
  Wire.beginTransmission(motor);
  Wire.write(CONTROL);
  Wire.write(value);
  byte wireAck = Wire.endTransmission();
  DBGPRINTF("motor %d %s\n", motorNumFromIndex(motor), msg);
}

void setCoast(byte thisMotor) {
  int value = (V_SET << 2) | 0x00;
  sendMessage(thisMotor, value, "coast");
}

void setForward(byte thisMotor, int speed) {
  int value = (speed << 2) | 0x01;
  sendMessage(thisMotor, value, "forward");
}

void setReverse(byte thisMotor, int speed) {
  int value = (speed << 2) | 0x02;
  sendMessage(thisMotor, value, "reverse");
}

void setBrake(byte thisMotor) {
  int value = (V_SET << 2) | 0x03;
  sendMessage(thisMotor, value, "brake");
}

void getFault(int thisMotor, bool shouldClearFault) {
  uint8_t registerFault;
  uint8_t totalRegisterFaults = 0;

  Wire.beginTransmission(thisMotor);
  Wire.write(FAULT);
  byte wireAck = Wire.endTransmission();

  Wire.requestFrom(thisMotor, 1);
  while (Wire.available()) {
    registerFault = Wire.read();
    totalRegisterFaults |= registerFault;
    if (registerFault != 0) {
      // Update the fault collectors.
      faultCollectors[motorNumFromIndex(thisMotor)] |= registerFault;

#ifdef TRANSMIT_FAULT_IMMEDIATE
      // Send a fault report to the client.
      char sendBuf[3] = {RETURN_MSG_FAULT, motorNumFromIndex(thisMotor), registerFault};
      bleSendData(sendBuf, 3);
#endif

#if defined(TRANSMIT_FAULT_STRING) || defined(SERIAL_ENABLE)
      // Build a string and send that to the client, but only if we're doing serial or trasnmitting it.
      char buf[256];
      snprintf(buf, 256, "Motor %d faults (err %d): %02x (", motorNumFromIndex(thisMotor), wireAck, registerFault);

      if (registerFault & 0x01) //fault bit
        strncat(buf, " FAULT ", 256);

      if (registerFault & 0x02) //OCP event
        strncat(buf, " OCP ", 256);

      if (registerFault & 0x04) //UVLO event
        strncat(buf, " UVLO ", 256);

      if (registerFault & 0x08) //OTS event
        strncat(buf, " OTS ", 256);

      if (registerFault & 0x10) //ILIMIT event
        strncat(buf, " ILIMIT ", 256);

      strncat(buf, ") ", 256);
#endif
      DBGPRINTLN(buf);
#ifdef TRANSMIT_FAULT_STRING
      bleSendString(buf);
#endif
    }
  }
  // If we had any faults and we are supposed to clear them, do so.
  if (shouldClearFault && totalRegisterFaults != 0)
    clearFault(thisMotor);
}

void clearFault(byte thisMotor) {
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
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 2; j++)
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
