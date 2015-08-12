#include <Wire.h>
#include <RFduinoBLE.h>
#include "rfduinoControl.h"

// General timeouts/intevals.
#define CONNECTION_TIMEOUT      1000 /* Time (ms) with no received messages before all motors are turned off. */
#define UPDATE_INTERVAL_DEFAULT 1000 /* Interval (ms) between status updates.  Undefine to not send updates. Note that this is a lower bound. */
#define UPDATE_INTERVAL_FAST     100 /* Interval (ms) between fast status updates.  If fastUpdate is set, it will update one time at this faster interval; can be 0. */
#define UPDATE_INTERVAL_IMMEDIATE  0 /* Interval to use to immediately update the client. */

// Igniter-related timeouts.
// Summary: After starting, the igniter will turn off IGNITER_RELEASE_TIME ms after all triggers are released, but in no case will it turn off before IGNITER_MINIMUM_TIME or stay on past IGNITER_MAXIMUM_TIME.
#define IGNITER_MINIMUM_TIME 2500 /* Minimum time (ms) after activation of trigger, before the igniter is turned off.  */
#define IGNITER_RELEASE_TIME 500  /* Time (ms) that the igniter will stay on after the trigger has been released.  Will not exceed maximum. */
#define IGNITER_MAXIMUM_TIME 5000 /* Maximum time (ms) for the igniter to be continuously on. */

// Flags (#define to enable or #undef to disable)
#define SERIAL_ENABLE            /* Enable serial communications. Watch out for loss of side effects in calls. */
#undef  DEBUG_RECEIVE_LONG       /* Define to print extended receive messages; 0 to just print '@'. */
#undef  TRANSMIT_ACK             /* Transmit an acknowledgement after each packet. */
#undef  TRANSMIT_FAULT_STRING    /* Transmit a string version of the fault in addition to the coded version. */
#undef  TRANSMIT_FAULT_IMMEDIATE /* Transmit fault messages immediately upon receiving fault. Disabled by default to reduce wireless spam (as it now comes with updates). */
#undef  TEST_MOTORS_ON_CONNECT   /* Define to do the "motor dance" when BLE connects or disconnects. */
#undef  DEBUG_VOLTAGE_READING    /* Send debug messages every time we read the battery voltage. */
#undef  DEBUG_I2C_EXP            /* Debug messages to/from i2c expander. */
#define DEBUG_TRIGGER_INTERRUPT  /* Print a "!" when a trigger interrupt happens. */
#define REMOTE_IGNITER           /* Trigger igniter from controller signal */
#define ENABLE_MOTOR_FAULT_MODE  /* If a motor faults, perform a cool-down sequence */

// Motor and othefgr i2c addresses.
#define MOTOR1 0x63
#define MOTOR2 0x64
#define MOTOR3 0x61
#define CONTROL 0x00
#define FAULT 0x01
#define PROTOCOL_VERSION 1
#define V_SET 0x3f // 3f = 5.06, or ~20v max output.

#define MOTOR_FAULT_FAULT 0x01
#define MOTOR_FAULT_OCP 0x02
#define MOTOR_FAULT_UVLO 0x04
#define MOTOR_FAULT_OTS 0x08
#define MOTOR_FAULT_ILIMIT 0x10

#define MOTOR_FAULT_MODE_MASK (MOTOR_FAULT_FAULT | MOTOR_FAULT_OCP | MOTOR_FAULT_UVLO | MOTOR_FAULT_OTS | MOTOR_FAULT_ILIMIT ) /* motor fault bits that will trigger motor fault mode */
#define MOTOR_FAULT_MODE_RESET_MILLIS 250 /* minimum elapsed time until motor fault mode ends */

// i2c expander
#define EXP_I2C_ADR 0x20
#define EXP_REGISTER_INPUT 0x00
#define EXP_REGISTER_OUTPUT 0x01
#define EXP_REGISTER_CONFIG 0x03
#define EXP_PIN_IGNITER 3
#define EXP_PIN_LED_RED 5
#define EXP_PIN_LED_GREEN 4
#define EXP_PIN_FAULT 6
#define IF_EXPANDER_ELSE(a,b) (expanderPresent ? (a) : (b))

// rfduino pin assignment for boards without i/o expander
#define PRE_EXP_FAULT_PIN 4
#define PRE_EXP_TRIGGER_PIN 3
#define PRE_EXP_IGNITER_PIN 2

// rfduino pin assignments for boards with i/o expander
#define BATT_PIN 4
#define INT_PIN 3

/*
// Macros for manually calculating divider split.
#define VDIVIDER_TOP 10000.0
#define VDIVIDER_BOTTOM 10000.0 // Theoretical 50/50 divider with 10k resistors.
#define VDIVIDER_BOTTOM 7142.857 // Theoretical accounting for 25k internal and 10k external parallel resistors to ground.
#define REAL_BATTERY_V_SCALE ((VDIVIDER_TOP+VDIVIDER_BOTTOM)/VDIVIDER_BOTTOM * 3.6 / 1023.0)
*/
#define REAL_BATTERY_V_SCALE (2.0 * 3.6 / 1023.0) // if using the battery VCC
#define VDD_V_SCALE (3.6 / 1023.0) // if using VDD reference
#define BATTERY_CUTOFF 3.0
#define BATTERY_RECOVERED_MIN_V 3.67 // low voltage state will automatically reset if battery voltage is at least this high
#define BATTERY_CUTOFF_MILLIS 50 // must stay low for >0 milliseconds - not just a single read
#define BATTERY_FAKE_VOLTAGE 4.0 // if voltage is being ignored

// rfduino pin assignments.
#define SCL_PIN     6 /* 5 for slimstack board */
#define SDA_PIN     5 /* 6 for slimstack board */
#define TRIGGER_PIN IF_EXPANDER_ELSE(2,PRE_EXP_TRIGGER_PIN)

// Controller trigger bits
#define CTRL_BIT_LEFT_TRIGGER    0x04
#define CTRL_BIT_RIGHT_TRIGGER   0x20
#define CTRL_BIT_IGNITER         0x08
#define CTRL_BIT_SUPRESS_IGNITER 0x01

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
bool triggerInterruptCalled = false;
bool timeoutPossible = 0;
bool isConnected = false;
bool voltageIsLow = false;
#define TIME_NEVER 0xffff
unsigned long voltageLowStartTime = TIME_NEVER; /* the time at which the battery voltage went below the threshold, or NEVER if it is over the threshold. */
bool ignoreBatteryVoltage = false; /* Ignore the battery voltage; useful for testing on USB power, which always reads as low. */
bool overrideBatteryVoltage = false; /* Flag set asynchronously. */
unsigned long motorFaultModeStart[3] = {TIME_NEVER, TIME_NEVER, TIME_NEVER}; /* the time at which the fault cool-off state started for a certain motor, or TIME_NEVER for normal operation */
bool expanderPresent = 0; // auto detect
uint8_t expanderOutput = 0; // default state = all output pins low
unsigned int nextUpdateTime = 0; // Send the first update right away. 
unsigned long lastUpdateMillis = 0;

// blinky
#define LED_ON 0
#define LED_OFF 1
#define LED_ON_MILLIS 15
#define LED_OFF_MILLIS 1000
#define LED_OFF_SHORT_MILLIS 150
ledStates ledState = LEDSTATE_START;
unsigned long ledStateLastChange = 0;

#ifdef SERIAL_ENABLE
#define DBGPRINT(...) Serial.print(__VA_ARGS__)
#define DBGPRINTLN(...) Serial.println(__VA_ARGS__)
#define DBGPRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBGPRINT(...) {}
#define DBGPRINTLN(...) {}
#define DBGPRINTF(...) {}
#endif


// reference http://www.ti.com/lit/ds/symlink/tca6408a.pdf

byte writeExpanderRegister(uint8_t expRegister, uint8_t value) {
  Wire.beginTransmission(EXP_I2C_ADR);
  Wire.write(expRegister);
  Wire.write(value);
  byte wireAck = Wire.endTransmission();
#ifdef DEBUG_I2C_EXP
  DBGPRINTF("i2c_exp write %x %x %d\n", (int)expRegister,(int)value,(int)wireAck);
#endif
  return wireAck;
}

uint8_t readExpanderRegister(uint8_t expRegister) {
  int result = 0;
  Wire.beginTransmission(EXP_I2C_ADR);
  Wire.write(expRegister);
  byte wireAck = Wire.endTransmission();
  Wire.requestFrom(EXP_I2C_ADR, 1, true);
  if (Wire.available()) {
    result = Wire.read();
#ifdef DEBUG_I2C_EXP
    DBGPRINTF("i2c_exp read %x from %d\n", (int) result, (int)expRegister);
#endif
  } else {
#ifdef DEBUG_I2C_EXP
    DBGPRINTF("i2c_exp failed to read from %d\n", (int)expRegister);
#endif
  }
  return result;
}

// Make the next update faster.  If the value received is actually slower than the next scheduled update, don't change anything.
void fastUpdate(int newUpdateTime = UPDATE_INTERVAL_FAST) {
  if (newUpdateTime < nextUpdateTime)
    nextUpdateTime = newUpdateTime;
}

void configureExpander() {
  uint8_t configuration=0;
  configuration |= (1<<EXP_PIN_FAULT);  // fault pin is input

  // igniter and led1 and led2 are output
  // looks like unused pins float. configured these as outputs to avoid spurious interrupts.
  if(writeExpanderRegister(EXP_REGISTER_CONFIG,configuration) == 0) {
    expanderPresent = 1;

    // set outputs to default value (all low. right? this probably turns both LEDs ON and igniter OFF)
    writeExpanderRegister(EXP_REGISTER_OUTPUT,expanderOutput);

    // interrupt for motor fault
    pinMode(IF_EXPANDER_ELSE(INT_PIN,PRE_EXP_FAULT_PIN), INPUT); // interrupt/fault pin
    // todo: enable interrupt

    int gotConfig = readExpanderRegister(EXP_REGISTER_CONFIG);
    if(gotConfig == configuration) {
      DBGPRINTLN("i2c_exp configured");
    } else {
      DBGPRINTF("i2c_exp failed to configure: %x != %x\n",(int)configuration,(int)gotConfig);
    }

  } else {
    DBGPRINTLN("i2c_exp NOT found (old board)");
  }
}

void setExpanderOutput(byte pin,bool setHigh) {
  uint8_t newExpanderOutput = expanderOutput;
  if(setHigh) {
    newExpanderOutput |= (1<<pin);
  } else {
    newExpanderOutput &= 0xff ^ (1<<pin);
  }
  if(newExpanderOutput != expanderOutput) {
    expanderOutput = newExpanderOutput;
    writeExpanderRegister(EXP_REGISTER_OUTPUT,expanderOutput);
  }
}

bool getExpanderInput(byte pin) {
  uint8_t inputs = readExpanderRegister(EXP_REGISTER_INPUT);
  return (inputs & (1<<pin)) != 0;
}

int triggerPinCallback(uint32_t ulPin) {
  // We are assuming that this is the trigger pin; be sure to change this
  // if we end up with multiple interrupts.

  // For now, we are assuming that if this was called, the trigger is or just
  // was on.  Testing shows that it's possible for the pin to be low before
  // it gets read.  If for some reason the callback gets called while
  // there genuinely is no trigger, we'll have to reevaluate this.
#ifdef DEBUG_TRIGGER_INTERRUPT
  DBGPRINT("!");
#endif
  triggerInterruptCalled = true;
  return 0;
}

void setup_io() {
  analogReference(VBG); // Reference supply

  /*
  analogSelection(AIN_1_3_PS);  // selected analog pin input, 1/3 prescaling as the analog source
  int readBatt = analogRead(BATT_PIN);
  analogSelection(VDD_1_3_PS);  // VDD input, 1/3 prescaling as the analog source
  int readVdd = analogRead(BATT_PIN);
  delay(20);
  DBGPRINTF("batt = %d, vdd = %d\n",readBatt,readVdd);
  */

  // all boards: start i2c
  Wire.beginOnPins(SCL_PIN, SDA_PIN);
  delay(20);

  configureExpander();

  pinMode(TRIGGER_PIN, INPUT);
  RFduino_pinWakeCallback(TRIGGER_PIN, 0, triggerPinCallback) ;
  RFduino_pinWake(TRIGGER_PIN, HIGH);

  if(expanderPresent) {
    // new board
    pinMode(BATT_PIN, INPUT); // battery pin
    analogSelection(AIN_1_3_PS);  // selected analog pin input, 1/3 prescaling as the analog source
  } else {
    // old board
    pinMode(PRE_EXP_FAULT_PIN, INPUT); // fault pin
    pinMode(PRE_EXP_IGNITER_PIN, OUTPUT); // igniter pin

    // "battery" level from VDD
    analogSelection(VDD_1_3_PS);  // VDD input, 1/3 prescaling as the analog source
  }
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

  checkBatteryVoltage(millis());

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
  vsnprintf(buf, maxLen, format, myargs);
  va_end(myargs);
  bleSendString(buf);
}

// Store any updates to rssi.
void RFduinoBLE_onRSSI(int rssi) {
  curRSSI = rssi;
}

boolean checkMotorFault()
{
  if(expanderPresent) {
    return !getExpanderInput(EXP_PIN_FAULT);
  } else {
    return (digitalRead(PRE_EXP_FAULT_PIN) == LOW);
  }
}

float getBatteryVoltage() {
  if (ignoreBatteryVoltage)
    return BATTERY_FAKE_VOLTAGE;

  if(expanderPresent) {
#ifdef DEBUG_VOLTAGE_READING
    int batteryReading = analogRead(BATT_PIN);
    float batteryVoltage = REAL_BATTERY_V_SCALE * batteryReading;
    bleNPrintf(32, "battery: %d = %d.%03d", batteryReading, int(batteryVoltage), int(1000.0*(batteryVoltage-int(batteryVoltage))));
    return batteryVoltage;
#else
    return REAL_BATTERY_V_SCALE * analogRead(BATT_PIN);
#endif
  } // else
#ifdef DEBUG_VOLTAGE_READING
  int batteryReading = analogRead(1);
  float batteryVoltage = VDD_V_SCALE * batteryReading;
  bleNPrintf(32, "battery: %d = %d.%03d", batteryReading, int(batteryVoltage), int(1000.0*(batteryVoltage-int(batteryVoltage))));
  return batteryVoltage;
#else
  return VDD_V_SCALE * analogRead(1); // pin doesn't matter, reading from VDD src
#endif
}

float checkBatteryVoltage(unsigned long curTime) {
  // If the override flag has been set, go back to running.
  if (overrideBatteryVoltage) {
    // Disable low voltage detection, reset voltage state, and
    // make sure we don't have a stored interrupt.
    overrideBatteryVoltage = false;
    ignoreBatteryVoltage = true;
    voltageIsLow = false;
    voltageLowStartTime = TIME_NEVER;
    triggerInterruptCalled = false;
    fastUpdate(UPDATE_INTERVAL_IMMEDIATE);
  }

  float batteryVoltage = getBatteryVoltage();

  if (batteryVoltage < BATTERY_CUTOFF && !voltageIsLow) {
    // low voltage reading detected, not already in low power mode

    if(voltageLowStartTime == TIME_NEVER) {
      // turn off after voltage remains low for some measurable amount of time
      voltageLowStartTime = curTime;
      fastUpdate(UPDATE_INTERVAL_IMMEDIATE);
    }
    unsigned long elapsed = curTime - voltageLowStartTime;
    if (elapsed >= BATTERY_CUTOFF_MILLIS) {
      DBGPRINTF(" ---- LOW VOLTAGE %fV < %fV for %d ms ----\n", batteryVoltage, BATTERY_CUTOFF, (int)elapsed);
      voltageIsLow = true;
      fastUpdate(UPDATE_INTERVAL_IMMEDIATE);
      initDevices();
    }
  } else {
    // voltage is not low (now / anymore)
    voltageLowStartTime = TIME_NEVER;
    if(voltageIsLow && batteryVoltage >= BATTERY_RECOVERED_MIN_V) {
      // Voltage has recovered sufficiently to cancel the low voltage state.
      voltageIsLow = false;
      fastUpdate(UPDATE_INTERVAL_IMMEDIATE);
    }
  }

  return batteryVoltage;
}

void updateLeds(unsigned long curTime) {
 
  if (igniterState == IGNITER_STATE_ON) {
    // leave lights off if igniter is on
    setExpanderOutput(EXP_PIN_LED_GREEN, LED_OFF);
    setExpanderOutput(EXP_PIN_LED_RED, LED_OFF);

  } else if (isConnected && !voltageIsLow &&  0 != (curControllerTriggerState & (CTRL_BIT_LEFT_TRIGGER | CTRL_BIT_RIGHT_TRIGGER))) {
    // alternate lighting when user pulls xbox trigger (help identify blimp)
    if (curControllerTriggerState & CTRL_BIT_LEFT_TRIGGER) {
      setExpanderOutput(EXP_PIN_LED_GREEN, LED_ON);
    } else {
      setExpanderOutput(EXP_PIN_LED_GREEN, LED_OFF);
    }
    if (curControllerTriggerState & CTRL_BIT_RIGHT_TRIGGER) {
      setExpanderOutput(EXP_PIN_LED_RED, LED_ON);
    } else {
      setExpanderOutput(EXP_PIN_LED_RED, LED_OFF);
    }
    // after the triggers are both off, return to normal lighting
    ledState = LEDSTATE_START;

  } else {
    unsigned long elapsed = curTime - ledStateLastChange;
    ledStates ledNextState = ledState;

    switch (ledState) {
    case LEDSTATE_START:
      setExpanderOutput(EXP_PIN_LED_RED, LED_OFF);
      setExpanderOutput(EXP_PIN_LED_GREEN, LED_OFF);
    case LEDSTATE_BEACON_OFF:
      if (voltageIsLow) {
        ledNextState=LEDSTATE_LOW_VOLTAGE;
      } else if (!isConnected) {
        ledNextState=LEDSTATE_CONNECTING;
      } else if(elapsed >= LED_OFF_MILLIS ) {
        setExpanderOutput(EXP_PIN_LED_GREEN, LED_ON);
        ledNextState = LEDSTATE_BEACON_ON;
      }
      break;
    case LEDSTATE_BEACON_ON:
      if (elapsed >= LED_ON_MILLIS) {
        // turn off 
        setExpanderOutput(EXP_PIN_LED_GREEN, LED_OFF);
        ledNextState = LEDSTATE_BEACON_OFF;
      }
      break;

    case LEDSTATE_LOW_VOLTAGE:
      setExpanderOutput(EXP_PIN_LED_GREEN, LED_OFF);
    case LEDSTATE_LOW_VOLTAGE_BLINK_OFF:
      if (!voltageIsLow) {
        ledNextState = LEDSTATE_START;
      } else if (elapsed >= LED_OFF_MILLIS) {
        setExpanderOutput(EXP_PIN_LED_RED, LED_ON);
        ledNextState = LEDSTATE_LOW_VOLTAGE_BLINK_ON;
      }
      break;
    case LEDSTATE_LOW_VOLTAGE_BLINK_ON:
      if (elapsed >= LED_ON_MILLIS) {
        setExpanderOutput(EXP_PIN_LED_RED, LED_OFF);
        ledNextState = LEDSTATE_LOW_VOLTAGE_BLINK_OFF;
      }

    case LEDSTATE_CONNECTING:
      setExpanderOutput(EXP_PIN_LED_RED, LED_OFF);
      setExpanderOutput(EXP_PIN_LED_GREEN, LED_OFF);
    case LEDSTATE_CONNECTING_GREEN_OFF:
      if (voltageIsLow) {
        ledNextState=LEDSTATE_LOW_VOLTAGE;
      } else if (isConnected) {
        ledNextState=LEDSTATE_START;
      } else if (elapsed >= LED_OFF_SHORT_MILLIS ) {
        setExpanderOutput(EXP_PIN_LED_GREEN, LED_ON);
        ledNextState = LEDSTATE_CONNECTING_GREEN_ON;
      }
      break;
    case LEDSTATE_CONNECTING_GREEN_ON:
      if (elapsed >= LED_ON_MILLIS) {
        setExpanderOutput(EXP_PIN_LED_GREEN, LED_OFF);
        ledNextState = LEDSTATE_CONNECTING_RED_OFF;
      }
    case LEDSTATE_CONNECTING_RED_OFF:
      if (elapsed >= LED_OFF_SHORT_MILLIS) {
        setExpanderOutput(EXP_PIN_LED_RED, LED_ON);
        ledNextState = LEDSTATE_CONNECTING_RED_ON;
      }
      break;
    case LEDSTATE_CONNECTING_RED_ON:
      if (elapsed >= LED_ON_MILLIS) {
        setExpanderOutput(EXP_PIN_LED_RED, LED_OFF);
        ledNextState = LEDSTATE_CONNECTING_GREEN_OFF;
      }
      break;
    default:
      ledNextState = LEDSTATE_START;
    }

    // If we've changed state, mark the time.
    if (ledState != ledNextState) {
      ledStateLastChange = curTime;
      ledState = ledNextState;
    }
  }
}

void updateMotorFaults(unsigned long now) {
  // First check for any faults from the motor controllers and clear the ones we find.
  if (checkMotorFault()) {
    DBGPRINTLN(" ----FAULT----  ");
    for(int motorNum=0; motorNum<3; motorNum++) {
      uint8_t fault = getFault(motorIndexes[motorNum],motorNum,true);
#ifdef ENABLE_MOTOR_FAULT_MODE
      if (0 != (fault & MOTOR_FAULT_MODE_MASK) ) {
        if(motorFaultModeStart[motorNum] == TIME_NEVER) {
          // when the fault state begins, put the motor in coast
          setCoast(motorIndexes[motorNum]);
        }
        // keep updating the fault start time (re-faulting) while a fault is still present
        motorFaultModeStart[motorNum] = now;
      }
#endif
    }
  }

#ifdef ENABLE_MOTOR_FAULT_MODE
  // if a qualifying motor fault event happened recently, the time of the event 
  // will be stored in motorFaultModeStart[motor]
  //
  // here, we reset the fault mode after enough time has passed since it cleared
  for(int motorNum=0;motorNum<3;motorNum++) {
    if(motorFaultModeStart[motorNum] != TIME_NEVER) {
      int elapsed = now - motorFaultModeStart[motorNum];
      if(elapsed > MOTOR_FAULT_MODE_RESET_MILLIS) {
        // time to reset this motor and take it out of coast
        motorFaultModeStart[motorNum] = TIME_NEVER;
        // Stop the motor until the next message.
        setBrake(motorNum);
      } 
    }
  }
#endif
}

void loop()
{
  unsigned long loopMillis = millis();

  float batteryVoltage = checkBatteryVoltage(loopMillis);

  updateMotorFaults(loopMillis);

  // Time out and shut everything down if we haven't heard from the transmitter in too long.  Note that loopMillis can actually be less than lastPingMillis, as receives are asynchronous.
  if (loopMillis - lastPingMillis > CONNECTION_TIMEOUT && loopMillis > lastPingMillis && timeoutPossible == 1) {
    DBGPRINTLN(" TIMED OUT ");
    bleSendString(" TIMED OUT ");
    timeoutPossible = 0; //can only timeout once
    initDevices();
  }

  if(expanderPresent) {
    updateLeds(loopMillis);
  }

  if(!voltageIsLow && isConnected) {
    // Process any motor updates.
    processAllMotorUpdates();

    // Check if the triggers have changed state, then check if the Igniter needs to be turned on or off.
    updateBlimpTrigger(digitalRead(TRIGGER_PIN) == HIGH ? 0x01 : 0x00);
    updateControllerTrigger(nextControllerTriggerState);
    updateIgniterState();
  }

  // Check if it's time to send an update.
  if (loopMillis - lastUpdateMillis >= nextUpdateTime) {
    float curTemp = RFduino_temperature(FAHRENHEIT);
    byte statusFlags = (voltageIsLow ? RETURN_STATUS_LOW_VOLTAGE : 0);
    unsigned short batteryVoltageX100 = (100.0 * batteryVoltage); // convert value to hundredths of a volt
    int bufLen = 1 + sizeof(curRSSI) + sizeof(curTemp) + 6 + sizeof(batteryVoltageX100);
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
    buf[1 + sizeof(curRSSI) + sizeof(curTemp) + 5] = statusFlags;
    memcpy(&buf[1 + sizeof(curRSSI) + sizeof(curTemp) + 6], &batteryVoltageX100, sizeof(batteryVoltageX100));

    bleSendData(buf, bufLen);                 // Send the status update.
    lastUpdateMillis = millis();              // Record the last update time.
    nextUpdateTime = UPDATE_INTERVAL_DEFAULT; // Return the next update time to default.
  }
}

void RFduinoBLE_onConnect() {
  isConnected = true;
  DBGPRINTLN("connected");
#ifdef TEST_MOTORS_ON_CONNECT
  testMotors(0x3F, 100);
#endif
  fastUpdate(UPDATE_INTERVAL_IMMEDIATE); // Immediately send an update on connect.
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
      nextControllerTriggerState = data[cmdStart + 2];
    } else if (data[cmdStart + 0] == 0x04) {
      RFduino_systemReset();
    } else if (data[cmdStart + 0] == 0x05) {
      if (data[cmdStart + 1] == 0x01) {
        overrideBatteryVoltage = true;
      } else {
        bleSendString("voltage override canceled but not implemented");
      }
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

void processMotorUpdate(byte motorNum, uint8_t value, uint8_t speed) {
  // If the value has not changed, simply return.
  if (curMotorStates[motorNum][0] == value &&
      curMotorStates[motorNum][1] == speed)
    return;

  // Otherwise, update the motor setting and store the new values.
  if (value == 0x01) {
    setForward(motorNum, speed);
  } else if (value == 0x02) {
    setReverse(motorNum, speed);
  } else {
    setBrake(motorNum);
  }
}

// Try to update all motor states.  State differencing is done in processMotorUpdate.
void processAllMotorUpdates(void) {
  for (int i = 0; i < 3; i++) {
#ifdef ENABLE_MOTOR_FAULT_MODE
    if (motorFaultModeStart[i] != TIME_NEVER) {
      // there is a fault right now. motor should be in coast.
    } else
#endif
    processMotorUpdate(i, nextMotorStates[i][0], nextMotorStates[i][1]);
  }
}

void setIgniter(byte igniterCode) {
  // Set the igniter and then do a quick update.
  if(expanderPresent) {
    if (igniterCode == 0x01) {
      setExpanderOutput(EXP_PIN_IGNITER,true);
      DBGPRINTLN("EXP-IGNITER ON");
    } else {
      setExpanderOutput(EXP_PIN_IGNITER,false);
      DBGPRINTLN("EXP-IGNITER OFF");
    }
  } else {
    if (igniterCode == 0x01) {
      digitalWrite(PRE_EXP_IGNITER_PIN, HIGH);
      DBGPRINTLN("IGNITER ON");
    } else {
      digitalWrite(PRE_EXP_IGNITER_PIN, LOW);
      DBGPRINTLN("IGNITER OFF");
    }
  }
  igniterStateByte = igniterCode;
  fastUpdate(UPDATE_INTERVAL_IMMEDIATE);
}

void updateIgniter(byte igniterCode) {
  if (igniterCode != igniterStateByte)
    setIgniter(igniterCode);
}

void updateIgniterState(void) {
  unsigned long curMillis = millis();

  // Treat the controller and trigger as a single combined trigger.
  bool igniterTriggered = blimpTriggerState || triggerInterruptCalled;

#ifdef REMOTE_IGNITER
  igniterTriggered = igniterTriggered || (0 != (nextControllerTriggerState & CTRL_BIT_IGNITER));
#endif
  // Reset the interrupt, regardless of whether it triggered or not.
  // In theory a new trigger can happen before this clears it, but that
  // lost trigger is unlikely to matter when in the middle of handling an
  // active trigger.
  triggerInterruptCalled = false;

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
    DBGPRINTF("controller trigger state change: 0x%x\n", (int)triggerCode);
  }
}

void updateBlimpTrigger(byte triggerCode) {
  if (triggerCode != blimpTriggerState) {
    blimpTriggerState = triggerCode;
    DBGPRINTF("blimp trigger state change: %d%s\n", triggerCode, (triggerCode == 1 && igniterState == IGNITER_STATE_LOCKED) ? " (but IGNITER_STATE_LOCKED)" : "");
    fastUpdate();
  }
}

void sendMotorMessage(byte motor, uint8_t value, char *msg) {
  Wire.beginTransmission(motor);
  Wire.write(CONTROL);
  Wire.write(value);
  byte wireAck = Wire.endTransmission();
  DBGPRINTF("motor %d %s\n", motorNumFromIndex(motor), msg);
}

void setCoast(byte thisMotor) {
  int value = (V_SET << 2) | 0x00;
  sendMotorMessage(thisMotor, value, "coast");
  // Note that motor state is currently undefined here.  We could make it 0x03.
}

void setForward(byte thisMotor, int speed) {
  int value = (speed << 2) | 0x01;
  sendMotorMessage(motorIndexes[thisMotor], value, "forward");
  nextMotorStates[thisMotor][0] = curMotorStates[thisMotor][0] = 0x01;
  nextMotorStates[thisMotor][1] = curMotorStates[thisMotor][1] = speed;
}

void setReverse(byte thisMotor, int speed) {
  int value = (speed << 2) | 0x02;
  sendMotorMessage(motorIndexes[thisMotor], value, "reverse");
  nextMotorStates[thisMotor][0] = curMotorStates[thisMotor][0] = 0x02;
  nextMotorStates[thisMotor][1] = curMotorStates[thisMotor][1] = speed;
}

void setBrake(byte thisMotor) {
  int value = (V_SET << 2) | 0x03;
  sendMotorMessage(motorIndexes[thisMotor], value, "brake");
  nextMotorStates[thisMotor][0] = curMotorStates[thisMotor][0] = 0x00;
  nextMotorStates[thisMotor][1] = curMotorStates[thisMotor][1] = 0;
}

uint8_t getFault(int thisMotor, int motorNum, bool shouldClearFault) {
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
      char sendBuf[3] = {RETURN_MSG_FAULT, motorIndex, registerFault};
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

  return registerFault;
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
  setBrake(0);
  setBrake(1);
  setBrake(2);
}

void resetState(void) {
  initDevices();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++)
      curMotorStates[i][i] = 0;
  }
  blimpTriggerState = 0;
  curControllerTriggerState = 0;
  expectedMsgCounter = 0xff;
}

void testMotors(uint8_t velocity, int interval)
{
  if(voltageIsLow)
    return;

  initDevices();

  delay(25);
  clearAllFaults();
  delay(25);

  setForward(0, velocity);
  delay(interval);
  setBrake(0);
  delay(interval);

  setForward(1, velocity);
  delay(interval);
  setBrake(1);
  delay(interval);

  setForward(2, velocity);
  delay(interval);
  setBrake(2);
  delay(interval);
}
