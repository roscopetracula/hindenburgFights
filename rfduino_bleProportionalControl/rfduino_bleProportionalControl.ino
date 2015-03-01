#include <Wire.h>	
#include <RFduinoBLE.h>

#define Vref = 1.285
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

int sclPin = 2; // 5 for slimstack board
int sdaPin = 3; // 6 for slimstack board
int faultPin = 4;

void setup()
{
  pinMode(faultPin, INPUT);
  Wire.beginOnPins(sclPin, sdaPin);
  delay(20);
  RFduinoBLE.deviceName = "RFduino Blimp";
  RFduinoBLE.begin();
  Vset = 0x3F;
  lastPing = millis();
  testMotors(0x3F, 200);
  Serial.println("ready to go!"); 
}

void loop()
{
  thisLoopMillis=millis();

  if (digitalRead(faultPin) == LOW) {
    Serial.println(" ----FAULT----  ");
    clearAllFaults();
  }

  if ((thisLoopMillis - lastPing) > timeout && timeoutPossible == 1){
    Serial.print("lastPing timeout:  ");
    Serial.println(lastPing);
    Serial.print("thisLoopMillis:    ");
    Serial.println(thisLoopMillis);
    setBrake(MOTOR1);
    setBrake(MOTOR2);
    setBrake(MOTOR3);
    timeoutPossible = 0; //can only timeout once
    Serial.println(" TIMED OUT ");
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

void RFduinoBLE_onReceive(char *data, int len) {
  lastPing=millis();
  Serial.print("lastPing set:    ");
  Serial.println(lastPing);
  Serial.println( "-----------RX-----------");
  clearAllFaults();

  if (len == 6) {
    receiveMotorCommand(MOTOR1, data[0], data[1]);
    receiveMotorCommand(MOTOR2, data[2], data[3]);
    receiveMotorCommand(MOTOR3, data[4], data[5]);
  }

  // now that we've recieved data it's possible to timeout.
  timeoutPossible = 1;
}

void receiveMotorCommand(byte motor, uint8_t value, uint8_t speed) {
  if (value == 0x01) {
    setForward(motor, speed);
  } else if (value == 0x02) {
    setReverse(motor, speed);
  } else {
    setBrake(motor);
  }
}

void sendMessage(byte motor, uint8_t value, String msg) {
  Wire.beginTransmission(motor);
  Wire.write(CONTROL);
  Wire.write(value);
  Serial.print(msg);
  Serial.println(Wire.endTransmission());
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

void testMotors(uint8_t velocity, int interval)
{
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
