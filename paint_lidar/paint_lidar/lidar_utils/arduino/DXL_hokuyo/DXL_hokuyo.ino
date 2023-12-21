#include "DxlMaster.h"

#define openValue    530
#define closeValue   515

#define baud_serial  115200

//Настройки для Dinamixel
#define baud_dxl     1000000
#define dxl_id       1
#define dxl_speed    300
#define max_range    1023

DynamixelMotor motor1((byte)dxl_id);

unsigned long timer = 0;
const int delay_timer = 100;
int currentValue = 0;
bool state = false;
int steps = 0;
String command = "";

void setup() {
  currentValue = closeValue;
  Serial.begin(baud_serial);
  DxlMaster.begin(baud_dxl);
  motor1.init();
  motor1.enableTorque();
  motor1.jointMode(204, 820);
  motor1.speed(dxl_speed);
  pinMode(LED_BUILTIN, OUTPUT);
  timer = millis();
  motor1.goalPosition(currentValue);
  Serial.println("Ready");
}

void loop() {
  String receivedData = readSerialString();
  if (receivedData != "") {
    command = receivedData;
    if (command == "n") {
      digitalWrite(LED_BUILTIN, HIGH);
      if (!state) motor1.goalPosition(openValue);
      Serial.println("Open");
      state = true;
    }
    if (command == "f") {
      digitalWrite(LED_BUILTIN, LOW);
      if (state) motor1.goalPosition(closeValue);
      Serial.println("Close");
      state = false;
    }
    if (command == "c") {
      Serial.println("Ready");
    }
  }
  command = "";
}

String readSerialString() {
  String serialString = "";
  while (Serial.available()) {
    char c = Serial.read();
    // Игнорируем символы новой строки и возврата каретки
    if (c != '\n' && c != '\r') {
      serialString += c;
    }
  }
  return serialString;
}
