#define servoPin    3
#define openValue   70
#define closeValue  130

#include <Servo.h>

Servo myservo;

unsigned long timer = 0;
const int delay_timer = 100;
int currentValue = 0;
bool state = false;
int steps = 0;

void setup() {
  myservo.attach(servoPin);
  currentValue = closeValue;
  myservo.write(closeValue);

  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115200);
  Serial.setTimeout(100);
  Serial.println("Ready!");

  timer = millis();
}

void loop() {

  if (millis() - timer > 100) {
    timer = millis();
    //Serial.println("Ok");
  }

  if (Serial.available()) {
    String command = Serial.readString();
    command.trim(); //remove \r\n
    if (command == "On" || command == "on") {
      digitalWrite(LED_BUILTIN, HIGH);
      //if (!state) smoothServo(openValue, 10);
      if (!state) myservo.write(openValue);
      Serial.println("Open");
      state = true;
    }
    if (command == "Off" || command == "off") {
      digitalWrite(LED_BUILTIN, LOW);
      //if (state) smoothServo(closeValue, 10);
      if (state) myservo.write(closeValue);
      Serial.println("Close");
      state = false;
    }
  }
}

void smoothServo(int goal, int delayTime)
{
  if (goal != currentValue) {
    int current = 0;
    int steps = 10;
    for (int i = 0; i < steps + 1; i++) {
      current = map(i, 0, steps, currentValue, goal);
      Serial.println(current);
      myservo.write(current);
      delay(delayTime);
    }
    currentValue = current;
  }
}
