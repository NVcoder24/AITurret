#include <GParser.h>
#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void setup() {
  servo1.attach(11);
  servo2.attach(10);
  servo3.attach(9);
  servo4.attach(6);

  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(140);

  pinMode(2, OUTPUT);

  Serial.begin(9600);
  Serial.setTimeout(5);
}

void loop() {
  if (Serial.available()) {
    char str[30];
    int amount = Serial.readBytesUntil(';', str, 30);
    str[amount] = NULL;
    GParser data(str, ',');
    int am = data.split();

    int cmd = data.getInt(0);
    if (cmd == 1) {
      float ang1 = data.getFloat(1);
      float ang2 = data.getFloat(2);
      servo1.write((float)90 - ang1);
      servo2.write((float)90 - ang2);
      servo3.write((float)90 + ang2);
    } else if (cmd == 2) {
      float ang = data.getFloat(1);
      servo4.write(ang);
    } else if (cmd == 3) {
      int d = data.getInt(1);
      digitalWrite(2, d);
    }
  }
}
