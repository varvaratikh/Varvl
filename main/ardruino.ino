#include <stdio.h>
#include <string.h>
#include <ServoEasing.hpp>

#define DEFAULT_MICROSECONDS_FOR_0_DEGREE 500
#define DEFAULT_MICROSECONDS_FOR_180_DEGREE 2500

const int SERVO_PIN1 = 3;
const int SERVO_PIN2 = 4;
const int SERVO_PIN3 = 6;

const int TARGET_ANGLE1 = 5;
const int TARGET_ANGLE2 = 0;
const int TARGET_ANGLE3 = 20;

const int LIFT_SERVO_ANGLE = -5;
const int LOWER_SERVO_ANGLE = 10;

const int MOVE_ANGLE1 = 45;
const int MOVE_ANGLE3 = 45;

static int servoSpeed = 30;

ServoEasing servo1;
ServoEasing servo2;
ServoEasing servo3;

enum CommandType {
  UNKNOWN_COMMAND,
  G0_COMMAND,
  G1_COMMAND
};

struct ParsedCoordinates {
  CommandType command;
  int x;
  int y;
};

void initializeServos() {
  servo1.attach(SERVO_PIN1, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, 0, 180);
  servo2.attach(SERVO_PIN2, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, 0, 180);
  servo3.attach(SERVO_PIN3, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, 0, 180);
  servo1.setEasingType(EASE_CUBIC_IN_OUT);
  servo2.setEasingType(EASE_CUBIC_IN_OUT);
  servo3.setEasingType(EASE_CUBIC_IN_OUT);

  if (servo2.read() != TARGET_ANGLE1) {
    servo2.write(TARGET_ANGLE1);
  }
  if (servo1.read() != TARGET_ANGLE2) {
    servo1.write(TARGET_ANGLE2);
  }
  if (servo3.read() != TARGET_ANGLE3) {
    servo3.write(TARGET_ANGLE3);
  }
}

void setup() {
  Serial.begin(115200);
  initializeServos();
  setSpeedForAllServos(servoSpeed);
}

void moveServosSequentially() {
  if (servo2.read() != TARGET_ANGLE1) {
    int currentAngle2 = servo2.read();
    for (int angle = currentAngle2; angle <= TARGET_ANGLE1; angle++) {
      servo2.write(angle);
      delay(500);
    }
  }
  delay(500);

    if (servo1.read() != TARGET_ANGLE2) {
    int currentAngle1 = servo1.read();
    for (int angle = currentAngle1; angle >= TARGET_ANGLE2; angle--) {
      servo1.write(angle);
      delay(500);
    }
  }

  if (servo3.read() != TARGET_ANGLE3) {
      int currentAngle3 = servo3.read();
      for (int angle = currentAngle3; angle <= TARGET_ANGLE3; angle++) {
         servo3.write(angle);
         delay(500);
      }
  }
}

ParsedCoordinates parseCoordinates(String line) {
  ParsedCoordinates coords;
  coords.command = UNKNOWN_COMMAND;
  coords.x = 0;
  coords.y = 0;

  line.trim();

  if (line.length() > 0) {
    char command = line.charAt(0);
    coords.command = UNKNOWN_COMMAND;

    int firstSpace = line.indexOf(' ');
    int secondSpace = line.indexOf(' ', firstSpace + 1);

    if (firstSpace != -1 && secondSpace != -1) {
      String xStr = line.substring(firstSpace + 1, secondSpace);
      String yStr = line.substring(secondSpace + 1);

      coords.x = xStr.toInt();
      coords.y = yStr.toInt();

      if (command == 'U') {
        coords.command = G0_COMMAND;
      } else if (command == 'D') {
        coords.command = G1_COMMAND;
      } else {
        Serial.print("Unknown command: ");
        Serial.println(command);
      }
    } else {
      Serial.println("Invalid coordinate format");
    }
  }
    return coords;
}

#define L1 75
#define L2 65

// ограничения на углы
// #define MIN_ANGLE1 5
// #define MAX_ANGLE1 180
// #define MIN_ANGLE3 5
// #define MAX_ANGLE3 180

void moveToCoordinates(ParsedCoordinates coords) {
  int x = coords.x;
  int y = coords.y;
  CommandType type = coords.command;

  float d = sqrt(x * x + y * y);
  float q1 = acos((d * d - L2 * L2 + L1 * L1) / (2 * L1 * d)) + atan2(y, x);
  float q2 = acos((L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2));

  float angle1 = round(q1 * (180 / M_PI));
  float angle3 = round(q2 * (180 / M_PI));
  if (isnan(angle1) || isnan(angle3)) {
      Serial.println("Недостижимая точка");
      return;
  }
  Serial.print(angle1);
  Serial.print(" ");
  Serial.println(angle3);

  // angle1 = constrain(angle1, 0, 180);
  // angle3 = constrain(180 - angle3, 0, 180);

  if (coords.command == G0_COMMAND) {
    liftServo();
    delay(500);
  } else if (coords.command == G1_COMMAND) {
    lowerServo();
    delay(500);
  }

  moveToAngles(angle1, angle3);

  while (servo1.isMoving() || servo2.isMoving() || servo3.isMoving()) {
    delay(100);
  }

  Serial.flush();
}

void liftServo() {
  servo2.setEaseTo(LIFT_SERVO_ANGLE);
  synchronizeAllServosStartAndWaitForAllServosToStop();
}

void lowerServo() {
  servo2.setEaseTo(LOWER_SERVO_ANGLE);
  synchronizeAllServosStartAndWaitForAllServosToStop();
}

void moveToAngles(int angle1, int angle3) {
  servo1.setEaseTo(angle1);
  servo3.setEaseTo(angle3);
  synchronizeAllServosStartAndWaitForAllServosToStop();
}

void loop() {
    if (Serial.available() > 0) {
    String gcode = Serial.readStringUntil('\n');
    gcode.trim();
    // Serial.println("Line after trim: " + gcode);

    ParsedCoordinates coords = parseCoordinates(gcode);
    if (coords.command != UNKNOWN_COMMAND) {
      moveToCoordinates(coords);
      delay(1000);
    }
  }
}