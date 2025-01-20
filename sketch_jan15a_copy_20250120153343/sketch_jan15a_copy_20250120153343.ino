#include <ServoEasing.hpp>

#define DEFAULT_MICROSECONDS_FOR_0_DEGREE 500
#define DEFAULT_MICROSECONDS_FOR_180_DEGREE 2500

const int SERVO_PIN1 = 6;
const int SERVO_PIN2 = 4;
const int SERVO_PIN3 = 3;

const int TARGET_ANGLE1 = 5;
const int TARGET_ANGLE2 = 0;
const int TARGET_ANGLE3 = 20;

const int LIFT_SERVO_ANGLE = -5;
const int LOWER_SERVO_ANGLE = 10;

const int MOVE_ANGLE1 = 45;
const int MOVE_ANGLE3 = 45;

const int L1 = 75; // Длина первой секции
const int L2 = 65; // Длина второй секции

ServoEasing servo1;
ServoEasing servo2;
ServoEasing servo3;

void initializeServos() {
    servo1.attach(SERVO_PIN1, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, 0, 180);
    servo2.attach(SERVO_PIN2, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, 0, 180);
    servo3.attach(SERVO_PIN3, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE, 0, 180);

    servo1.setEasingType(EASE_CUBIC_IN_OUT);
    servo2.setEasingType(EASE_CUBIC_IN_OUT);
    servo3.setEasingType(EASE_CUBIC_IN_OUT);

  //   if (servo2.read() != TARGET_ANGLE1) {
  //   servo2.write(TARGET_ANGLE1);
  // }
  // if (servo1.read() != TARGET_ANGLE2) {
  //   servo1.write(TARGET_ANGLE2);
  // }
  // if (servo3.read() != TARGET_ANGLE3) {
  //   servo3.write(TARGET_ANGLE3);
  // }
  
}
void moveToCoordinates1(int x, int y) {

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

  
  moveToAngles(angle1, angle3);

  while (servo1.isMoving() || servo2.isMoving() || servo3.isMoving()) {
    delay(100);
  }

  Serial.flush();
}


void moveToAngles(int angle1, int angle3) {
    servo1.setEaseTo(angle1);
    servo3.setEaseTo(angle3);
    synchronizeAllServosStartAndWaitForAllServosToStop();
}

void moveToCoordinates(int x, int y) {
    static int currentX = 0; // Начальная X координата
    static int currentY = 0; // Начальная Y координата

    const int steps = 10; // Количество промежуточных точек
    float stepX = (x - currentX) / (float)steps;
    float stepY = (y - currentY) / (float)steps;

    for (int i = 1; i <= steps; i++) {
        float interpolatedX = currentX + stepX * i;
        float interpolatedY = currentY + stepY * i;

        float d = sqrt(interpolatedX * interpolatedX + interpolatedY * interpolatedY);
        float q1 = acos((d * d - L2 * L2 + L1 * L1) / (2 * L1 * d)) + atan2(interpolatedY, interpolatedX);
        float q2 = acos((L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2));

        float angle1 = round(q1 * (180 / M_PI));
        float angle3 = round(q2 * (180 / M_PI));

        if (isnan(angle1) || isnan(angle3)) {
            Serial.println("Недостижимая точка");
            return;
        }
      

        Serial.print("Moving to: ");
        Serial.print(interpolatedX);
        Serial.print(", ");
        Serial.print(interpolatedY);
        Serial.print(" | Angles: ");
        Serial.print(angle1);
        Serial.print(", ");
        Serial.println(angle3);

        moveToAngles(angle1, angle3);
        delay(0); // Небольшая задержка между шагами
    }

    currentX = x;
    currentY = y;
}
// void liftServo() {
//   servo2.setEaseTo(LIFT_SERVO_ANGLE);
//   synchronizeAllServosStartAndWaitForAllServosToStop();
// }

// void lowerServo() {
//   servo2.setEaseTo(LOWER_SERVO_ANGLE);
//   synchronizeAllServosStartAndWaitForAllServosToStop();
// }

void setup() {
    Serial.begin(115200);
    initializeServos();
    Serial.println("Initialization complete");
    // liftServo();
    //moveToCoordinates1(40,30);
    // lowerServo();
    //delay(1000);
    //moveToCoordinates1(10,40);
    // liftServo();
    //delay(1000);
    // Пример перемещения между двумя точками
    moveToCoordinates(50, 30); // Переместиться в точку (50, 30)
    // lowerServo();
    delay(1000);
    moveToCoordinates(20, 40);
    // liftServo(); // Переместиться в точку (20, 40)
}

void loop() {
    // В данном случае ничего не выполняется в цикле
}
