#define NUM_MOTORS 4

class Motor {
public:
  // Constructor to initialize motor pins
  Motor(int pin1, int pin2, int speedPin) {
    this->pin1 = pin1;
    this->pin2 = pin2;
    this->speedPin = speedPin;
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(speedPin, OUTPUT);
  }

  void moveForward(int speed) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(speedPin, speed);
  }

  void moveBackward(int speed) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(speedPin, speed);
  }

  void move(int speed) {
    if (speed > 0) {
      moveForward(speed);
    } else {
      moveBackward(-speed);
    }
  }

private:
  int pin1;
  int pin2;
  int speedPin;
};

// Create motor objects
Motor motor1(A0, A1, 6);
Motor motor2(A2, A3, 9);
Motor motor3(A5, A4, 5);
Motor motor4(8, 7, 10);


/*
The four motors' position:  M1  M2
                            M3  M4
*/

int clawServoPin = 11;
int armServoPin = 3;
int servoAngle;
bool openClaw = false;

unsigned long previousClockTime;
float millisCounter;

String BluetoothInput;
float x, y, radius;
float cos20radius;
double angle, cos20;
int CommaIndex;


void TurnLeft() {
  //wheel1 go backward
  motor1.moveBackward(100);
  //wheel2 go forward
  motor2.moveForward(100);
  //wheel3 go backward
  motor3.moveBackward(100);
  //wheel4 go forward
  motor4.moveForward(100);
}

void TurnRight() {
  //wheel1 go backward
  motor1.moveForward(100);
  //wheel2 go forward
  motor2.moveBackward(100);
  //wheel3 go backward
  motor3.moveForward(100);
  //wheel4 go forward
  motor4.moveBackward(100);
}

void moveServo(int angle, int pinNumber) {

  angle = constrain(angle, 0, 180);
  int pulseWidth = map(angle, 0, 180, 500, 2400);

  // Send the PWM signal
  digitalWrite(pinNumber, HIGH);  
  delayMicroseconds(pulseWidth);  
  digitalWrite(pinNumber, LOW);

  previousClockTime = millis();
  millisCounter = (20 - pulseWidth / 1000) / 1.024;

  // delay(20 - pulseWidth / 1000);
}

void CloseClaw() {
  openClaw = false;
}

void OpenClaw() {
  openClaw = true;
}

// void MoveArmServo(int angle) {
//   for (int i=0; i<15; i++) {
//     moveServo(angle, armServoPin);
//   }
// }


void setup() {
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);

  Serial.begin(9600);
}


void loop() {

  if (millis() - previousClockTime >= millisCounter) {
    moveServo(servoAngle, armServoPin);
    if (openClaw) {
      moveServo(135, clawServoPin);
    } else {
      moveServo(0, clawServoPin);
    }
  }

  if (Serial.available() > 0) {
    BluetoothInput = Serial.readStringUntil('$');

    if (BluetoothInput == "R") {
      TurnRight();
    } else if (BluetoothInput == "L") {
      TurnLeft();
    } else if (BluetoothInput == "O") {
      OpenClaw();
    } else if (BluetoothInput == "C") {
      CloseClaw();
    } else if (BluetoothInput == "T") {
      // top position
      servoAngle = 0;
    } else if (BluetoothInput == "M") {
      // middle position
      servoAngle = 45;
    } else if (BluetoothInput == "B") {
      // low position
      servoAngle = 90;
    }
    else {
      //convert input x,y into float number and get radius, cos(20)radius
      CommaIndex = BluetoothInput.indexOf(",");
      Serial.println(CommaIndex);

      // if (CommaIndex != -1) {
        angle = BluetoothInput.substring(0, CommaIndex).toFloat();
        radius = BluetoothInput.substring(CommaIndex + 1).toFloat();

        // radius = map(radius, 0, 150, 0, 255);
        // radius = radius >= 255 ? 255 : radius;

        angle = (angle) * (PI / 180);
        cos20 = cos(2 * angle);

        if (angle >= 0 && angle <= PI) {
          if (angle <= PI / 2) {
            // Serial.println("Quad 1");
            motor1.move(radius);
            motor4.move(radius);

            motor2.move(-cos20 * radius);
            motor3.move(-cos20 * radius);
          } else {
            // Serial.println("Quad 2");
            motor2.move(radius);
            motor3.move(radius);

            motor1.move(-cos20 * radius);
            motor4.move(-cos20 * radius);
          }
        } else if (angle <= 0 && angle >= -PI) {
            if (angle <= -PI / 2) {
              // Serial.println("Quad 3");
              motor1.move(-radius);
              motor4.move(-radius);

              motor2.move(cos20 * radius);
              motor3.move(cos20 * radius);
            } else {
              // Serial.println("Quad 4");
              motor2.move(-radius);
              motor3.move(-radius);

              motor1.move(cos20 * radius);
              motor4.move(cos20 * radius);
            }
        } 
    } // end of main swtich statements
  } 
}