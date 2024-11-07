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
Motor motor1(8, 10, 9);
Motor motor2(5, 7, 6);
Motor motor3(12, 13, 11);
Motor motor4(2, 4, 3);


/*
The four motors' position:  M1  M2
                            M3  M4
*/


String BluetoothInput;
float x, y, radius;
float cos20radius;
double angle, cos20;
int CommaIndex;


void TurnLeft() {
  //wheel1 go backward
  motor1.moveBackward(255);
  //wheel2 go forward
  motor2.moveForward(255);
  //wheel3 go backward
  motor3.moveBackward(255);
  //wheel4 go forward
  motor4.moveForward(255);
}

void TurnRight() {
  //wheel1 go backward
  motor1.moveBackward(255);
  //wheel2 go forward
  motor2.moveForward(255);
  //wheel3 go backward
  motor3.moveBackward(255);
  //wheel4 go forward
  motor4.moveForward(255);
}


void setup() {
  Serial.begin(9600);
}


void loop() {

  if (Serial.available()) {

    BluetoothInput = Serial.readStringUntil('$');

    if (BluetoothInput == "R") {
      TurnRight();
    } else if (BluetoothInput == "L") {
      TurnLeft();
    } else {
      //convert input x,y into float number and get radius, cos(20)radius
      CommaIndex = BluetoothInput.indexOf(",");
      angle = BluetoothInput.substring(0, CommaIndex).toFloat();
      radius = BluetoothInput.substring(CommaIndex + 1).toFloat();

      radius = radius >= 255 ? 255 : radius;

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
    }
  }
}