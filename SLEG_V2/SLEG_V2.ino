/* PIN connections
-------------------------BEERLECADA MOTOR DRIVER DUAL-------------------------
                      |              Uno              |
GND                   |                               |                GND
IN1B PWM_RIGHT yellow | pin 6                         |                VM
IN2B DIR_RIGHT green  | pin 4                   pin 7 | green DIR_LEFT IN2A
IN1A PWM_LEFT  yellow | pin 5                         |                SBY
...
-------------------------------------------------------------------------------


DIR LOW Forward
DIR HIGH Backwards
*/

//How to use Motors: (TEST)
// // motor A (left) forward
// digitalWrite(MOTOR_DIR_L_PIN, LOW);
// analogWrite(MOTOR_SPEED_L_PIN, 60);
// // motor B (right) forward
// digitalWrite(MOTOR_DIR_R_PIN, LOW);
// analogWrite(MOTOR_SPEED_R_PIN, 60);
// delay(5000);
// // motor A (left) backwards
// digitalWrite(MOTOR_DIR_L_PIN, HIGH);
// analogWrite(MOTOR_SPEED_L_PIN, 195);
// // motor B (right) backwards
// digitalWrite(MOTOR_DIR_R_PIN, HIGH);
// analogWrite(MOTOR_SPEED_R_PIN, 195);
// delay(5000);

#include <SoftwareSerial.h>

// #define DEBUG_MODE

#define MOTOR_DIR_L_PIN 7
#define MOTOR_SPEED_L_PIN 6
#define MOTOR_SPEED_R_PIN 5
#define MOTOR_DIR_R_PIN 4

#define DEAD_ZONE 30
#define MAX_SPEED 200
#define TURN_SENSITIVITY 0.1
#define TURN_SPEED 2  //mult factor for turning


//TXD = 8 | RXD = 9
SoftwareSerial bluetoothSerial(8, 9);

//Bluetooth controll values
int btAngle = 0;
int btStrenght = 0;
int btButton = 0;

//Motor controll values
int speedLeft = 0;   //derzeitige geschwindigkeit der linken räder
int speedRight = 0;  //derzeitige geschwindigkeit der rechten räder

void setup() {
  Serial.begin(9600);
  Serial.println("Los geht's");
  bluetoothSerial.begin(9600);

  pinMode(MOTOR_DIR_L_PIN, OUTPUT);
  pinMode(MOTOR_DIR_R_PIN, OUTPUT);
  pinMode(MOTOR_SPEED_L_PIN, OUTPUT);
  pinMode(MOTOR_SPEED_R_PIN, OUTPUT);
}
void loop() {
  readBluetooth();
  setSpeed();

#ifdef DEBUG_MODE
  Serial.println("Left   Speed: " + (String)currLeftSideSpeed);
  Serial.println("Right  Speed: " + (String)currRightSideSpeed);
#endif
}
void setSpeed() {
  float angleRad = radians(btAngle);
  byte speedDir = 0;
  if (btAngle <= 90) {
    //forward
    speedDir = 0;
    speedLeft = sin(angleRad) * (MAX_SPEED / 100) * btStrenght;
    speedRight = speedLeft - map(btAngle, 0, 90, speedLeft, 0);
  } else if (btAngle <= 180) {
    //forward
    speedDir = 0;
    speedRight = sin(angleRad) * (MAX_SPEED / 100) * btStrenght;
    speedLeft = speedRight - map(btAngle, 180, 90, speedRight, 0);
  } else if (btAngle <= 270) {
    //backward
    speedDir = 1;
    speedRight = abs(sin(angleRad)) * (MAX_SPEED / 100) * btStrenght;
    speedLeft = speedRight - map(btAngle, 270, 180, 0, speedRight);
    speedRight = 255 - speedRight;
    speedLeft = 255 - speedLeft;
  } else if (btAngle <= 360) {
    //backward
    speedDir = 1;
    speedLeft = abs(sin(angleRad)) * (MAX_SPEED / 100) * btStrenght;
    speedRight = speedLeft - map(btAngle, 270, 360, 0, speedLeft);
    speedRight = 255 - speedRight;
    speedLeft = 255 - speedLeft;
  }
  // Serial.println("\t" + (String)speedDir);
  Serial.println((String)speedLeft + "\t\t" + (String)speedRight);

  digitalWrite(MOTOR_DIR_R_PIN, speedDir);
  analogWrite(MOTOR_SPEED_R_PIN, speedRight);
  digitalWrite(MOTOR_DIR_L_PIN, speedDir);
  analogWrite(MOTOR_SPEED_L_PIN, speedLeft);
}
void readBluetooth() {
  /*
  If joystick to the right -> 0 degrees
  top = 90
  left = 180
  bottom =270
  */
  if (bluetoothSerial.available()) {
    String bluetoothInput = bluetoothSerial.readStringUntil('#');
    // #ifdef DEBUG_MODE
    // Serial.println("Bluetooth input:\t" + bluetoothInput);
    // #endif
    if (bluetoothInput.length() != 7) {
      return;
    }
    String angle = bluetoothInput.substring(0, 3);
    String strength = bluetoothInput.substring(3, 6);
    String button = bluetoothInput.substring(6, 8);

    if (isDigit(angle)) {
      btAngle = angle.toInt();
    }
    if (isDigit(strength)) {
      btStrenght = strength.toInt();
    }
    if (isDigit(button)) {
      btButton = button.toInt();
    }
    // #ifdef DEBUG_MODE
    //     Serial.println("Angle\t\t\t" + (String)btAngle);
    //     Serial.println("Strength\t\t" + (String)btStrenght);
    //     Serial.println("Button\t\t\t" + (String)btButton);
    // #endif
    bluetoothSerial.flush();
    bluetoothInput = "";
  }
}
bool isDigit(String text) {
  for (size_t i = 0; i < text.length(); i++) {
    if (!isdigit(text.charAt(i))) {
      return false;
    }
  }
  return true;
}