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

#include <Pixy2.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// #define DEBUG_MODE

#define MOTOR_DIR_L_PIN 7
#define MOTOR_SPEED_L_PIN 6
#define MOTOR_SPEED_R_PIN 5
#define MOTOR_DIR_R_PIN 4

#define DEAD_ZONE 30
#define MAX_SPEED 200
#define TURN_SENSITIVITY 0.1
#define TURN_SPEED 2  //mult factor for turning


Adafruit_PWMServoDriver pwm;
Pixy2 pixy;  //Auflösung: 79x52

//TXD = 8 | RXD = 9
SoftwareSerial bluetoothSerial(8, 9);

const uint16_t SERVO_MAX = 560;
const uint16_t SERVO_MIN = 110;

const uint8_t X_SERVO_PIN = 0;
const uint8_t Y_SERVO_PIN = 4;

/*
0 = connected pos
1 = ready pos
2 = home  pos
*/

const short H_0s[3] = { 175, 155, 110 };

//x position der Oberleitung
float oberleitung_x = -1;

//beschreibt die höhe in der sich der Stromabnehmer befindet
int currentState = 1;

//wird von den Werten der pixycam gesetzt
double currSidePos = 90;  //grad



//Bluetooth controll values
int btAngle = 0;
int btStrenght = 0;
int btButton = 0;

//Motor controll values
int speedLeft = 0;   //derzeitige geschwindigkeit der linken räder
int speedRight = 0;  //derzeitige geschwindigkeit der rechten räder

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize the PCA9685
  Serial.println("Init PCA9685...");
  pwm.begin();
  pwm.setPWMFreq(50);  // Set the PWM frequency (adjust according to your servo)
  Serial.println("Finished!");


  // Initialize the Pixycam
  Serial.println("Init PixyCam...");
  pixy.init();
  pixy.setLamp(0, 0);
  delay(500);
  pixy.setLamp(1, 1);
  delay(1500);
  pixy.setLamp(0, 0);
  // change to the line_tracking program.  Note, changeProg can use partial strings, so for example,
  // you can change to the line_tracking program by calling changeProg("line") instead of the whole
  // string changeProg("line_tracking")
  Serial.println(pixy.changeProg("line"));
  Serial.println("Finished!");

  Serial.println("Init Bluetooth...");
  bluetoothSerial.begin(9600);
  Serial.println("Finished!");

  pinMode(MOTOR_DIR_L_PIN, OUTPUT);
  pinMode(MOTOR_DIR_R_PIN, OUTPUT);
  pinMode(MOTOR_SPEED_L_PIN, OUTPUT);
  pinMode(MOTOR_SPEED_R_PIN, OUTPUT);

  //TODO Autokalibrierung

  Serial.println("Running...");
}
void loop() {
  read_bluetooth();
  set_speed();
  update_pixycam();
  //follow_line();

#ifdef DEBUG_MODE
  Serial.println("Left   Speed: " + (String)currLeftSideSpeed);
  Serial.println("Right  Speed: " + (String)currRightSideSpeed);
#endif
}
void update_pixycam() {
  const int delta_x_threshold = 6;//in pixel
  // pixy.line.getMainFeatures(); // sollte man nicht benutzen weil es nur die Vectoren holt
  pixy.line.getAllFeatures();  //diese holt alle Features also auch die linien

  Serial.print("Vector Count: ");
  Serial.println(pixy.line.numVectors);


  if (pixy.line.numVectors == 0) {
    // oberleitung_x = -1;
    //keine oberleitug gefunden
    return;
  }

  int selectedVecCount = 0;
  float xmid = 0;
  // float ymid = 0;
  Vector* curr_vector_ptr = pixy.line.vectors;
  for (int i = 0; i < pixy.line.numVectors; i++) {
    //go to next vector
    curr_vector_ptr += i;
    
    //da viele falsche Vektoren erkannt werden 
    //müssen wir überprüfen ob diese in frage kommen 
    int delta_x = abs(curr_vector_ptr->m_x0 - curr_vector_ptr->m_x1);
    if(delta_x > delta_x_threshold){
      continue;
    }

    xmid += curr_vector_ptr->m_x0;
    xmid += curr_vector_ptr->m_x1;
    selectedVecCount++;
    // ymid += curr_vector_ptr->m_y0;
    // ymid += curr_vector_ptr->m_y1;
  }
  xmid /= selectedVecCount * 2;
  // ymid /= pixy.line.numVectors * 2;

  oberleitung_x = xmid;
  Serial.println((String) xmid);
}
void follow_line() {
  if (oberleitung_x < 70 && oberleitung_x > 7) {
    currSidePos = -1.36 * oberleitung_x + 135; //Formel generiert mit Excel
    set_servo_position(X_SERVO_PIN, currSidePos - 6);

    double theta = calc_theta_from_phi(currSidePos, H_0s[currentState]);
    Serial.println((String)theta);

    theta = constrain(theta, 70, 170);
    set_servo_position(Y_SERVO_PIN, theta);
  }
}
void set_servo_position(uint8_t servoChannel, double angle) {
  angle = constrain(angle, 0, 180);
  // Convert the angle to pulse width
  uint16_t pulseWidth = (int)(map(angle, 0, 180, SERVO_MIN, SERVO_MAX) + 0.5);

  // Set the pulse width on the servo channel
  pwm.setPWM(servoChannel, 0, pulseWidth);
  // Serial.println("side pos   " + (String)currSidePos);
  // Serial.println("oberleitung_x " + (String)oberleitung_x);
  //Serial.println("Pulse Width: " + (String)pulseWidth);
}
double calc_theta_from_phi(uint16_t phi, uint16_t H_0) {
  const auto H_00 = 170;  //175;
  const auto s = 150.0;   //158.2;
  const auto r_k = 100;

  const auto seitlicheAblage = cos(radians(phi)) * r_k;

  const auto delta_h = (1 - sin(radians(phi))) * r_k;
  const auto l_0 = sqrt(H_00 * H_00 + s * s);
  const auto l = sqrt(pow(H_00 - delta_h, 2) + s * s);
  const auto theta_0 = asin(H_0 / l_0);
  const auto theta_00 = atan(H_00 / s);
  const auto delta_theta = asin(H_0 / l) - asin((H_0 - delta_h * cos(theta_00 - theta_0)) / l);
  auto theta = PI / 2 + theta_00 - theta_0 - delta_theta;
  return degrees(theta);  // - delta_theta;
}
void set_speed() {
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
void read_bluetooth() {
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

    if (is_digit(angle)) {
      btAngle = angle.toInt();
    }
    if (is_digit(strength)) {
      btStrenght = strength.toInt();
    }
    if (is_digit(button)) {
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
bool is_digit(String text) {
  for (size_t i = 0; i < text.length(); i++) {
    if (!isdigit(text.charAt(i))) {
      return false;
    }
  }
  return true;
}