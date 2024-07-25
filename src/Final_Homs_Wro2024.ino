#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <math.h>
#include <NewPing.h>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define TRIGGER_PIN_R1 52  // Ultra Sonic defines with these objects
#define ECHO_PIN_R1 A14
///////////////////////////////////////////////////////////
#define TRIGGER_PIN_L1 20
#define ECHO_PIN_L1 A0
////////////////////////////////////////////////////////////
#define TRIGGER_PIN_R2 50
#define ECHO_PIN_R2 A13
////////////////////////////////////////////////////////////
#define TRIGGER_PIN_L2 40
#define ECHO_PIN_L2 A15
////////////////////////////////////////////////////////////
#define TRIGGER_PIN_F 46
#define ECHO_PIN_F A11
////////////////////////////////////////////////////////////
#define MAX_DISTANCE 200
/////////////////////////////////////////////////////////////
NewPing sonar_R1(TRIGGER_PIN_R1, ECHO_PIN_R1, MAX_DISTANCE);
NewPing sonar_L1(TRIGGER_PIN_L1, ECHO_PIN_L1, MAX_DISTANCE);
NewPing sonar_R2(TRIGGER_PIN_R2, ECHO_PIN_R2, MAX_DISTANCE);
NewPing sonar_L2(TRIGGER_PIN_L2, ECHO_PIN_L2, MAX_DISTANCE);
NewPing sonar_F(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE);
///////////////////////////////////////////////////////////////
Servo ESC;
Servo myservo;
MPU6050 mpu;
/////////////////////////////////////////Gyro
const int calibrationSamples = 1000;
float gyroZOffset = 0;
float angleZ = 0;
////////////////////////////////////////
double target = 0;  // Set your desired orientation angle (adjust as needed)
double z;
double err; 
/////////////////////////stander muser
int T_angle = 20;
int dis_b = 10; //make it 11 if need
int denger_dis = 13 ;
////////////////////////////prop
float kpR = 1;
float KPL=1;
float Kp = 2.2;
///////////////////correction
float ZL1;
float error_l;
float teta_l;
float L;
int angel_ZL;
float ZR;
float error_r;
float teta_r;
float R;
int angel_ZR;
float integral;
float prevErr;
float privosZ=0;
///////////////sensor
int L1;
int L2;
int R1;
int R2;
int F;
int button_state;
///////////////////timer
unsigned long startMillis;  //variables for turning
unsigned long currentMillis;
unsigned long period = 0;
unsigned long timer = 0;
//////////////////Motor
int Speed = 11;
int center = 86;
//////////////////
int turn = 0;
int Target_AR;
int Target_AL;
int buttonPin=30;
//////////////////////////////////////////////////////////functions'
void mpu_setup() {

  mpu.initialize();
  // Verify the connection
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }
  calibrateGyro();
}

//________________________________________________________________________________________________________
void mpu_loop() {

  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // Convert milliseconds to seconds
  lastTime = currentTime;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Gyro rate for Z-axis (in degrees per second) with offset correction
  float gyroRateZ = (gz - gyroZOffset) / 131.0;

  // Integrate the gyro rate to get the yaw angle
  angleZ += gyroRateZ * dt;

  // Apply a simple condition to stabilize the angle when not rotating
  if (abs(gyroRateZ) < 0.1) {  // Assuming small drift values, can be adjusted

    gyroRateZ = 0;  // Zero out the small drift rate
  }

  // Print the yaw angle
  // Serial.print("Z: ");
  // Serial.println(z);
  delayMicroseconds(10);
}

//________________________________________________________________________________________________________
void calibrateGyro() {
  long gzOffsetSum = 0;
  for (int i = 0; i < calibrationSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gzOffsetSum += gz;
    delay(1);
  }
  gyroZOffset = gzOffsetSum / calibrationSamples;
}
//________________________________________________________________________________________________________
double P_id_gyro() {
  err = target - angleZ;
  integral += err;
  // Calculate the control output (PID)
  double output = Kp * err;
  // Map the output to servo position (adjust the mapping range)
  double mappedOutput = map(output, -90, 90, 43, 133);
  
  myservo.write(mappedOutput);
  // Update previous error for next iteration
  prevErr = err;
  // Serial.print("z = ");
  // Serial.println(z);
  // Serial.print("target = ");
  // Serial.println(target);
  // Serial.print("err = ");
  // Serial.println(err);
  return err;
}

void sensor() {
  L1 = sonar_L1.ping_cm();
  if (L1 == 0) { L1 = 200; }
  delayMicroseconds(100);
  L2 = sonar_L2.ping_cm();
  if (L2 == 0) { L2 = 200; }
  delayMicroseconds(100);
  R1 = sonar_R1.ping_cm();
  if (R1 == 0) { R1 = 200; }
  delayMicroseconds(100);
  R2 = sonar_R2.ping_cm();
  if (R2 == 0) { R2 = 200; }
  delayMicroseconds(100);
  F = sonar_F.ping_cm();
  if (F == 0) { F = 200; }
  delayMicroseconds(100);
  Serial.print("L1=");
  Serial.println(L1);
  Serial.print("L2=");
  Serial.println(L2);
  Serial.print("R1=");
  Serial.println(R1);
  Serial.print("R2=");
  Serial.println(R2);
  Serial.print("F=");
  Serial.println(F);
  mpu_loop();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void correction_L(int L1, int L2) {
  ZL1 = float((L1 - L2) / dis_b);
  error_l = atan(ZL1);
  teta_l = (error_l * 180.0) / PI;
  Serial.print("Theta = ");
  Serial.print(teta_l);
  if (teta_l >T_angle){teta_l = T_angle;}
  if (teta_l <-T_angle){teta_l =- T_angle;}
  angel_ZL = (teta_l * KPL) + center;
  Serial.print(" \nAngel ZL1=");
  Serial.println(angel_ZL);
  L = abs(((L1 + L2) / 2 * cos(1)));
}

void dis_l(float ZL1) {
  myservo.write((ZL1 - denger_dis) * KPL + center);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void correction_R(int R1, int R2) {
  ZR = (R1 - R2) / dis_b;
  error_r = atan(ZR);
  teta_r = (error_r * 180) / PI;
  if (teta_r > T_angle) { teta_r = T_angle; }
  if (teta_r < -T_angle) { teta_r = -T_angle; }
  angel_ZR = -1 * (teta_r * kpR) + center;
  Serial.print("Angel ZR=");
  Serial.println(angel_ZR);

  R = abs(((R1 + R2) / 2) * (cos(1)));
}
void dis_r(float ZR) {
  myservo.write(-1 * (ZR - denger_dis) * kpR + center);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Stop() {
  ESC.write(0);
 
  delay(10000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000);
}
/////////////////////////////start program
void setup() {
  Serial.begin(115200);
  ESC.attach(9, 1000, 2000);
  delay(2000);
  ESC.write(0);
  myservo.attach(3);
  mpu_setup();
  myservo.write(86);
  Serial.println("Done!\n");
  // delay(2000);
  startMillis = millis();
//   while(!digitalRead(buttonPin)){
//     mpu_loop();
//   }
//   Serial.println("Done!\n");
 }


void loop() {
   if (abs(turn)==12 ){
    ESC.write(Speed);
    delay(800);
    Speed=0;
  
   }else{

ESC.write(Speed);
  sensor();
  correction_L(L1, L2);
  correction_R(R1, R2);
  if (turn == 0) {
    P_id_gyro();
  }
  if (turn > 0) {
    if (L1 < denger_dis) {
      dis_l(ZL1);
    } else if (R1 < denger_dis) {
      dis_r(ZR);
    } else {
      myservo.write(angel_ZL);
    }
  }
  if (turn < 0) {
    if (R1 < denger_dis) {
      dis_r(ZR);
    } else if (L1< denger_dis) {
      dis_l(ZL1);
    } else {
      myservo.write(angel_ZR);
    }
  }

  currentMillis = millis();
   if (currentMillis - startMillis >= period) {
    Speed = 11;
    if (turn >= 0) {
      if ((R1 > 100 && F < 100 ) || (R1-L1>100 && F < 100) || (R1-R2 >100 && F < 100)  ){
        turn++;
        period = 1000;
        privosZ=abs(angleZ);
        myservo.write(41);
        while (abs(angleZ) < privosZ+90) {
          Serial.println(angleZ);
          mpu_loop();
        }
        myservo.write(center);
        startMillis = millis();
        Speed=11;
      }
    }  if (turn <= 0) {
      //  Serial.println(angleZ);
      if ( (L1 > 100 && F < 100) || (L1-R1>100 && F < 100) ||  (L1-L2 >100 && F < 100)  ||  (L2 > 100 && F < 100) ){
        turn--;
        startMillis = millis();
        period = 1000;
        //rate the gyro rate to get the yaw angle
        privosZ=abs(angleZ);
        myservo.write(131);
        while (abs(angleZ) < privosZ+90){
          // Serial.println(angleZ);
          mpu_loop();}
        myservo.write(center);
        startMillis = millis(); 
        Speed=11;
        ;
      }
    }
   }    //Serial.println();
   }
}
