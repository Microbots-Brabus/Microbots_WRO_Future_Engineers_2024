#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <math.h>
#include <NewPing.h>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define TRIGGER_PIN_R1 43 // Ultra Sonic defines with these objects
#define ECHO_PIN_R1 A15
///////////////////////////////////////////////////////////
#define TRIGGER_PIN_L1 35
#define ECHO_PIN_L1 A13
////////////////////////////////////////////////////////////
#define TRIGGER_PIN_R2 45
#define ECHO_PIN_R2 A14
////////////////////////////////////////////////////////////
#define TRIGGER_PIN_L2 37
#define ECHO_PIN_L2 A12
////////////////////////////////////////////////////////////
#define TRIGGER_PIN_F 48
#define ECHO_PIN_F A9
////////////////////////////////////////////////////////////
#define TRIGGER_PIN_b 46
#define ECHO_PIN_b A8
#define MAX_DISTANCE 200
/////////////////////////////////////////////////////////////
NewPing sonar_R1(TRIGGER_PIN_R1, ECHO_PIN_R1, MAX_DISTANCE);
NewPing sonar_L1(TRIGGER_PIN_L1, ECHO_PIN_L1, MAX_DISTANCE);
NewPing sonar_R2(TRIGGER_PIN_R2, ECHO_PIN_R2, MAX_DISTANCE);
NewPing sonar_L2(TRIGGER_PIN_L2, ECHO_PIN_L2, MAX_DISTANCE);
NewPing sonar_F(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE);
NewPing sonar_b(TRIGGER_PIN_b, ECHO_PIN_b, MAX_DISTANCE);
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
int b;
///////////////////timer
unsigned long startMillis;  //variables for turning
unsigned long currentMillis;
unsigned long period = 0;
unsigned long timer = 0;

//////////////////Motor
int Speed = 4.5;
int center = 86;
//////////////////
int turn = 0;
int Target_AR;
int Target_AL;
int buttonPin=30;
int motorPin=22;

#define outputA 26
 #define outputB 28
 int counter = 0; 
 int aState;
 int aLastState; 
float last_encoder_pi;
float last_angle_pi;
unsigned long sleep;
float encoder_pi;
float angle_pi;
float doller_sin;

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
 void encoder(){
  aState = digitalRead(outputA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != aState) { 
       counter ++;
     } else {
       counter --;
     }
  //   Serial.print("Position: ");
    // Serial.println(counter);
   } 
   aLastState = aState;
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
  if (L1 == 0 ) { L1 = 200; }
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
   b = sonar_b.ping_cm();
  if (b == 0) { b = 200; }
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
  Serial.print("b=");
  Serial.println(b);
   
  if (L1 >200) { L1 = sonar_L1.ping_cm();
  delayMicroseconds(100); }
  
  if (L2 >200) { L2 = sonar_L1.ping_cm(); 
  delayMicroseconds(100);}
 
  if (R1 >200) { R1 = sonar_L1.ping_cm();
  delayMicroseconds(100); }

  if (R2 >200) { R2 = sonar_L1.ping_cm();
  delayMicroseconds(100); }
if (F>200) { F = sonar_L1.ping_cm();
delayMicroseconds(100); }

 if (b >200) { b = sonar_L1.ping_cm();
 delayMicroseconds(100); }
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
//bool on = 1;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Stop() {
  ESC.write(0);
 
  delay(10000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000);
}
void turn_Right(){
         if ((abs((L1+L2)/2)>50)){
         while(F > 20) {
         sensor();
         }
        ESC.write(0);
        privosZ=abs(angleZ);
        digitalWrite(motorPin,LOW);
        myservo.write(131);
        currentMillis = millis();
      startMillis = millis();
      while (currentMillis - startMillis <= 250){
         mpu_loop();
        currentMillis = millis();
      }
        ESC.write(5);
        while (abs(angleZ) < privosZ+90) {
          Serial.println(angleZ);
          mpu_loop();

        }
         myservo.write(center);
        while(b>20 && b<200){
            sensor();
        }
        ESC.write(0);
        digitalWrite(motorPin,HIGH);
        myservo.write(center);
      currentMillis = millis();
      startMillis = millis();
      while (currentMillis - startMillis <= 750){
         mpu_loop();
        //read data note...
        currentMillis = millis();
      }
        ESC.write(5);
        turn++;
        period = 4000;
      }
      else {
        privosZ=abs(angleZ);
        myservo.write(41);
        while (abs(angleZ) < privosZ+90) {
          Serial.println(angleZ);
          mpu_loop();
        }
        myservo.write(center);
        ESC.write(0);
        digitalWrite(motorPin,LOW);
      currentMillis = millis();
      startMillis = millis();
      while (currentMillis - startMillis <= 250){
         mpu_loop();
        currentMillis = millis();
      }
        ESC.write(5);
         while(b>20 && b<200){
            sensor();
        }
        ESC.write(0);
        digitalWrite(motorPin,HIGH);
        currentMillis = millis();
      startMillis = millis();
      while (currentMillis - startMillis <= 750){
         mpu_loop();
           //read data note...
        currentMillis = millis();
      }
        ESC.write(5);
        Speed = 5;
        turn++;
        period = 4000;
      }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void turn_Left(){
   if ((abs((R1+R2)/2)>50)){
         while(F > 20) {
         sensor();
         }
        ESC.write(0);
        privosZ=abs(angleZ);
        digitalWrite(motorPin,LOW);
        myservo.write(41);
         currentMillis = millis();
         startMillis= millis();
      while (currentMillis - startMillis <= 300){
            mpu_loop();
            currentMillis = millis();
            }
         ESC.write(5);
        while (abs(angleZ) < privosZ+90) {
          Serial.println(angleZ);
          mpu_loop();
        }
        myservo.write(center);
         while(b>20 && b<200){
            sensor();
        }

        ESC.write(0);
         digitalWrite(motorPin,HIGH);
        myservo.write(center);
        currentMillis = millis();
         startMillis= millis();
      while (currentMillis - startMillis <= 750){
            mpu_loop();
              //read data note...
            currentMillis = millis();
            }

        ESC.write(5);
        turn--;
        period = 4000;
      }
      else {
      privosZ=abs(angleZ);
        myservo.write(131);
        while (abs(angleZ) < privosZ+90) {
          Serial.println(angleZ);
          mpu_loop();
        }
        myservo.write(center);
        ESC.write(0);
        digitalWrite(motorPin,LOW);
      currentMillis = millis();
      startMillis = millis();
      while (currentMillis - startMillis <= 250){
         mpu_loop();
        currentMillis = millis();
      }
        ESC.write(5);
         while(b>20 && b<200){
            sensor();
        }
        ESC.write(0);
        digitalWrite(motorPin,HIGH);
        currentMillis = millis();
      startMillis = millis();
      while (currentMillis - startMillis <= 750){
         mpu_loop();
        //read data note...
        currentMillis = millis();
      }
        ESC.write(5);
        Speed = 5;
        turn++;
        period =4000;
      }
      }

  void MoveSabbagh(){
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
      }

  void obstacle(){
        mpu_loop();
    if (counter>=90){
      ESC.write(0);
      currentMillis = millis();
      sleep = millis();
      while (currentMillis - sleep <= 1000){
            mpu_loop();
            currentMillis = millis();
             String data = Serial.readStringUntil(10);
    Serial.println(data);
    int commaIndex1 = data.indexOf(',');
    int commaIndex2 = data.indexOf(',', commaIndex1 + 1);
      
       encoder_pi = data.substring(0, commaIndex1).toFloat();
       angle_pi = data.substring(commaIndex1 + 1, commaIndex2).toFloat();
   //   Serial.println(encoder_pi);
    //  Serial.println(angle_pi);
      encoder_pi *=2;
            }
      counter=0;
        }
         else{ 
         if ( (last_encoder_pi == encoder_pi) && (last_angle_pi = angle_pi) ){
      mpu_loop();
      ESC.write(0);
      }
      else {
      mpu_loop();
      ESC.write(5);
        privosZ=abs(angleZ);
        counter=0; 
         Serial.println("If 3:");
      while (counter< encoder_pi ){
        myservo.write(center - angle_pi);
          while (abs(angleZ) < abs (privosZ+angle_pi)){
            mpu_loop();
            encoder();
             Serial.println("while 1:");
            }
          myservo.write(center);
          while (counter< encoder_pi ){
             Serial.println("while 2:");
            encoder();
        }
      }
        myservo.write(center+angle_pi);
          while (abs(angleZ) > privosZ){
             Serial.println("while 3:");
            mpu_loop();
            }
        myservo.write(center);
      last_encoder_pi = encoder_pi ;
      last_angle_pi = angle_pi ; 
     }
   }
  }

void setup() {
  pinMode(motorPin,OUTPUT);
  digitalWrite(motorPin,HIGH);
  pinMode (outputA,INPUT);
  pinMode (outputB,INPUT);  
  Serial.begin(115200);
  ESC.attach(9, 1000, 2000);
  ESC.write(0);
  myservo.attach(3);
  mpu_setup();
  myservo.write(center);
  startMillis = millis();
  while(!digitalRead(buttonPin)){
    mpu_loop();
  }
  Serial.println("Done!\n");
}


void loop() {
  mpu_loop();
     if (Serial.available() > 0) {
    String data = Serial.readStringUntil(10);
    //serial.println(data);

    int commaIndex1 = data.indexOf(',');
      int commaIndex2 = data.indexOf(',', commaIndex1 + 1);
      
      float encoder_pi = data.substring(0, commaIndex1).toFloat();
      float angle_pi = data.substring(commaIndex1 + 1, commaIndex2).toFloat();
   //   Serial.println(encoder_pi);
    //  Serial.println(angle_pi);
      encoder_pi *=2;

   if (abs(turn)==12 ){
    ESC.write(Speed);
    sensor();
    if (b>150){
    delay(200);
    Speed=0;
    }
  
   } 
   else{
    if (encoder_pi==0&&angle_pi==0){
   MoveSabbagh();
   currentMillis = millis();
   if (currentMillis - startMillis >= period) {
       Speed = 5;
   if (turn >= 0) {
    if ((R1 > 100 && F < 100 ) || (R1-L1>100 && F < 100) || (R1-R2 >100 && F < 100)  ){
         myservo.write(center);
         turn_Right();
      
    }}  if (turn <= 0) {
      if ( (L1 > 100 && F < 100) || (L1-R1>100 && F < 100) ||  (L1-L2 >100 && F < 100)  ||  (L2 > 100 && F < 100) ){
      myservo.write(center);
      turn_Left();
        
    }
  }    //Serial.println();
   
   }

  }
else {
  doller_sin=abs(encoder_pi*sin(angle_pi));
  if(angle_pi>0){
    if(doller_sin<((R1+R2))/2){
      obstacle();
    }
    else{
      encoder_pi=0;
      angle_pi=0;
       if (encoder_pi==0&&angle_pi==0){
   MoveSabbagh();
   currentMillis = millis();
   if (currentMillis - startMillis >= period) {
       Speed = 5;
   if (turn >= 0) {
    if ((R1 > 100 && F < 100 ) || (R1-L1>100 && F < 100) || (R1-R2 >100 && F < 100)  ){
         myservo.write(center);
         turn_Right();
      
    }}  if (turn <= 0) {
      if ( (L1 > 100 && F < 100) || (L1-R1>100 && F < 100) ||  (L1-L2 >100 && F < 100)  ||  (L2 > 100 && F < 100) ){
      myservo.write(center);
      turn_Left();
        
    }
  }    //Serial.println();
   
   }

  }
    }
  }

  else if(angle_pi<0){
    if(doller_sin<((L1+L2))/2){
      obstacle();
    }
else{
      encoder_pi=0;
      angle_pi=0;
       if (encoder_pi==0&&angle_pi==0){
   MoveSabbagh();
   currentMillis = millis();
   if (currentMillis - startMillis >= period) {
       Speed = 5;
   if (turn >= 0) {
    if ((R1 > 100 && F < 100 ) || (R1-L1>100 && F < 100) || (R1-R2 >100 && F < 100)  ){
         myservo.write(center);
         turn_Right();
      
    }}  if (turn <= 0) {
      if ( (L1 > 100 && F < 100) || (L1-R1>100 && F < 100) ||  (L1-L2 >100 && F < 100)  ||  (L2 > 100 && F < 100) ){
      myservo.write(center);
      turn_Left();
        
    }
  }    //Serial.println();
   
   }

  }
    }
}
 
                                 }
      }} }