
#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include "Adafruit_VL53L0X.h"
#include <math.h>
//________________________________________________________________________________________________________

#define LOX1_ADDRESS 0x35
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x20
#define LOX4_ADDRESS 0x30
#define LOX5_ADDRESS 0x39

// set the pins to shutdown
#define SHT_LOXRight1 22
#define SHT_LOXRight2 24
#define SHT_LOXFront3 26
#define SHT_LOXLeft4 28
#define SHT_LOXLeft5 30

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox5 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;
VL53L0X_RangingMeasurementData_t measure5;


Servo ESC;
Servo myservo;
MPU6050 mpu;

//________________________________________________________________________________________________________

unsigned long startMillis;  //variables for turning
unsigned long currentMillis;
unsigned long period = 0; 

unsigned long timer = 0;
int Speed = 15;
int buttonPin = 52;
// Define ultrasonic pins (you can add more if needed)

double target = 0; // Set your desired orientation angle (adjust as needed)
double z;

const int calibrationSamples = 1000; //Gyro
float gyroZOffset = 0;
float angleZ = 0;

double d_tof_r1;   //tof 
double d_tof_r2;  
double d_tof_L1;
double d_tof_L2;
double d_tof_f;
double R1,;
double R2;
double L1;
double L2;
double F;
double x1;
double y1;
double defL1R2;


double error ;
double teta  ;
double err; // PID parameters (tune these values)
double kp = 2 ;
double ki = 0.1*0;
double kd = 0.01*0;
double kps = 2 ;

double prevErr = 0;
double integral = 0;
 
int n; 


//________________________________________________________________________________________________________

void mpu_setup() {
  
  mpu.initialize();
  // Verify the connection
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  calibrateGyro();
}

//________________________________________________________________________________________________________
  void mpu_loop() {
  
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Convert milliseconds to seconds
  lastTime = currentTime;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Gyro rate for Z-axis (in degrees per second) with offset correction
  float gyroRateZ = (gz - gyroZOffset) / 131.0;

  // Integrate the gyro rate to get the yaw angle
  angleZ += gyroRateZ * dt;

  // Apply a simple condition to stabilize the angle when not rotating
  if (abs(gyroRateZ) < 0.1) { // Assuming small drift values, can be adjusted
    gyroRateZ = 0; // Zero out the small drift rate
  }
   
   z = angleZ ;
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
   err = target - z;
    integral += err;
    double derivative = err - prevErr;
    // Calculate the control output (PID)
    double output = kp * err + ki * integral + kd * derivative;
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
//________________________________________________________________________________________________________
/*
int P_id_tof(int x , int x1) {  
  error = atan ((x-x1)/125);
  teta = (error*180.0) / PI ;
  Serial.print("teta = ");
  Serial.println(error);
  target+= teta-err; 
  delay(1000);
  // Proportional term
  //float proportional = kps * error;
  // PID output
  // float output = proportional ;
  // Map PID output to servo range (0-180 degrees)
  //int servo_position = map(output, -1000 , 1000 , 68 , 108 );
  // Move the servo to the calculated position
  // myservo.write(servo_position);
  //return error ; 
  }*/
//________________________________________________________________________________________________________

void setID() {
  // all reset

//SHT_LOXRight1 SHT_LOXRight2 SHT_LOXFront3  SHT_LOXLeft4 SHT_LOXLeft5 30


  digitalWrite(SHT_LOXRight1, LOW);    
  digitalWrite(SHT_LOXRight2, LOW);
  digitalWrite(SHT_LOXFront3, LOW);
  digitalWrite(SHT_LOXLeft4, LOW);
  digitalWrite(SHT_LOXLeft5, LOW);

  delay(10);
  // all unreset
  digitalWrite(SHT_LOXRight1, HIGH);
  digitalWrite(SHT_LOXRight2, HIGH);
  digitalWrite(SHT_LOXFront3, HIGH);
  digitalWrite(SHT_LOXLeft4, HIGH);
  digitalWrite(SHT_LOXLeft5, HIGH);

  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOXRight1, HIGH);
  digitalWrite(SHT_LOXRight2, LOW);
  digitalWrite(SHT_LOXFront3, LOW);
  digitalWrite(SHT_LOXLeft4, LOW);
  digitalWrite(SHT_LOXLeft5, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOXRight2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
   delay(10);
   digitalWrite(SHT_LOXFront3, HIGH);

   if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while(1);
  }
  delay(10);

    digitalWrite( SHT_LOXLeft4, HIGH);

   if(!lox4.begin(LOX4_ADDRESS)) {
    Serial.println(F("Failed to boot four VL53L0X"));
    while(1);
  }
  delay(10);
   digitalWrite( SHT_LOXLeft5, HIGH);

   if(!lox5.begin(LOX5_ADDRESS)) {
    Serial.println(F("Failed to boot fifth VL53L0X"));
    while(1);
  }
  delay(10);
  lox1.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  lox2.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  lox3.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  lox4.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  lox5.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);


 
}
//________________________________________________________________________________________________________


double read_tof_R2(){
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout! 
   
   // print sensor one reading
   d_tof_r2 = measure2.RangeMilliMeter;
  Serial.print(F("right 2 : "));
  if(measure2.RangeStatus != 4) {     // if not out of range
    Serial.print(measure2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print(F(" "));
  return d_tof_r2;

}
//________________________________________________________________________________________________________
double read_tof_f() {
  lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
  // print sensor two reading
   d_tof_f = measure3.RangeMilliMeter;
  Serial.print(F("f : "));
  if(measure3.RangeStatus != 4) {
    Serial.print(measure3.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.println();
  return d_tof_f;
}
//________________________________________________________________________________________________________
double read_tof_L1() {
  lox4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!
  // print sensor two reading
   d_tof_L1 = measure4.RangeMilliMeter;
  Serial.print(F("left 1 : "));
  if(measure4.RangeStatus != 4) {
    Serial.print(measure4.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.println();
  return d_tof_L1;
  }
//________________________________________________________________________________________________________

double read_tof_L2() {
  lox5.rangingTest(&measure5, false); // pass in 'true' to get debug data printout!

  // print sensor two reading
   d_tof_L2 = measure5.RangeMilliMeter;
  Serial.print(F("left 2 : "));
  if(measure5.RangeStatus != 4) {
    Serial.print(measure5.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.println();
  return d_tof_L2;

}

//________________________________________________________________________________________________________


double read_tof_R1() {
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!

  // print sensor two reading
   d_tof_r1 = measure1.RangeMilliMeter;
  Serial.print(F("right1 : "));
  if(measure1.RangeStatus != 4) {
    Serial.print(measure1.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.println();
  return d_tof_r1;
}

//________________________________________________________________________________________________________

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pinMode(buttonPin , INPUT);
    ESC.attach(9, 1000, 2000);
    ESC.write(0);
    myservo.attach(13);
    myservo.write(88);
    mpu_setup();
    Serial.println("Done!\n");
    delay(1000);
    startMillis = millis(); 

  while (! Serial) { delay(1); }

  pinMode(SHT_LOXRight1, OUTPUT);
  pinMode(SHT_LOXRight2, OUTPUT);
  pinMode(SHT_LOXFront3, OUTPUT);
  pinMode(SHT_LOXLeft4, OUTPUT);
  pinMode(SHT_LOXLeft5, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOXRight1, LOW);
  digitalWrite(SHT_LOXRight2, LOW);
  digitalWrite(SHT_LOXFront3, LOW);
  digitalWrite(SHT_LOXLeft4, LOW);
  digitalWrite(SHT_LOXLeft5, LOW);
  

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
  setID();
}

void ultra_sonic_R(){

digitalWrite(trig_R , LOW);
delayMicroseconds(2);
digitalWrite(trig_R , HIGH);
delayMicroseconds(10);
digitalWrite(trig_R , LOW);
duration_R = pulsln(echo_R , HIGH)
distance_R = (duration_R *.0343)/2 ;
Serial.print("Right =");
Serial.print(distance_R);


}

void ultra_sonic_L(){

digitalWrite(trig_L , LOW);
delayMicroseconds(2);
digitalWrite(trig_L , HIGH);
delayMicroseconds(10);
digitalWrite(trig_L , LOW);
duration_L = pulsln(echo_L , HIGH)
distance_L = (duration_L *.0343)/2 ;
Serial.print("Left =");
Serial.print(distance_L);
delay(100);
}
//________________________________________________________________________________________________________

void loop() {
    ESC.write(Speed);
    mpu_loop();
    P_id_gyro();
    R1=read_tof_R1(); 
    R2=read_tof_R2(); 
    L1=read_tof_L1(); 
    L2=read_tof_L2(); 
    F=read_tof_f();

    R11=read_tof_R1(); 
    R22=read_tof_R2(); 
    L11=read_tof_L1(); 
    L22=read_tof_L2(); 
    F=read_tof_f();
    R11=R1
    R111=R11
   // defL1R2=abs(L1-R2);
   currentMillis = millis();
   if (currentMillis - startMillis >= period){
        Speed=13;
    if( (err < 10) && (err > -10) ){
        if ( (R2-R1>700)  ((R1>2000) && (R2>2000))  ((R1>1000)&&(F<300)) ){
          startMillis = millis();
          period =1500 ;
           target -= 90 ;
           Speed=29; }
    //   else if ( (F<=700)&&(L1>1200) ){
    //   target += 88 ; }
    } 
    }

     if( (err < 25) && (err > -25) ){
      if((target<=-88)) {
      error = atan ((L1-L2)/125);
  teta = (error*180.0) / PI ;
  // Serial.print("teta = ");
  // Serial.println(teta);
  target+= teta-err;
      }
}
}
