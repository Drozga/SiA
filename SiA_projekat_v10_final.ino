/*
 *Projekat iz predmeta: Senzori i Aktuatori 
 *Tema projekta: 
 
                 Pomoću senzora MPU6050 izvršiti grupu merenja prilikom kretanja autića. 
                 Takođe proračunati preciznost senzora MPU6050.
               
 *Mentor: Dr Jovan Bajić
 *Projekat izradili:
    Đorđe Drozgović EE43/2018 
    Filip Popadić EE45/2019
    Datum: 25.07.2023.
*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <MPU6050.h>
#include <SimpleKalmanFilter.h>
#include <math.h>
#include <NewPing.h>                  // Biblioteka za ultrazvučne senzore, koristi se za merenje udaljenosti
#include <Adafruit_MPU6050.h>        // Adafruit biblioteka za senzor MPU6050
#include <Adafruit_Sensor.h>        // Adafruit senzorska biblioteka za različite senzore
#include <SoftwareSerial.h>        // Biblioteka za serijsku komunikaciju putem softverskog serijskog porta
#include <Wire.h>                 // Biblioteka za I2C komunikaciju
Adafruit_MPU6050 mpu;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime1, currentTime1, previousTime1;
int c = 0;

#define OUTPUT_READABLE_YAWPITCHROLL
#define trigPin 11
#define echoPin 12
#define trigPin1 13
#define echoPin1 2
#define MAX_DISTANCE 800

NewPing sonar(trigPin, echoPin, MAX_DISTANCE);
NewPing sonar1(trigPin1, echoPin1, MAX_DISTANCE);
SoftwareSerial Genotronex(9, 10);
SimpleKalmanFilter simpleKalmanFilter(0.5, 0.5, 0.03);

const int leftSpeed = 5;
const int pogonskiPWM = 6;
const int left1 = 7;
const int left2 = 8;
const int pogonski1 = 3;
const int pogonski2 = 4;

int zastava = 0;
int zastava2 = 0;
int zastava3 = 0;
int zastava4 = 0;
int BluetoothData;

const float targetAcceleration = 3;

float targetGyro = 80;
float duration1, distance1;
float duration2, distance2;
float trenutnaBrzina = 0.0;
float trenutniPut = 0.0;
float trenutniUgao = 0.0;
float correct;
float  novaX = 0.0;
float  novaY = 0.0;
float deltaX = 0;
float deltaY = 0;
float yaw2 = 0;
float trPut = 0;
float distanca = 0;
float estimated_value, estimated_value1;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  calculate_IMU_error();
  delay(20);
  
  Genotronex.begin(9600);
  Genotronex.println("");

  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(pogonski1, OUTPUT);
  pinMode(pogonski2, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(pogonskiPWM, OUTPUT);

  digitalWrite(pogonski1, LOW);
  digitalWrite(pogonski2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, LOW);
  analogWrite(pogonskiPWM, 0);
  analogWrite(leftSpeed, 0);

}

void loop() {

 //=== Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime1 = currentTime1;        // Previous time is stored before the actual time read
  currentTime1 = millis();            // Current time actual time read
  elapsedTime1 = (currentTime1 - previousTime1) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ - 1.187; //+ 0.79; // GyroErrorZ ~ (-0.8) -1.19
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime1; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime1;

  estimated_value1 = simpleKalmanFilter.updateEstimate(GyroZ);
  yaw =  yaw + estimated_value1 * elapsedTime1;
  AccY = (AccY + 0.017) * 9.81;

    ocitaj1();
    ocitaj2();  
if(zastava4 < 5){ // stani nakon 4 skretanja
     
        if (distance1 > 25 && zastava3 != 1 && zastava4 != 4) {  
      idi_pravo1();
      } 
          else if (distance1 > 10 && distance1 < 24) { 
          zastava3 = 1;
          } else if (zastava3 == 1) {
              if (yaw < targetGyro) {
                levo();
               }
                else{
                    distanca = trPut;                                                 
                     trPut += trenutniPut;                                           
                     yaw2 = radians(yaw);
                     deltaY = distanca * cos(yaw2);
                     deltaX = distanca * sin(yaw2);
                       novaX += deltaX;
                       novaY += deltaY;
                                                  
                       stani();
                       zastava4++;
                       zastava3 = 0;
                       targetGyro += 90;
                                                      
                       ispis("Kord Y: ",novaY);
                       ispis("Kord X: ",novaX);
                       ispis("PrPut: ", trPut);
                       ispis("yaw:", yaw);

//                       trenutniPut = 0;
//                       trenutnaBrzina = 0.23; 
            } 
        }
    }
} 
//////////////////////////////////////////////////////////////////////////////////////////////
                                           //--FUNKCIJE--
//////////////////////////////////////////////////////////////////////////////////////////////
void idi_pravo1() {  
  if (AccY < targetAcceleration) {
    pravo();
       trenutnaBrzina += AccY * elapsedTime1;
       trenutniPut += trenutnaBrzina * elapsedTime1;
  } else {   
    stani();
    delay(600);
  }
}
void ocitaj1() {
  duration1 = sonar.ping();
  distance1 = (duration1 / 2) * 0.0343;
}
void ocitaj2() {
  duration2 = sonar1.ping();
  distance2 = (duration2 / 2) * 0.0343;
}
void enableBluetooth() {
  while (Genotronex.available()) {
    BluetoothData = Genotronex.read();
    if (BluetoothData == '1') {
      zastava2 = 1;
      break;
    } else if (BluetoothData == '0') {
      zastava2 = 0;
      break;
    }
  }
}
void ispis(String tekst, float vrednost) {
  Genotronex.print(tekst);
  Genotronex.print(vrednost);
  Genotronex.println();
}
void stani(){
  digitalWrite(pogonski1, LOW);
  digitalWrite(pogonski2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, LOW);
  analogWrite(pogonskiPWM, 0);
  analogWrite(leftSpeed, 0);
}
void levo(){ 
  digitalWrite(pogonski1, LOW);
  digitalWrite(pogonski2, HIGH);
  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
  analogWrite(pogonskiPWM, 170);
  analogWrite(leftSpeed, 170);
}
void pravo(){
  digitalWrite(pogonski1, LOW);
  digitalWrite(pogonski2, HIGH);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
  analogWrite(pogonskiPWM, 120);
  analogWrite(leftSpeed, 120);
}
void calculate_IMU_error() {
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}
