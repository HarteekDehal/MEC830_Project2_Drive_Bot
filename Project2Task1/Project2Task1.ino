#include <Wire.h>
#include <PID_v1.h>

#define trigPin 13
#define echoPin 12

// Define motor control pins
const int lmotorPWM = 3;   // PWM speed control for the right motor
const int lmotorDir1 = 6;  // Direction control 1 for the right motor
const int lmotorDir2 = 5; // Direction control 2 for right the motor
const int rmotorPWM = 11;   // Direction control 1 for left the motor
const int rmotorDir1 = 10;  // Direction control 2 for the left motor
const int rmotorDir2 = 9; // PWM speed control for the left motor


int counter = 0; //Counter to determine turns



const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; //linear acceleration
float GyroX, GyroY, GyroZ; //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
float angle; 


void setup() {

    Serial.begin(9600);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
  calibrate();
  delay(20);
  
  // Set motor control pins as OUTPUT
  pinMode(rmotorPWM, OUTPUT);
  pinMode(rmotorDir1, OUTPUT);
  pinMode(rmotorDir2, OUTPUT);
  pinMode(lmotorPWM, OUTPUT);
  pinMode(lmotorDir1, OUTPUT);
  pinMode(lmotorDir2, OUTPUT);
  Serial.begin(9600); // Initialize serial communication
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {

  readAcceleration();
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; //AccErrorX is calculated in the calculateError() function
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;
  
  // === Read gyroscope (on the MPU6050) data === //
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds
  readGyro();
  // Correct the outputs with the calculated error values
  GyroX -= GyroErrorX; //GyroErrorX is calculated in the calculateError() function
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;
  //combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined through trial and error by other people
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  angle = -roll;



  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reading the echo pulse duration
  long duration = pulseIn(echoPin, HIGH);

  // Calculating distance in centimeters
  int distance = duration * 0.034 / 2;



    if (distance < 10) {
    Serial.print("Object detected at ");
    Serial.print(distance);
    Serial.println(" cm");
          

      turnRight();

    


    
    
    }
  else {
    Serial.println("No object detected");
     moveForward();
  }

    
  
}









void moveForward() {
  digitalWrite(rmotorDir1, HIGH);
  digitalWrite(rmotorDir2, LOW);
  analogWrite(rmotorPWM, 50); // Adjust PWM value for desired speed
 digitalWrite(lmotorDir1, HIGH);
 digitalWrite(lmotorDir2, LOW);
  analogWrite(lmotorPWM, 65); // Adjust PWM value for desired speed
}

void moveBackward() {
  digitalWrite(rmotorDir1, LOW);
  digitalWrite(rmotorDir2, HIGH);
  analogWrite(rmotorPWM, 60); // Adjust PWM value for desired speed
   digitalWrite(lmotorDir1, LOW);
  digitalWrite(lmotorDir2, HIGH);
  analogWrite(lmotorPWM, 60); // Adjust PWM value for desired speed
}

void stopMotor() {
  digitalWrite(rmotorDir1, LOW);
  digitalWrite(rmotorDir2, LOW);
  analogWrite(rmotorPWM, 0);
    digitalWrite(lmotorDir1, LOW);
  digitalWrite(lmotorDir2, LOW);
  analogWrite(lmotorPWM, 0);
}

void turnRight() {
  digitalWrite(rmotorDir1, LOW);
  digitalWrite(rmotorDir2, HIGH);
  analogWrite(rmotorPWM, 60); // Adjust PWM value for desired speed
}

void turnLeft() {
 digitalWrite(lmotorDir1, LOW);
  digitalWrite(lmotorDir2, HIGH);
  analogWrite(lmotorPWM, 60); // Adjust PWM value for desired speed
}


void calibrate() {
  // Read accelerometer values 200 times
  c = 0;
  while (c < 200) {
    readAcceleration();
    // Sum all readings
    AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
    c++;
  }
  //Divide the sum by 200 to get the error value, since expected value of reading is zero
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    readGyro();
    // Sum all readings
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  Serial.println("The the gryoscope setting in MPU6050 has been calibrated");
}
void readAcceleration() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the MPU6050 datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
}
void readGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}
