//Take care of the imports and make sure they work!
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

//change those pins according to what is connected to the arduino
#define N1 12
#define N2 13
#define N3 7
#define N4 8
#define ENB1 11
#define ENB2 6

//PID controller parameters. NEED TO BE CHANGED
#define Kp  28
#define Kd  -1.19
#define Ki  167
#define sampleTime  0.001
#define targetAngle -7.5

MPU6050 mpu;

int16_t accX, accZ, gyroY;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  
  if(leftMotorSpeed >= 0) {
    analogWrite(ENB1, abs(leftMotorSpeed));
    digitalWrite(N1, LOW);
    digitalWrite(N2, HIGH);
  }
  else {
    analogWrite(ENB1, abs(leftMotorSpeed));
    digitalWrite(N1, HIGH);
    digitalWrite(N2, LOW);
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(ENB2, abs(rightMotorSpeed));
    digitalWrite(N3, HIGH);
    digitalWrite(N4, LOW);
  }
  else {
    analogWrite(ENB2, abs(rightMotorSpeed));
    digitalWrite(N3, LOW);
    digitalWrite(N4, HIGH);
  }
}

void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  Serial.begin(115200);
  // set the motor control and PWM pins to output mode
  pinMode(N1, OUTPUT);
  pinMode(N2, OUTPUT);
  pinMode(N3, OUTPUT);
  pinMode(N4, OUTPUT);
  pinMode(ENB1, OUTPUT);
  pinMode(ENB2, OUTPUT);
  
  
  // initialize the MPU6050 and set offset values
  mpu.initialize();

  //Those offsets need to be tuned
  mpu.setXAccelOffset(-300);
  /*mpu.setYAccelOffset(-95);
  mpu.setZAccelOffset(1099);
  mpu.setXGyroOffset(85);
  mpu.setYGyroOffset(9);
  mpu.setZGyroOffset(6);*/
  
  // initialize PID sampling loop
  init_PID();
}

void loop() {
  // read acceleration and gyroscope values
  accX = mpu.getAccelerationX();
  accZ = mpu.getAccelerationZ();  
  gyroY = mpu.getRotationY();
  
  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);
  Serial.print("motor power: ");
  Serial.println(motorPower);
  
}

// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accX, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  Serial.print("Angle: ");
  Serial.println(accAngle);
  Serial.print("acceleration: ");
  Serial.println(gyroY);
  
 // error = currentAngle - targetAngle;
  error = accAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime + Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;

}
