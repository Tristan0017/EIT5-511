#define tINT 2            // Total number of interrupts
#define pingDelay 50      // How many milliseconds between each measurement ; keep > 5ms
#define debugDelay 200    // How many milliseconds between each Serial.print ; keep > 200ms
#define soundSpeed 343.0  // Speed of sound in m/s
#include <math.h>
#include <Servo.h>
#include <krnl.h>
#include <arduino.h>

// Inputs/Outputs
int EchoLeft    = 2;  // Left Echo Pin
int EchoRight   = 3;  // Right Echo Pin
int triggerPin  = 4;  // Ultralyd Trigger Pin
int infraredPin = 5;  // Infrared Pin
int DTEn        = 6;  //Datatransfer enable pin
int servoPin    = 7;  // Servo Data Pin
int PWM2        = 8;  // DC motor PWM
int InB2        = 9;  // 2INB
int InA2        = 10; // 2INA
int en1         = 11; // Enable 1
int en2         = 12; // Enable 2
int reader      = 13; // Encoder reader Pin

// Infrared
unsigned long previousTime = 0;
unsigned long currentTime = millis();
int Infrared_State = 0;
int lastSeen;
boolean irlast = 0;
bool irSeen;
const unsigned long eventInterval = 250;  // ms before stopping when the leader vehicle is not visible.

// Stop Varaible
boolean StopState = 0; //If 1 it will start is stop state if 0 it will start in drive state.

// Reference values
float ReferenceDistance = 60, PWMinput, Error;

// Values used for the motor transfer function
float result, yn, xn, xn1 = 0, yn1 = 0, T = 0.0033;
float c = 15, pi = 3.14159265359, MiddleLine; // Dette er er sidelængder på trekanten
float kp = 62.165, ki = 181.30;
float km = 1; //14.125 <- Theoretical turning variable - bigly
float z; //output from the transfer function

// Kernel setup
struct k_t *p1, *p2, *p3, *sem1, *sem2, *sem3;

// Ultrasonic sensors
volatile unsigned long travelTime[tINT];  // Place to store traveltime of the pusle
volatile unsigned long startTime[tINT];   // Place to store ping times (interrupt)
float lastDistance;
float distance[tINT];                     // Calculated distances in cm
const int averageLength = 5;
int averageIndex = 0;
float totalL = 0, totalR = 0;
int runningAverageRegL[averageLength];
int runningAverageRegR[averageLength];
unsigned long lastPollMillis;
unsigned long lastDebugMillis;
float A, D, D_Rad, d = 0, B, C, MeasDistance, MAFL, MAFR; // Triangle values.
float x, y = 0;

// Encoder
int counter = 0;
int currentState = 0;
int lastState = 0;
float measureTime = 0.01; // Encoder measure time in seconds
float distancePrTick = 0.004188842565 ; //Distance travelled in one encoder interrupt.
float velocity = 0; // Meter / seconds

// Motor inputs
int PWM1_val; // Servo motor
int PWM2_val; // PM DC motor

// Create a servo object
Servo Servo1;

void setup()
{
  Serial.begin(9600);
  while (!Serial) {
    ;
  }


  pinMode(reader, INPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(infraredPin, INPUT);
  lastState = digitalRead(reader);
  digitalWrite(DTEn, HIGH);
  Servo1.attach(servoPin);
  pinMode(triggerPin, OUTPUT);

  // Manage interrupt pins here
  pinMode(EchoLeft, INPUT);    // Set interrupt pin 2 (INT0) as INPUT (sensor 1)
  pinMode(EchoRight, INPUT);    // Set interrupt pin 3 (INT1) as INPUT (sensor 2)
  attachInterrupt(0, call_INT0, CHANGE );   // ISR for INT0
  attachInterrupt(1, call_INT1, CHANGE );   // ISR for INT1

  lastPollMillis = millis();
  lastDebugMillis = millis();

  k_init(3, 3, 5);
  // Tasks
  p1 = k_crt_task(MeasureDistanceCaclulateAngle, 30, 200);
  p2 = k_crt_task(DataTransfer, 20, 200);
  p3 = k_crt_task(Infrared, 10, 200);

  // Semaphores
  sem1 = k_crt_sem(0, 1);
  sem2 = k_crt_sem(0, 1);
  sem3 = k_crt_sem(0, 1);

  k_start(1); // start kernel with tick speed 1 milli seconds
}
void loop() {}

void Infrared() {
  k_set_sem_timer(sem3, 1);
  while (1) {
    k_wait(sem3, 0);
    if (digitalRead(infraredPin) != irSeen) {
      irSeen = digitalRead(infraredPin);
      lastSeen = 0;
    }
    currentTime = millis();
    if (irSeen == 1) {
      if (currentTime - previousTime >= eventInterval) {
        Infrared_State = 1;
      }
    }
    if (irSeen == 0) {
      previousTime = currentTime;
      Infrared_State = 0;
    }
  }
  k_sleep(100);
  k_signal(sem3);
}

void DataTransfer() {
  k_set_sem_timer(sem2, 1);
  while (1) {
    k_wait(sem2, 0);
    Serial.print(" Wheel angle: ");
    Serial.print(D * km);
    Serial.print(" degrees");
    Serial.print(" Velocity: ");
    Serial.print(velocity);
    Serial.print(" m/s");
    Serial.print(" Distance: ");
    Serial.print(MiddleLine / 100);
    Serial.println(" meters");
    k_sleep(100);
    k_signal(sem2);
  }
}

void MeasureDistanceCaclulateAngle() {
  k_set_sem_timer(sem1, 3.3);
  while (1) {
    k_wait(sem1, 0);
    if (Infrared_State == 1)
    {
      digitalWrite(InA2, LOW);
      digitalWrite(InB2, LOW);
      PWM2_val = 0;
      analogWrite(PWM2, PWM2_val);
      if (irlast == 0)
      {
        digitalWrite(InA2, LOW);
        digitalWrite(InB2, HIGH);
        PWM2_val = 255;
        analogWrite(PWM2, PWM2_val);
        k_eat_ticks(velocity * 3.6);
        irlast = 1;
      }
      digitalWrite(InA2, LOW);
      digitalWrite(InB2, LOW);
    }
    if (Infrared_State == 0) {
      irlast = 0;
      doMeasurement();
      lastPollMillis = millis();
      for (int i = 0; i < tINT; i++) {
        if (i == 0) {
          distance[i] = totalL / averageLength;
        }
        else {
          distance[i] = totalR / averageLength;
        }
      }
      doAngleMath();
      if (MeasDistance > (velocity * 0.48816 * 3.6 + 25))
        StopState = 0;
      else if (MeasDistance <= (velocity * 0.48816 * 3.6 + 25)) {
        StopState = 1;
        Stop(velocity);
      }
      Encoder();
      Error = MeasDistance - ReferenceDistance;
      PWMinput = MotorTransferFunction(Error / 100) * 41.07 + 61.6;
      Drive(PWMinput);
      lastDebugMillis = millis();
      k_signal(sem1);
    }
  }
}

void Drive(float PWM) {
  if (StopState == 0) {
    digitalWrite(en1, HIGH);
    digitalWrite(en2, HIGH);
    digitalWrite(InA2, HIGH);
    digitalWrite(InB2, LOW);

    if (PWM > 123)     // PM DC Motors output is cutted to half
      PWM2_val = 123;

    if (PWM < 0)
      PWM2_val = 0;

    if (PWM < 123 && PWM > 0)
      PWM2_val = PWM;

    analogWrite(PWM2, PWM2_val);
  }
}

float MotorTransferFunction(float xn) {
  yn = xn * kp - xn1 * kp + xn1 * ki * T + yn1; // PM DC Motor controller

  xn1 = xn;
  if (yn > 6.2)
    yn = 6.2;

  if (yn < -6.2)
    yn = -6.2;

  yn1 = yn;

  return yn;
}

void Encoder() {
  unsigned long tid = millis();
  while (tid + (measureTime * 1000) > millis()) {
    currentState = digitalRead(reader);
    if (currentState == 1 && lastState == 0) {
      counter++;
    }
    lastState = currentState;
  }
  velocity = distancePrTick * counter / measureTime;
  counter = 0;
}

void doAngleMath() {
  float b = distance[1];
  float a = distance[0];
  if (a < (b * 1.5) && a > (b * 0.5) && b < (a * 1.5) && b > (a * 0.5)) {
    A = acos(((b * b) + (c * c) - (a * a) ) / ( 2 * b * c));

    MiddleLine = sqrt((b * b) + (c / 2 * c / 2) - (2 * b * c / 2 * cos(A)));
    if (!isnan(MiddleLine))
      MeasDistance = MiddleLine;

    D_Rad = acos(((MeasDistance * MeasDistance) + (c / 2 * c / 2) - (b * b) ) / ( 2 * MeasDistance * c / 2));
    D = (((D_Rad * 180) / (pi)) - 90);
    if (D < -25)
      D = -25;
    if (D > 25)
      D = 25;
  }
  if (isnan(D))
    D = y;

  PWM1_val = (((D * 80 * km) / 50) + 89); // Servo motor controller + servo offset

  if (PWM1_val < 50)
    PWM1_val = 50;
  if (PWM1_val > 135)
    PWM1_val = 135;

  Servo1.write(PWM1_val);
  y = D;
}

void Stop(float Velocityx) {
  digitalWrite(InA2, LOW);
  digitalWrite(InB2, HIGH);
  PWM2_val = 255;
  analogWrite(PWM2, PWM2_val);
  k_eat_ticks(Velocityx * 3.6);
  digitalWrite(InA2, LOW);
  digitalWrite(InB2, LOW);
  k_eat_ticks(Velocityx * 3.6);
}

void doMeasurement()
{
  noInterrupts();   // Read the previous result (pause interrupts while doing so)
  for (int i = 0; i < tINT; i++) {
    if (i == 0) {
      MAFL = travelTime[i] / 2.0 * (float)soundSpeed / 10000.0;
      if (MAFL < 100) {
        totalL = totalL - runningAverageRegL[averageIndex];
        runningAverageRegL[averageIndex] = MAFL;
        totalL = totalL + runningAverageRegL[averageIndex];
      }
    }
    else {
      MAFR = travelTime[i] / 2.0 * (float)soundSpeed / 10000.0;
      if (MAFR < 100) {
        totalR = totalR - runningAverageRegR[averageIndex];
        runningAverageRegR[averageIndex] = MAFR;   // in m
        totalR = totalR + runningAverageRegR[averageIndex];
        averageIndex++;
      }
    }
  }
  if (averageIndex > averageLength - 1)
    averageIndex = 0;

  interrupts();
  digitalWrite(triggerPin, HIGH);    // HIGH pulse for at least 10µs
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);     // Set LOW again
}

void call_INT0() {
  byte pinRead;
  pinRead = digitalRead(EchoLeft);
  interruptHandler(pinRead, 0);
}

void call_INT1() {
  byte pinRead;
  pinRead = digitalRead(EchoRight);
  interruptHandler(pinRead, 1);
}

void interruptHandler(bool pinState, int nIRQ) {
  unsigned long currentTime = micros();   // Get current time (in µs)
  if (pinState) { // If pin state has changed to HIGH -> remember start time (in µs)
    startTime[nIRQ] = currentTime;
  }
  else {          // If pin state has changed to LOW -> calculate time passed (in µs)
    travelTime[nIRQ] = currentTime - startTime[nIRQ];
  }
}
