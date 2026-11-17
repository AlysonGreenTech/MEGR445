#include <QTRSensors.h>

// This example is designed for use with six analog QTR sensors. These
// reflectance sensors should be connected to analog pins A0 to A5. The
// sensors' emitter control pin (CTRL or LEDON) can optionally be connected to
// digital pin 2, or you can leave it disconnected and remove the call to
// setEmitterPin().
//
// The setup phase of this example calibrates the sensors for ten seconds and
// turns on the Arduino's LED (usually on pin 13) while calibration is going
// on. During this phase, you should expose each reflectance sensor to the
// lightest and darkest readings they will encounter. For example, if you are
// making a line follower, you should slide the sensors across the line during
// the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is. Improper calibration will result in
// poor readings.
//
// The main loop of the example reads the calibrated sensor values and uses
// them to estimate the position of a line. You can test this by taping a piece
// of 3/4" black electrical tape to a piece of white paper and sliding the
// sensor across it. It prints the sensor values to the serial monitor as
// numbers from 0 (maximum reflectance) to 1000 (minimum reflectance) followed
// by the estimated location of the line as a number from 0 to 5000. 1000 means
// the line is directly under sensor 1, 2000 means directly under sensor 2,
// etc. 0 means the line is directly under sensor 0 or was last seen by sensor
// 0 before being lost. 5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.


/* ---------------------------------------------------------------------
   [A]  WIRING
   A record of where the wires go, not a set of choices.
   --------------------------------------------------------------------- */




const int TRIG_PIN = 9;       // ultrasonic TRIG  -> SERVO_2 header, signal pin
const int ECHO_PIN = 10;      // ultrasonic ECHO  -> SER1 header, signal pin

const int DIR_LATCH = 12;     // the four pins that run the shift register
const int DIR_CLK   = 4;      // underneath the motor shield
const int DIR_SER   = 8;
const int DIR_EN    = 7;      // active LOW

const int CH[5][3] = {
  {  0, 0, 0 },               // unused
  { 11, 2, 3 },               // M1   left motor
  {  3, 1, 4 },               // M2
  {  5, 5, 7 },               // M3   right motor
  {  6, 0, 6 }                // M4
};
const int LEFT  = 3;
const int RIGHT = 4;


/* ---------------------------------------------------------------------
   [B]  SETTINGS
   --------------------------------------------------------------------- */
const int  TRIM_L  = 100;      // Week 9. Put YOUR numbers here.
const int  TRIM_R  = 100;
const int  CRUISE  = 150;     // wheel speed, 0 to 255

const int  STOP_CM = 15;      // anything closer than this: stop
const int  NOTHING_SEEN = 999;// what [D] hands back when no echo came home

const long ECHO_TIMEOUT_US = 30000;  // stop listening after 30 ms
const long MEASURE_MS      = 60;     // one measurement every 60 ms
const long PRINT_MS        = 200;    // one line on the monitor every 200 ms

/* ---------------------------------------------------------------------
   [C]  MOTORS  -  Week 8 and Week 9, unchanged and working
   --------------------------------------------------------------------- */
byte latchState = 0;

void latchTx() {
  digitalWrite(DIR_LATCH, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(DIR_CLK, LOW);
    digitalWrite(DIR_SER, (latchState >> i) & 1);
    digitalWrite(DIR_CLK, HIGH);
  }
  digitalWrite(DIR_LATCH, HIGH);
}

void setBit(int b, int v) {
  if (v) latchState |=  (1 << b);
  else   latchState &= ~(1 << b);
  latchTx();
}

void drive(int ch, int speed) {
  long mag = speed < 0 ? -speed : speed;
  if (mag > 255) mag = 255;

  if (ch == LEFT)  mag = (mag * 100) / TRIM_L;
  if (ch == RIGHT) mag = (mag * 100) / TRIM_R;

  if (speed > 0)      { setBit(CH[ch][1], 1); setBit(CH[ch][2], 0); }
  else if (speed < 0) { setBit(CH[ch][1], 0); setBit(CH[ch][2], 1); }
  else                { setBit(CH[ch][1], 0); setBit(CH[ch][2], 0); }

  analogWrite(CH[ch][0], (int)mag);
}


/* ---------------------------------------------------------------------
   [D]  MEASURE
   One job: how far away is the nearest thing in front, in centimetres.
   --------------------------------------------------------------------- */
int readDistanceCm() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);          // a 10 microsecond ping
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  int duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);   // out and back, in microseconds
  //Serial.print("duration - ");
  //Serial.println(duration);
  if (duration == 0) return NOTHING_SEEN;   // Week 4: no echo is not "a wall right here"

  // sound covers 0.0343 cm every microsecond, and the ping goes there AND back:
  //   cm = duration x 0.0343 / 2  =  duration x 343 / 20000   (whole numbers only)
  int cm = (duration * 343) / 20000;
  //Serial.print("readDistanceCM() cm= ");
  //Serial.println(cm);
  return cm;
}


/* ---------------------------------------------------------------------
   [E]  DECIDE AND ACT
   --------------------------------------------------------------------- */
int  cm          = NOTHING_SEEN;
bool blocked     = false;
long lastMeasure = 0;
long lastPrint   = 0;






QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

void setup()
{
  Serial.begin(9600);

  //motor pins defined 
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(DIR_LATCH, OUTPUT);
  pinMode(DIR_CLK,   OUTPUT);
  pinMode(DIR_SER,   OUTPUT);
  pinMode(DIR_EN,    OUTPUT);
  digitalWrite(DIR_EN, LOW);
  latchTx();
  //------------------------

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);

   
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  Serial.println("Starting calibration");
  delay(500);
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
 
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop()
{
  readDistanceCm();
  
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  // position in relation to the middle black line
  Serial.print("position - ");
  Serial.println(position);
  
  // delayed blocked 
  if (millis() - lastMeasure >= MEASURE_MS) {
    lastMeasure = millis();
    cm = readDistanceCm();
    blocked = (cm < STOP_CM);
  }

  Serial.print("Loop cm= ");
  Serial.println(cm);


  drive(LEFT, 150); drive(RIGHT, 150);

  bool rDrift = int(position) >= 2300; 
  bool lDrift = int(position) <= 1000; 
  
 if (!rDrift) { drive(LEFT, 0);      drive(RIGHT, 0);      }
  else         { drive(LEFT, CRUISE*2); drive(RIGHT, CRUISE); }

 if (!lDrift) { drive(LEFT, 0);      drive(RIGHT, 0);      }
  else         { drive(LEFT, CRUISE); drive(RIGHT, CRUISE*2); }
  
}
