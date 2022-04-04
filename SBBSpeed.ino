/*
 * Code for counting encoder and computing wheel speed. Communication using SPI interface
 * Author: Stefano Lovato, 11-10-2021
 */

/*
 * Radius on wheel is 255mm
 * Radius of encoder wheel is 61/2=30.5mm
 * Gear ratio between tire and encoder is roughly 255/30.5=8.36
 * Correction factor of 0.9784 found by calibration
 */

#include "SerialTransfer.h"

//config
//#define IS_DEBUG //uncomment to enable debug (print to Serial)
#define T_DEBUG 10e3 //sampling time of debug
#define VAR2SHOW c //variable to show in monitor

#define IS_INTERRUPT_ON_B //uncomment to use both interrupts
#define MODE_INT CHANGE //RISING, FALLING, or CHANGE

#define ENCA_PIN 5 //encoder A pin
#define ENCB_PIN 6 //encoder B pin
#define ENCZ_PIN 7 //encoder Z pin

#define SERIALUSED Serial1
#define BAUDRATE 4e6

#define PPR 100.0 //pulse per revolution of the encoder
#define SPEED_RATIO 8.36 //speed ratio between wheel and encoder speed
#define RADIUS 0.295 //wheel effective radius
#define DT 10.0e3 //update time (us)
#define SPEED_SCALE 1.0e3 //speed scale, now m/s to mm/s
#define DIST_SCALE 4.0 //distance scale, now 1m to units of 0.25m
#define CORRECTION_FAC 0.9784 //correction factor from calibration (should be close to 1)


#ifndef IS_INTERRUPT_ON_B
#if MODE_INT==CHANGE
#define MULTIPLIER 2.0F //multiplier is two if interrupt on A only w/ change mode
#else
#define MULTIPLIER 1.0F //multiplier is two if interrupt on A only w/ rising or falling mode
#endif
#else
#if MODE_INT==CHANGE
#define MULTIPLIER 4.0F //multiplier is four if interrupt on both A, B w/ change mode
#else
#define MULTIPLIER 2.0F //multiplier is four if interrupt on both A, B w/ rising or falling
#endif
#endif

//typedef
struct Speed_T { //size=4 bytes
	int16_t speed;
	uint16_t dist;
};

//variables
Speed_T speed;
uint32_t timer = 0; //timer
volatile int8_t dir = 1; //CW is 1, and CCW is -1
volatile uint32_t lastPulseTime, timeBetweenPulse = -1; //timers
volatile uint32_t rev = 0; //revolution counter
volatile int64_t c; //pulse counter

//object for serial transfer
SerialTransfer serialTransfer;

#ifdef IS_DEBUG
uint32_t timerDebug = 0; //debug timer
#endif

void setup() {


  //start serial
  SERIALUSED.begin(BAUDRATE);
  while (!SERIALUSED); //wait for serial begins
  serialTransfer.begin(SERIALUSED);

  //set pins
  pinMode(ENCA_PIN, INPUT_PULLUP);
  pinMode(ENCB_PIN, INPUT_PULLUP);
  pinMode(ENCZ_PIN, INPUT_PULLUP);

  //set interrupt on pin A
  attachInterrupt(digitalPinToInterrupt(ENCA_PIN), doEncoderA, MODE_INT);
#ifdef IS_INTERRUPT_ON_B
  //set interrupt on pin B
  attachInterrupt(digitalPinToInterrupt(ENCB_PIN), doEncoderB, MODE_INT);
#endif
  //set interrupt on pin Z
  attachInterrupt(digitalPinToInterrupt(ENCZ_PIN), doEncoderZ, RISING);

  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, HIGH);
  
}

void loop() {  
  
  if ((micros()-timer)>=DT) {
    
    timer = micros();//update timer
    
    //compute speed
	speed.speed = dir * round(CORRECTION_FAC*double(2 * PI*RADIUS) / double(MULTIPLIER*PPR*SPEED_RATIO*timeBetweenPulse)*1e6*SPEED_SCALE);
	speed.dist = 2 * PI*RADIUS*float(rev) / SPEED_RATIO * DIST_SCALE * CORRECTION_FAC;
	//send struct via serial
	serialTransfer.sendDatum(speed);

  }

#ifdef IS_DEBUG //do debug stuff
  if ((micros()-timerDebug)>=T_DEBUG) {
    timerDebug = micros();
    Serial.println(VAR2SHOW);
  }
#endif
  
}

void doEncoderA() {
    timeBetweenPulse = micros()-lastPulseTime;
    lastPulseTime = micros();

    if (digitalReadFast(ENCA_PIN)) {
      if (!digitalReadFast(ENCB_PIN)) {
        dir = +1;
        c++;
      }
      else {
        dir = -1;
        c--;
      }
    } else {
      if (digitalReadFast(ENCB_PIN)) {
        dir = +1;
        c++;
      }
      else {
        dir = -1;
        c--;
      }
    }
}

#ifdef IS_INTERRUPT_ON_B
void doEncoderB() {
  //count time between pulses
    timeBetweenPulse = micros()-lastPulseTime;
    lastPulseTime = micros();

    if (digitalReadFast(ENCB_PIN)) {
      if (digitalReadFast(ENCA_PIN)) {
        dir = +1;
        c++;
      }
      else {
        dir = -1;
        c--;
      }
    } else {
      if (!digitalReadFast(ENCA_PIN)) {
        dir = +1;
        c++;
      }
      else {
        dir = -1;
        c--;
      }
    }
}
#endif

void doEncoderZ() {
	rev++;
}
