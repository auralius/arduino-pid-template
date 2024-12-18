#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#define USE_TIMER_1                   true

#include "TimerInterrupt.h"

 
#define TIMER1_INTERVAL_MS             1


int encoderA   = 3;      // Digital pin #3
int encoderB   = 2;      // Digital pin #2
const int pwm  = 11;     // PWM of motor, Timer 2
const int dir  = 13;     // Direction of the motor. 


int pulses     = 0;      // Output pulses.
const int ppr  = 2940;   // Pulses per rotation 


String inputs[4];        // ["KP", "KI", "KD", "SV"]


float past_err  = 0.0;
float err       = 0.0;
float derr      = 0.0;


float KP        = 200.0; // kp
float KD        = 0.1;   // ki
float KI        = 0.1;   // hs
float CO        = 0.0;   // control output
float PV        = 0.0;   // process value
float SV        = 0.0;   // set value


unsigned long base;       
unsigned long now;       // current time stamp in micro-secs
unsigned long prev;      // previous time stamp in micro-secs
float dt;                // elapsed time in milli-secs


void setup() {
  ITimer1.init();
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, Timer1Handler);
 
   
  // https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency
  TCCR2B = TCCR2B & B11111000 | B00000001; // Timer2, PWM frequency: 31372.55 Hz
  
  Serial.begin(115200);

  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
  
  analogWrite(pwm, 0);     
  digitalWrite(dir, HIGH);
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);

  attachInterrupt(0, A_CHANGE, CHANGE);

  base = micros();
  now  = micros() - base;
}


void rx()
{
  while(Serial.available()){
    int StringCount = 0;
    String input = Serial.readString();

    // Split the string into substrings
    while (input.length() > 0){
      int index = input.indexOf(' ');
      if (index == -1) { // No space found
        inputs[StringCount++] = input;
        break;
      }
      else {
        inputs[StringCount++] = input.substring(0, index);
        input = input.substring(index+1);
      }
    }

    KP = inputs[0].toFloat();
    KI = inputs[1].toFloat();
    KD = inputs[2].toFloat();
    SV = inputs[3].toFloat();
  }
}


inline void tx()
{
  Serial.print(dt);
  Serial.print(",");
  Serial.print(SV);
  Serial.print(",");
  Serial.print(CO);
  Serial.print(",");
  Serial.print(PV);
  Serial.print("\n");  
}


inline void clock_update()
{
  prev = now;
  now  = micros() - base;
  dt   = (float)(now - prev) / 1000.0; // in msecs 
}


/*
 * Put your PID contro here!
 */
inline void PID()
{ 
  // PV = baca analor sensor temeprature
  past_err = err;
  err      = SV - PV;
  derr     = err - past_err;
  
  CO       = abs(err * KP);
}


void Timer1Handler()
{
  // Update all time variables
  clock_update();

  // PID control
  PID();

  // Guard the control signal
  if (CO > 255.)
    CO = 255.;

  // Handle direction
  if (err < 0.0)
    digitalWrite(dir, LOW);
  else
    digitalWrite(dir, HIGH);

  // Send control output to PWM
  analogWrite(pwm, (int)CO);
}


/*
 * Arduino's forever loop
 */
void loop()
{
  // Serial receive
  rx();
  
  // Serial transmit
  tx();

  delay(1);
}


/*
 * The following two functions handle the quadrature encoder.
 */
void A_CHANGE() 
{
  if ( digitalRead(encoderB) == 0 ) {
    if ( digitalRead(encoderA) == 0 ) {
      // A fell, B is low
      pulses--; // Moving forward
    } 
    else {
      // A rose, B is high
      pulses++; // Moving reverse
    }
  } 
  else {
    if ( digitalRead(encoderA) == 0 ) {
      pulses++; // Moving reverse
    } 
    else {
      // A rose, B is low
      pulses--; // Moving forward
    }
  }

  PV = (float)pulses / (float)ppr * 360.0;
}
