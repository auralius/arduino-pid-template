#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#define USE_TIMER_1                   true

#include "TimerInterrupt.h"

 
#define TIMER1_INTERVAL_MS             1


int encoderA_pin         = 3;      // Digital pin #3
int encoderB_pin         = 2;      // Digital pin #2
const int pwm_port       = 11;     // PWM of motor, Timer 2
const int dir_port       = 13;     // Direction of the motor. 


volatile int pulses      = 0;      // Output pulses.
const int ppr            = 2940;   // Pulses per rotation 


String inputs[4]; // ["KP", "KI", "KD", "SV"]


float past_err  = 0.0;   // previous error
float err       = 0.0;   // current error
float P         = 0.0;   // proportional term
float I         = 0.0;   // integral term
float D         = 0.0;   // derivative term
float Df        = 0.0;   // filtered derivative term


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

  pinMode(pwm_port, OUTPUT);
  pinMode(dir_port, OUTPUT);
  
  analogWrite(pwm_port, 0);     
  digitalWrite(dir_port, HIGH);
  pinMode(encoderA_pin, INPUT);
  pinMode(encoderB_pin, INPUT);

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
  Serial.print((float)now/1000000., 4);
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
 * Put your PID control here!
 */
float tau = 0.1; // seconds
inline void PID()
{ 
  PV       = (float)pulses / (float)ppr * 360.0; // update your process value here

  past_err = err;
  err      = SV - PV;

  P        = KP * err;
  D        = KD * (err - past_err) / (dt*1e-3);
  I        = I + KI * (dt*1e-3) * err;
  Df       = (tau * Df + (dt*1e-3) * D) / (tau + (dt*1e-3)) ;
  
  CO       = abs(P + I + Df);
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
    digitalWrite(dir_port, HIGH);
  else
    digitalWrite(dir_port, LOW);

  // Send control output to PWM
  analogWrite(pwm_port, (int)CO);
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
}


/*
 * The following two functions handle the quadrature encoder.
 */
void A_CHANGE() 
{
  if ( digitalRead(encoderB_pin) == 0 ) {
    if ( digitalRead(encoderA_pin) == 0 ) {
      // A fell, B is low
      pulses--; // Moving forward
    } 
    else {
      // A rose, B is high
      pulses++; // Moving reverse
    }
  } 
  else {
    if ( digitalRead(encoderA_pin) == 0 ) {
      pulses++; // Moving reverse
    } 
    else {
      // A rose, B is low
      pulses--; // Moving forward
    }
  }
}
