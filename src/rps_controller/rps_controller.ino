// rps_controller.ino - Firmware for RPS control on ESC/Brushless motor pair
// For Arduino Pro Mini
// Written by George Zogopoulos, 2020-02-12

///////////
// INCLUDES
///////////

#include <FreqMeasure.h> // Used for RPM reading
#include "Servo.h"
#include "PID_v1.h"

//////////
// DEFINES
//////////

// TIME CONSTANTS
#define T_READ_FAST_US 10000 // Period of fast sensor read rate
#define T_READ_SLOW_US 40000 // Period of slow sensor read rate
#define T_PUB_US 20000 // Period of data publish rate
#define T_DEBUG_US 1000000 // Period of debug print rate
#define T_LED_US 500000 // Period of LED toggle
#define T_RPS_TIMEOUT_MS 1000 // Zero RPS measurement after this time of inactivity
#define T_PWM_LOW_MAX_US 40000 // Maximum number of microseconds of low-time in a PWM frame

// PIN AND PORT DECLARATIONS
//#define COMMS_PORT Serial // Computer interface through FTDI
#define DEBUG_PORT Serial // Computer interface through FTDI
#define RPM_PIN 8 // RPM reading
#define THROT_IN_PIN 7 // PWM input
#define THROT_OUT_PIN 9 // PWM output
#define LED_PIN 13 // Alive LED

// OTHER DECLARATIONS
#define DEBUG false
#define DEBUG_TOGGLE_PIN 2 // Toggle in every loop subsection
#define RPS_MIN 0 // Minimum reference RPS
#define RPS_MAX 250 // Maximum reference RPS
#define PID_P 3
#define PID_I 11
#define PID_D 0.1
#define NUM_POLES 12 // Number of motor poles
#define FREQ_COMPENSATE 8 // freqMeasure to o'scope freq ratio

//////////////////////
// GLOBAL DECLARATIONS
//////////////////////

uint32_t timestamp, tstamp_read_fast_us, tstamp_read_slow_us, tstamp_pub_us, tstamp_led_us, tstamp_debug_us;
uint32_t delta_read_fast_us, delta_read_slow_us, delta_pub_us, delta_led_us, delta_debug_us;

uint32_t rps_msr;
uint32_t throttle_cmd_us;
uint32_t rps_ref_cmd;
int32_t throttle_cmd;
double throttle_cmd_d, rps_msr_d, rps_ref_cmd_d;
double rps_msr_d_prev=0;

//Specify the links and initial tuning parameters
PID esc_pid(&rps_msr_d, &throttle_cmd_d, &rps_ref_cmd_d, PID_P, PID_I, PID_D, DIRECT);
Servo esc_driver;

///////////////////
// Helper functions
///////////////////

// Toggle the debug output pin
void toggle_debug_pin()
{
  digitalWrite(DEBUG_TOGGLE_PIN,!digitalRead(DEBUG_TOGGLE_PIN));
}

////////////////
// Setup routine
////////////////
void setup() {
  // Initialize serial ports
  if (DEBUG)
  {
    DEBUG_PORT.begin(115200);
  }

  // Setup I/O
  pinMode(LED_PIN, OUTPUT);
  pinMode(RPM_PIN, INPUT);
  pinMode(THROT_IN_PIN, INPUT);
  pinMode(THROT_OUT_PIN, OUTPUT);
  pinMode(DEBUG_TOGGLE_PIN, OUTPUT);

  FreqMeasure.begin(); // Initialize RPM counter
  rps_ref_cmd = 0; // Initialize RPS reference command
  rps_ref_cmd_d = 0; // Initialize RPS reference command

  esc_driver.attach(THROT_OUT_PIN);
  
  esc_pid.SetOutputLimits(-500, 500);
  esc_pid.SetMode(AUTOMATIC);
  esc_pid.SetSampleTime(T_PUB_US/1000);

  // Initialize timers
  tstamp_read_fast_us = micros();
  tstamp_read_slow_us = tstamp_read_fast_us;
  tstamp_pub_us = tstamp_read_fast_us;
  tstamp_debug_us = tstamp_read_fast_us;
}

///////////////
// Loop routine
///////////////
void loop() {

  ////////////////////////////////
  //Reading from fast rate sensors
  timestamp = micros();
  delta_read_fast_us = timestamp - tstamp_read_fast_us;
  if (delta_read_fast_us > T_READ_FAST_US) // Time to take measurements in fast loop
  {
//    toggle_debug_pin();
    tstamp_read_fast_us = timestamp;
    
    // Read RPM
    if (FreqMeasure.available())
    {
      uint32_t engine_cycle = FreqMeasure.read();
      rps_msr_d = FreqMeasure.countToFrequency(engine_cycle);
      rps_msr_d = rps_msr_d / NUM_POLES * 3 / FREQ_COMPENSATE; // Divide the measured frequency with the number of motor poles per phase

      // For some reason measurement bugs out and returns 0
      if (rps_msr_d<1)
      {
        rps_msr_d = rps_msr_d_prev;
      }
      else
      {
        rps_msr_d_prev = rps_msr_d;
      }
    }
    else
    {
        // If reading RPS has timed out, then zero out the measurement
        rps_msr = 0;
        rps_msr_d = 0;
    }
    
    rps_msr = rps_msr_d;
  }

  ////////////////////////////////
  // Reading from low rate sensors
  delta_read_slow_us = timestamp - tstamp_read_slow_us;
  if (delta_read_slow_us > T_READ_SLOW_US) // Time to take measurements in slow loop
  {
//    toggle_debug_pin();
    tstamp_read_slow_us = timestamp;
    unsigned long rps_ref_cmd_us = pulseIn(THROT_IN_PIN, HIGH, T_PWM_LOW_MAX_US); // Initial read of PWM ref command, may timeout and yield 0.
    rps_ref_cmd_us = constrain(rps_ref_cmd_us, 1000, 2000);
    rps_ref_cmd = map(rps_ref_cmd_us, 1000, 2000, RPS_MIN, RPS_MAX);
    rps_ref_cmd_d = rps_ref_cmd;
  }

  //////////////////
  // Control outputs
  delta_pub_us = timestamp - tstamp_pub_us;
  if (delta_pub_us > T_PUB_US) // Time to calculate new controls
  {
//    toggle_debug_pin();
    tstamp_pub_us = timestamp;

    bool pid_update = esc_pid.Compute();
    throttle_cmd = (int32_t)throttle_cmd_d;
    throttle_cmd_us = constrain(throttle_cmd, -500, 500) + 1500; // Translate throttle command to pwm microseconds
        
    // Override controller and force cutoff if needed
    if (rps_ref_cmd<2)
    {
      throttle_cmd_us = 1000;
    }
    
    esc_driver.writeMicroseconds(throttle_cmd_us);
  }

  //////////////
  // Debug output
  delta_debug_us = timestamp - tstamp_debug_us;
  if ((delta_debug_us > T_DEBUG_US) && (DEBUG)) // Time to print debug output
  {
//    toggle_debug_pin();
    tstamp_debug_us = timestamp;
    DEBUG_PORT.print("\tRPS_REF: "); DEBUG_PORT.print(rps_ref_cmd);
    DEBUG_PORT.print("\tRPS_MSR: "); DEBUG_PORT.print(rps_msr);
    DEBUG_PORT.print("\tRPM_MSR: "); DEBUG_PORT.print(rps_msr*60);
    DEBUG_PORT.print("\tPWM_OUT: "); DEBUG_PORT.print(throttle_cmd_us);
    DEBUG_PORT.print("\tLoop Time: "); DEBUG_PORT.print(micros() - timestamp);
    DEBUG_PORT.println();
  }

  // Heartbeat LED
  delta_led_us = timestamp - tstamp_led_us;
  if (delta_led_us > T_LED_US) // Time to toggle led
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    tstamp_led_us = micros();
  }

}

