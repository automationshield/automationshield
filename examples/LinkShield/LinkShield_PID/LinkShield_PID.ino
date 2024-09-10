/*
  LinkShield identification example.

  Example used to acquire data for LinkShield system identification.

  The LinkShield implements an a flexible rotational link experiment
  on an Arduino shield. This example initialises the sampling
  subsystem from the AutomationShield library and allows user

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Martin Vríčan.
  Last update: 24.05.2023.
*/

#include <LinkShield.h>            	// Include header for hardware API


unsigned long Ts = 5;               // Sampling in milliseconds
unsigned long k = 0;                // Sample index
bool nextStep = false;              // Flag for sampling
bool realTimeViolation = false;     // Flag for real-time sampling violation
bool endExperiment = false;         // Boolean flag to end the experiment

float y = 0.0;
float y_1 = 0.0;                     // Output variable
float y_2 = 0.0;                     // Output variable
float r = 0.0;                        	// Input (open-loop), initialized to zero
float u = 0.0;
float u_slow = 0.0;
float u_fast = 0.0;
float R[]={0.00, PI/6, -PI/6, 0.00};
int T = 10;                         	// Section length (appr. '/.+2 s)
unsigned int i = 0;                            // Section counter

// PID Tuning
#define KP_Slow 85                   
#define KI_Slow 30
#define KD_Slow 1

#define VIBRATION_CONTROL 0

#define KP_Fast 80					
#define KI_Fast 50
#define KD_Fast 0.2


PIDAbsClass PIDAbsSlow;
PIDAbsClass PIDAbsFast;

void setup() {
  Serial.begin(250000);               // Initialize serial

  // Initialize linkshield hardware
  LinkShield.begin();                  // Define hardware pins
  LinkShield.calibrate();              // Remove sensor bias

  // Initialize sampling function
  Sampling.period(Ts * 1000);          // Sampling init.
  Sampling.interrupt(stepEnable);      // Interrupt fcn.

  // Set the PID constants
 PIDAbsSlow.setKp(KP_Slow); // Proportional
 PIDAbsSlow.setKi(KI_Slow); // Integral
 PIDAbsSlow.setKd(KD_Slow); // Derivative
  
 PIDAbsFast.setKp(KP_Fast); // Proportional
 PIDAbsFast.setKi(KI_Fast); // Integral
 PIDAbsFast.setKd(KD_Fast); // Derivative

 PIDAbsSlow.setTs(Sampling.samplingPeriod); // Sampling
 PIDAbsFast.setTs(Sampling.samplingPeriod); // Sampling
}

// Main loop launches a single step at each enable time
void loop() {
  if (nextStep) {  					  // If ISR enables
    step();                           // Algorithm step
    nextStep = false;                 // Then disable
  }
}

void stepEnable() {                                    // ISR
  if (endExperiment == true) {                         // If the experiment is over
  LinkShield.actuatorWriteNew(0.00);
    while (1);                                         // Do nothing
  }
  if (nextStep == true) {                              // If previous sample still running
    realTimeViolation = true;                      // Real-time has been violated
    Serial.println("Real-time samples violated."); // Print error message
    while (1);                                     // Stop program execution
  }
  nextStep = true;                                     // Change flag
}

// A single algorithm step
void step() {
	
	// Switching between experiment sections
	
  if (i > (sizeof(R) / sizeof(R[0]))) { // If at end of trajectory
		endExperiment = true;         // Stop program execution at next ISR
	
	} 
	else if (k % (T * i) == 0) {    // If at the end of section
    r = R[i];                     // Progress in trajectory
    i++;                          // Increment section counter
  }

	y_1 = LinkShield.servoPotRead();
	y_2 = LinkShield.flexRead();          // Read sensor
  
  u_slow =PIDAbsSlow.compute((r - y_1), -5, 5, -30, 30); // Compute constrained absolute-form PID
 
 #if VIBRATION_CONTROL
 u_fast =PIDAbsFast.compute((y_2), -5, 5, -100, 100); // Compute constrained absolute-form PID
 u = AutomationShield.constrainFloat((u_slow+u_fast),-5.0,5.0);
 #else
 u = AutomationShield.constrainFloat((u_slow),-5.0,5.0);
 #endif 
  	  
  LinkShield.actuatorWriteNew(u);         // [V] actuate

  // Print to serial port
  Serial.print(r);
  Serial.print(", ");
  Serial.print(y_1,8);  // Print reference
  Serial.print(", ");
  Serial.print(y_2,8);                        // Print output
  Serial.print(", ");
  Serial.print(u_slow);                        // Print output
  Serial.print(", ");
  Serial.print(u_fast);                        // Print output
  Serial.print(", ");
  Serial.println(u);                      // Print input

  
  k++;                                    // Increment time-step k
}
