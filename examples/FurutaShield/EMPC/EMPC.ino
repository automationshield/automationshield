#include <FurutaShield.h>   // Include main library
#include <SamplingServo.h>  // Include sampling library

#include "ectrl.h"  // Include header file for EMPC controller

#include <empcSequential.h>

#define MANUAL 0  // Choose manual reference using potentiometer (1) or automatic reference trajectory (0)

#define TS 10.0                  // Sampling period in milliseconds
unsigned long k = 0;             // Sample index
bool nextStep = false;           // Flag for step function
bool realTimeViolation = false;  // Flag for real-time sampling violation

float R[] = { 0.0, 0.4, 1.0, -0.55, 1.0, 0.85, 0.35, 0.8, 1.0, 0.0 };  // Reference trajectory
//float R[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // Reference trajectory
float y = 0.0;
float yprev = 0.0;
float u = 0.0;

int T = 600;  // Section length
int i = 0;    // Section counter
unsigned long prevMill;
float Ksu, Kq, Kdq, Ke, eta;

bool up = false;
bool STOP = false;
bool enable = false;

const int stop = 5000;
float mp = 0.00034;           // Pendulum mass [kg]
float lp = 0.055;             // Distance from axis to CoM [m]
int wmax = 100;
float E0 = 0.00183447;
float E;
float r = 0.0;

BLA::Matrix<4, 1> X;

BLA::Matrix<6, 1> Xr;


BLA::Matrix<2, 1> Y;
BLA::Matrix<2, 1> prevOutput;

BLA::Matrix<2, 4> H = { 1, 0, 0, 0, 0, 0, 1, 0 };

static float u_opt[MPT_RANGE];  // predicted inputs

extern struct mpc_ctl ctl;  // Object representing presets for MPC controller

void setup() {
  FurutaShield.actuatorWrite(0);

  Serial.begin(115200);  //--Initialize serial communication

  delay(1000);
  Sampling.period(TS * 1000.0);
  Sampling.interrupt(stepEnable);

  Serial.begin(115200);
  BLA::Matrix<5, 1> parseData = FurutaShield.swingUpPar();

  Ksu = parseData(0);
  Kq = parseData(1);
  Kdq = parseData(2);
  Ke = parseData(3);
  eta = parseData(4);
  FurutaShield.begin();
  delay(3000);
}

void loop() {

  if ((Serial.available() > 0) || (STOP)) {
    char incomingByte = Serial.read();

    if (incomingByte == 'S' || (STOP)) {
      while (1) {

        FurutaShield.actuatorWrite(stop);
      }
    }
  }

  if (enable) {
    step();
    enable = false;
  }
}

void stepEnable() {            // ISR
  if (enable == true) {        // If previous sample still running
    realTimeViolation = true;  // Real-time has been violated
    //Serial.println("Real-time samples violated.");  // Print error message
  }
  enable = true;  // Enable step flag
}

void step() {                     // Define step function
  Y = FurutaShield.sensorRead();  // Angle in radians
  X = FurutaShield.estimate(Y);
  X(2) = FurutaShield.wrapToPi(X(2));

  if (-0.3 <= X(2) && X(2) <= 0.3) {
    up = true;
        if (i >= sizeof(R) / sizeof(float)) {  // If trajectory ended
      //FurutaShield.actuatorWrite(5000);    // Stop the Motor
      while (true) {
        FurutaShield.actuatorWrite(stop);
      };  // End of program execution
    }
    if (k % (T * i) == 0) {  // Moving through trajectory values
      //r = R[i];
      r = R[i];
      i++;  // Change input value after defined amount of samples
    }
    k++;  // Increment

    float X_array[4];
    for (int i = 0; i < 4; i++) {
      X_array[i] = X(i);
      if (i == 0)
        X_array[0] = X_array[0] - r;
    }
    //Now call empcSequential with X_array
    empcSequential(X_array, u_opt);
    u = u_opt[0];  // Save system input into input variable

  } else {
    up = false;

    u = FurutaShield.swingUp(Ksu, Kq, Kdq, Ke, eta, X, wmax, (3 * PI / 4), mp,lp);
  }

  if (isnan(u)) {
    u = stop;
    STOP = true;
  }

  FurutaShield.actuatorWrite(u);  // Actuation
  Serial.print(r);
  Serial.print(" ");

  Serial.print(X(0));
  Serial.print(" ");

  Serial.print(X(1));
  Serial.print(" ");

  Serial.print(X(2));
  Serial.print(" ");

  Serial.print(X(3));
  Serial.print(" ");

  Serial.print(u);  // Print input
  Serial.println(" ");
}
