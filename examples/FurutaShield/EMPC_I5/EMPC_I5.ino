#include <FurutaShield.h>   // Include main library
#include <SamplingServo.h>  // Include sampling library
#include "furuta_ekf_model.h"
//asdasdasdasdassda

#include "ectrl.h"  // Include header file for EMPC controller
#include <empcSequential.h>

ekf_t ekf;

#define MANUAL 0                                                    // Choose manual reference using potentiometer (1) or automatic reference trajectory (0)
float R[] = { 0.8, 1.0, -0.55, 1.0, 0.35, -0.35, -0.8, 1.0, 0.0 };  // Reference trajectory for MANUAL 0

#define TS 10.0  // Sampling period in milliseconds


bool nextStep = false;           // Flag for step function
bool realTimeViolation = false;  // Flag for real-time sampling violation
bool STOP = false;
bool enable = false;
bool START;
int T = 600;                // Section length
int i = 0;                  // Section counter
int wmax = 100;             // Maximum angular velocity
float amax = (3 * PI / 4);  //Limit for arm angle

unsigned long k = 0;  // Sample index
unsigned long t1;
unsigned long period;
float Ksu, Kq, Kdq, Ke, eta;  // Swing-up gains
float mp = 0.00034;           // Pendulum mass [kg]
float lp = 0.055;             // Distance from axis to CoM [m]

double y[2] = { 0.0, 0.0 };  // Output vector
double u[1] = { 0.0 };       // Input vector

BLA::Matrix<5, 1> X;
BLA::Matrix<4, 1> X2;
BLA::Matrix<4, 1> X1;
BLA::Matrix<1, 1> Xr;
BLA::Matrix<2, 1> Y;
BLA::Matrix<2, 4> H = { 1, 0, 0, 0, 0, 0, 1, 0 };

static float u_opt[MPT_RANGE];  // predicted inputs

extern struct mpc_ctl ctl;  // Object representing presets for MPC controller


void setup() {
  FurutaShield.actuatorWrite(0);

  Serial.begin(115200);  //--Initialize serial communication

  FurutaShield.begin();  //--Initialize AeroShield
  delay(1000);
  Sampling.period(TS * 1000.0);
  Sampling.interrupt(stepEnable);

/*
  BLA::Matrix<5, 1> parseData = FurutaShield.swingUpPar();

  Ksu = parseData(0);
  Kq = parseData(1);
  Kdq = parseData(2);
  Ke = parseData(3);
  eta = parseData(4);

*/
  Ksu = 8.5;
  Kq = 9;
  Kdq = 10;
  Ke = 8;
  eta = 1.05;


  ekf.x[0] = 0.0;   // theta0
  ekf.x[1] = 0.0;   // dtheta0
  ekf.x[2] = M_PI;  // theta1 (kyvadlo dole)
  ekf.x[3] = 0.0;   // dtheta1

  // Inicializácia matíc P, Q
  // Inicializácia matíc P a Q
  /*for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      ekf.P[i][j] = (i == j) ? 0.01 : 0.0;
      ekf.Q[i][j] = (i == j) ? ((i == 3) ? 1e-3 : 0.0) : 0.0;
    }
  }*/

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      ekf.P[i][j] = (i == j) ? 0.01 : 0.0;
      ekf.Q[i][j] = (i == j) ? 1e-3 : 0.0;
      if (i == 1 && j == 1) {
        ekf.Q[i][j] = 0;
      }
      if (i == 3 && j == 3) {
        ekf.Q[i][j] = 0;
      }
    }
  }

  // Výpis matice Q
  Serial.println("Matrix Q:");
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Serial.print(ekf.Q[i][j], 7);  // 7 desatinných miest
      Serial.print("\t");
    }
    Serial.println();
  }


  // Inicializácia matice R
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      ekf.R[i][j] = (i == j) ? 1e-5 : 0.0;
    }
  }

  delay(3000);
  t1 = millis();
}



void loop() {

  if ((Serial.available() > 0) || (STOP)) {
    char incomingByte = Serial.read();

    if (incomingByte == 'S' || (STOP)) {
      while (1) {

        FurutaShield.emergStop();
      }
    }
  }

  if (enable) {
    step();
    period++;
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
  y[0] = Y(0);
  y[1] = Y(1);


  if (millis() - t1 >= 4000) {  // Moving through trajectory values
    START = true;
  }


  if (START) {
    if (abs(FurutaShield.wrapToPi(Y(1))) <= 0.3) {

      float X_array[6];
      for (int i = 0; i < 6; i++) {
        X_array[i] = X(i);
      }
      //Now call empcSequential with X_array
      empcSequential(X_array, u_opt);
      u[0] = u_opt[0];  // Save system input into input variable

#if MANUAL
      Xr = AutomationShield.mapFloat(FurutaShield.referenceRead(), 0, 100, -3 * M_PI / 4, 3 * M_PI / 4);
#elif !MANUAL
      if (i >= sizeof(R) / sizeof(float)) {  // If trajectory ended
        while (true) {
          FurutaShield.emergStop();  // Stop the Motor
        }                            // End of program execution
      }
      if (k % (T * i) == 0) {  // Moving through trajectory values
        //r = R[i];
        Xr = R[i];
        i++;  // Change input value after defined amount of samples
      }
      k++;  // Increment
#endif

    } else {
      u[0] = FurutaShield.swingUp(Ksu, Kq, Kdq, Ke, eta, X1, wmax, amax, mp, lp);
    }
  } else {
    if (period > 10 && period <= 40)
      u[0] = 0.2;
    else if (period > 40 && period <= 70)
      u[0] = -0.2;
    else{
       u[0] = 0;
    }
  }
  model(&ekf, ekf.x, u);
  ekf_step(&ekf, y);

  X2(0) = ekf.x[0];
  X2(1) = ekf.x[1];
  X2(2) = FurutaShield.wrapToPi(ekf.x[2]);
  X2(3) = ekf.x[3];


  X1 = FurutaShield.estimate(Y);
  X1(2) = FurutaShield.wrapToPi(X1(2));

  X(0) += (Xr(0) - Y(0));

  /* 
  X(1) = X2(0);
  X(2) = X2(1);
  X(3) = X2(2);
  X(4) = X2(3);
*/

  X(1) = X1(0);
  X(2) = X1(1);
  X(3) = X1(2);
  X(4) = X1(3);

  FurutaShield.actuatorWrite(u[0]);  // Actuation
  //FurutaShield.actuatorWrite(6.28);  // Actuation
/*
  Serial.print(Xr(0));  // Printing reference
  Serial.print(" ");

  Serial.print(X1(0));
  Serial.print(" ");

  Serial.print(X1(1));
  Serial.print(" ");

  Serial.print(X1(2));
  Serial.print(" ");

  Serial.print(X1(3));
  Serial.print(" ");

  Serial.print(u[0]);

  Serial.print(" ");
  Serial.print(X2(0));
  Serial.print(" ");
  Serial.print(X2(1));
  Serial.print(" ");
  Serial.print(X2(2));
  Serial.print(" ");
  Serial.println(X2(3));*/
  
}
