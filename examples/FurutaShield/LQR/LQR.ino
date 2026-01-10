#include "FurutaShield.h"
#include <SamplingServo.h>

#define TS 10

float Ksu, Kq, Kdq, Ke, eta;

float u = 0;

float mp = 0.00034;
float g = 9.81;
float lp = 0.055;
float Ip = mp * pow(lp, 2) / 3;
float beta = PI;
float w0 = sqrt((mp * g * lp) / Ip);
float w1 = mp * g * lp / 2;
int wmax = 100;
float E0 = 0.00183447;
int h;
const int stop = 5000;
float angle;
float prevMill;

bool up = false;
bool STOP = false;
bool enable = false;
bool realTimeViolation = false;


unsigned long prevTime;

BLA::Matrix<4, 1> X;
BLA::Matrix<4, 1> X1;


BLA::Matrix<2, 1> Y;

BLA::Matrix<2, 4> H = { 1, 0, 0, 0, 0, 0, 1, 0 };
BLA::Matrix<4, 4> Q_kalman = {
  0.0001, 0, 0, 0,
  0, 0.0001, 0, 0,
  0, 0, 0.00001, 0,
  0, 0, 0, 0.00001
};

BLA::Matrix<2, 2> R_kalman = { 1e-12, 0, 0, 1e-10 };

BLA::Matrix<1, 4> K = { -0.8840, -1.6925, -213.6926, -18.2625 };
//BLA::Matrix<1, 5> K = { -19.0138, 3.6472, 3.7027, -30.1155, -24.2531 };
BLA::Matrix<2, 1> prevOutput;

BLA::Matrix<4, 4> A = {
  1.0, 0.01, 0, 0,
  0, 1.0, 0, 0,
  0, 0, 1.0067, 0.01,
  0, 0, 1.338, 1.0027
};

BLA::Matrix<4, 1> B = { 0.0001, 0.01, -0.0001, -0.0136 };

void setup() {
  FurutaShield.begin();
  u = stop;
  FurutaShield.actuatorWrite(u);
  Serial.begin(115200);

  BLA::Matrix<5, 1> parseData = FurutaShield.parseControlData();

  Ksu = parseData(0);
  Kq = parseData(1);
  Kdq = parseData(2);
  Ke = parseData(3);
  eta = parseData(4);

  delay(3000);
  Wire.begin();

  Sampling.period(TS * 1000);      // Sampling init.
  Sampling.interrupt(stepEnable);  // Interrupt fcn.
}

void loop() {

  if ((Serial.available() > 0) || (STOP)) {
    char incomingByte = Serial.read();

    if (incomingByte == 'S') {
      Serial.println(Y);
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

void stepEnable() {                                 // ISR
  if (enable == true) {                             // If previous sample still running
    realTimeViolation = true;                       // Real-time has been violated
    Serial.println("Real-time samples violated.");  // Print error message
    STOP = true;
  }
  enable = true;  // Enable step flag
}


void step() {
  Y = FurutaShield.sensorRead();


  if (-0.5 <= Y(1) && Y(1) <= 0.5) {
    if (!up)

    up = true;
    u = -float((K * (X))(0, 0));

    if (abs(X(0)) >= PI) {
      STOP = true;
    }
  } else {
    up = false;

    u = FurutaShield.swingUp(Ksu, Kq, Kdq, Ke, eta, X, wmax, (3 * PI / 4), E0);
  }

  //u = 0;
  //X1 = ekf(u, Y, H, Q_kalman, R_kalman);
  X = FurutaShield.estimate(Y, H);
  //u = sat(u, 50, -50);

  

  X(2) = FurutaShield.wrapToPi(X(2));
  //X1(2) = FurutaShield.wrapToPi(X1(2));

  FurutaShield.actuatorWrite(u);

  Serial.print("0");
  Serial.print(" ");

  Serial.print(X(0));
  Serial.print(" ");

  Serial.print(X(1));
  Serial.print(" ");

  Serial.print(X(2));
  Serial.print(" ");

  Serial.print(X(3));
  Serial.print(" ");

  Serial.print(u);
  Serial.print(" ");

  Serial.print(X1(0));
  Serial.print(" ");

  Serial.print(X1(1));
  Serial.print(" ");

  Serial.print(X1(2));
  Serial.print(" ");

  Serial.print(X1(3));
  Serial.print(" ");

  Serial.println();
}









template<int n>  // Template function definition
BLA::Matrix<n, 1> ekf(float systemInput, BLA::Matrix<2, 1> &measuredOutput, BLA::Matrix<2, n> &C, BLA::Matrix<n, n> &Q, BLA::Matrix<2, 2> &R) {
  static BLA::Matrix<n, 1> x_hat;  // State matrix
  static BLA::Matrix<n, n> P;      // Covariance matrix
  static BLA::Matrix<n, n> I;      // Eye matrix
  BLA::Matrix<n, 1> f;
  BLA::Matrix<n, n> F;
  static bool wasInitialised = false;  // Boolean used to initialize static variables at the start
  static unsigned long _prevMill = 0;


  if (!wasInitialised) {  // Initialise static variables
    x_hat = (~C * measuredOutput);
    P.Fill(0);  // Initialise covariance matrix with zeros
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        I(i, j) = (i == j) ? 1.0 : 0.0;  // Create eye matrix I
      }
    }
    wasInitialised = true;  // Flag initialisation as complete
  }

  // Prediction

  float ddtheta1 = 0.75 * cos(x_hat(2)) * sin(x_hat(2)) * pow(x_hat(1), 2) - 0.40106952 * x_hat(3) + 133.77273 * sin(x_hat(2)) - 1.3636364 * systemInput * cos(x_hat(2));

  f(0) = x_hat(1);
  f(1) = systemInput;
  f(2) = x_hat(3);
  f(3) = ddtheta1;

  x_hat = x_hat + ((float)TS / 1000) * f;


  F = { 0, 1, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 1,
        0, 1.5 * x_hat(1) * cos(x_hat(2)) * sin(x_hat(2)), 133.77273 * cos(x_hat(2)) + 0.75 * pow(x_hat(1), 2) * pow(cos(x_hat(2)), 2) - 0.75 * pow(x_hat(1), 2) * pow(sin(x_hat(2)), 2) + 1.3636364 * systemInput * sin(x_hat(2)), -0.40106952 };

  P = F * P * ~F + Q;  // Calculate error covariance

  // Update
  BLA::Matrix<n, 2> K = P * ~C * (Inverse(C * P * ~C + R));
  x_hat = x_hat + K * (measuredOutput - (C * x_hat));  // Update the state estimate
  P = (I - K * C) * P;                                 // Update error covariance matrix

  return x_hat;  // Return vector of estimated states
}
