#include <FurutaShield.h>
#include <SamplingServo.h>


#define MANUAL 0                                                    // Choose manual reference using potentiometer (1) or automatic reference trajectory (0)
float R[] = { 0.0, 1.0, -0.55, 1.0, 0.35, -0.35, -0.8, 1.0, 0.0 };  // Reference trajectory for MANUAL 0

#define TS 10.0

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
float amax = (3 * PI / 4);  //Limit for arm angle

float E0 = 0.00183447;
int h;
const int stop = 5000;
float angle;
float prevMill;

bool up = false;
bool STOP = false;
bool enable = false;
bool realTimeViolation = false;

int T = 600;          // Section length
int i = 0;            // Section counter
unsigned long k = 0;  // Sample index

unsigned long prevTime;

BLA::Matrix<5, 1> X;
BLA::Matrix<4, 1> X1;


BLA::Matrix<2, 1> Y;

BLA::Matrix<1, 1> Xr;

BLA::Matrix<1, 5> K = { 0.0901, -10.6209, -6.2091, -308.0191, -36.7630};


void setup() {
  FurutaShield.begin();
  u = stop;
  FurutaShield.actuatorWrite(u);
  Serial.begin(115200);

  BLA::Matrix<5, 1> parseData = FurutaShield.swingUpPar();

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

    if (incomingByte == 'S' || (STOP)) {
      while (1) {

        FurutaShield.emergStop();
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


  if (abs(FurutaShield.wrapToPi(Y(1))) <= 0.3) {

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

    u = -float((K * (X))(0, 0));


  } else {


    u = FurutaShield.swingUp(Ksu, Kq, Kdq, Ke, eta, X1, wmax, amax, mp, lp);
  }

  if (isnan(u)) {
    u = stop;
    STOP = true;
  }


  X1 = FurutaShield.estimate(Y);
  X1(2) = FurutaShield.wrapToPi(X1(2));

  X(0) += (Xr(0) - Y(0));
  X(1) = X1(0);
  X(2) = X1(1);
  X(3) = X1(2);
  X(4) = X1(3);


  u = sat(u, 100.0, -100.0);
  FurutaShield.actuatorWrite(u);



  Serial.print(Xr(0));
  Serial.print(" ");

  Serial.print(X(1));
  Serial.print(" ");

  Serial.print(X(2));
  Serial.print(" ");

  Serial.print(X(3));
  Serial.print(" ");

  Serial.print(X(4));
  Serial.print(" ");

  Serial.print(u);
  Serial.print(" ");


  Serial.println();
}

float sat(float u, float umax, float umin) {
  return (u > umax ? umax : (u < umin ? umin : u));
}
