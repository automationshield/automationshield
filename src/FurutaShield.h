#ifndef FURUTASHIELD_H
#define FURUTASHIELD_H

#include "Arduino.h"
#include <AutomationShield.h>
#include <Wire.h>
#include "AS5600_AS.h"
#include "lib/BasicLinearAlgebra/BasicLinearAlgebra.h"

#define STEP_PIN 5
#define DIR_PIN 2
#define ENA_PIN 3

#define M0 8
#define M1 9
#define M2 10

#define FURUTA_RPIN A3

static bool t = false;
static bool dir;
static long steps;

class FurutaClass
{

public:
  /**
   * @brief Initializes the Furuta Shield: calibrates sensor, sets microstepping, and configures I/O pins.
   * @param microstepping Microstepping setting for the stepper motor. Use one of {1, 2, 4, 8, 16, 32}. Default is 32.
   */
  void begin(int microstepping = 32);

  /**
   * @brief Computes swing-up control input using energy-based method.
   * @param Ksu Main gain for swing-up.
   * @param Kq Gain for arm position.
   * @param Kdq Gain for arm angular velocity.
   * @param Ke Gain for energy correction.
   * @param eta Energy shaping parameter.
   * @param x State vector [theta_0, dot_theta_0, theta_1, dot_theta_1].
   * @param wmax Maximum arm angular velocity (used for saturation).
   * @param amax Maximum arm angle (used for saturation).
   * @param m Mass of pendulum.
   * @param l Distance from pivot to center of mass of pendulum [m].
   * @return Control input for swing-up phase.
   */
  float swingUp(float Ksu, float Kq, float Kdg, float Ke, float eta, BLA::Matrix<4, 1> &x, int wmax, int amax, float m, float l);

  /**
   * @brief Reads the current position of the arm and pendulum.
   * @return 2×1 vector containing [theta_0, theta_1] in radians,
   *         where theta_0 is the arm angle and theta_1 is the pendulum angle.
   */
  BLA::Matrix<2, 1> sensorRead(void);

  /**
   * @brief Sets the computed control input to the actuator (stepper motor).
   *
   * The control input represents the desired angular velocity [rad/s].
   * If the input is ±inf, the motor is disabled.
   *
   * @param u Control input from the controller [rad/s].
   */
  void actuatorWrite(float u);

  /**
   * @brief Wraps an angle to the interval (-π, π].
   * @param angle Angle in radians to be wrapped (can be outside the interval).
   * @return Wrapped angle in the range (-π, π], in radians.
   */
  float wrapToPi(float angle);

  /**
   * @brief Reads swing-up parameters from the serial input.
   *
   * This function allows easier tuning of swing-up control by reading
   * parameters sent from the computer via the serial interface.
   *
   * @return 5×1 vector containing [Ksu, Kq, Kdq, Ke, eta] for the swingUp() function.
   */
  BLA::Matrix<5, 1> swingUpPar(void);
  BLA::Matrix<5, 1> swingUpParWifi(void);

  /**
   * @brief Immediately stops the stepper motor when this function is called.
   */
  void emergStop(void);

  /**
   * @brief Reads the value of the potentiometer and converts it to a percentage.
   * @return Potentiometer value as a float in the range [0.0, 100.0].
   */
  float referenceRead(void);

  /**
   * @brief Estimates the full system state using numerical differentiation.
   *
   * Computes angular velocities of the arm and pendulum by differentiating the measured positions.
   *
   * @param measuredOutput 2×1 vector containing measured angles [theta_0, theta_1], in radians.
   * @param Ts Sampling time in seconds. Default is 0.01 s (10 ms).
   * @return 4×1 state vector [theta_0, dot_theta_0, theta_1, dot_theta_1].
   */

  BLA::Matrix<4, 1> estimate(BLA::Matrix<2, 1> &measuredOutput, float Ts = 0.01)
  {
    static BLA::Matrix<4, 1> x_hat;
    static BLA::Matrix<2, 1> prevOutput = {0, measuredOutput(1)};
    // BLA::Matrix<2, 4> C = {1, 0, 0, 0, 0, 0, 1, 0};

    x_hat(0) = measuredOutput(0);
    x_hat(1) = float((measuredOutput(0) - prevOutput(0)) / Ts);
    x_hat(2) = wrapToPi(measuredOutput(1));
    x_hat(3) = (measuredOutput(1) - prevOutput(1)) / Ts;

    prevOutput = measuredOutput;
    return x_hat;
  }

private:
  float sat(float x, float up, float down);
  int sign(float x);
  void setMicrosteping(int microstepping);
  static void stepEnable(void);

  // ====== Sensor and calibration ======
  bool wasCalibrated = false;
  int _quad;
  int _turn;
  int _prevQ;
  float g = 9.81;
  BLA::Matrix<2, 1> data = {0, 0};
  BLA::Matrix<5, 1> ControlData;

  // ====== Energy-based swing-up components ======
  //float E0 = 0;
  float E0 = 0.00183447;
  float E;
  float usu, uq, udq, ue;

  // ====== Motor control ======
  float _w = 0;     // current velocity input
  float _wprev = 0; // previous velocity input
  float _speed = 0; // calculated delay between steps in microseconds

  int _Ts;
  unsigned long _prevMill;

  const float _maxSpeed = 981000;
  const float _minSpeed = 19;

  int _motorCon;
  const float _motorstepsPerRev = 200;
  int _microstep; // default microstepping
};

#endif
extern FurutaClass FurutaShield;