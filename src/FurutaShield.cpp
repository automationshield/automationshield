#include "FurutaShield.h"

#include "SamplingStepper.h" // Uncomment this line if you want to use FurutaShield; otherwise, leave it commented.

AS5600 as5600;

void FurutaClass::begin(int microstepping)
{

#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_RENESAS_UNO) // For AVR, SAMD, Renesas architecture boards
  Wire.begin();                                                                                  // Use Wire object
  as5600.setWirePtr(&Wire);
#elif ARDUINO_ARCH_SAM // For SAM architecture boards
  Wire1.begin(); // Use Wire1 object
  as5600.setWirePtr(&Wire1);
#endif

  as5600.begin();
  bool isDetected = as5600.detectMagnet();
  if (isDetected == 0)
  { // If magnet not detected go on
    while (1)
    { // Go forever until magnet detected
      if (isDetected == 1)
      {                                                     // If magnet detected
        AutomationShield.serialPrint("Magnet detected \n"); // Print information then break
        break;
      }
      else
      {                                                           // If magnet not detected
        AutomationShield.serialPrint("Can not detect magnet \n"); // Print information then go back to check while statement
      }
      isDetected = as5600.detectMagnet();
    }
  }
  wasCalibrated = as5600.calibrate(MODE_0_360);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  setMicrosteping(microstepping);

  Serial.println("Potentiometer is set correctly");
  delay(500);
}

void FurutaClass::actuatorWrite(float u)
{

  if (isnan(u))
    emergStop();

  else
  {

    if (u != 5000)
    {
      //_Ts = millis() - _prevMill;
      _Ts = 10;
      _w = _wprev + (_Ts * u / 1000);
      _wprev = _w;
      _prevMill = millis();
    }
    else
    {
      _w = 0;
      _wprev = 0;
    }

    //_w = u;

    // Serial.println(_w);
    if (u >= 5000 || _w == 0)
    {
      t = false;
      _w = 0;
      digitalWrite(ENA_PIN, HIGH);
    }
    else
    {
      digitalWrite(ENA_PIN, LOW);
      t = true;

      _speed = ((_motorCon / _w));
      _speed = abs(_speed);

      if (_speed <= _minSpeed)
        _speed = _minSpeed;
      else if (_speed >= _maxSpeed)
        _speed = _maxSpeed;
      if (_w < 0)
      {
        digitalWrite(DIR_PIN, 0);
        dir = false;
      }
      else
      {
        digitalWrite(DIR_PIN, 1);
        dir = true;
      }

#ifdef STEPPER
      SamplingStepper.period(_speed);
      SamplingStepper.interrupt(stepEnable);
#endif
    }
  }
}

void FurutaClass::stepEnable(void)
{
  if (t)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEP_PIN, LOW);

    if (dir)
      steps++;
    else
      steps--;
  }
}

void FurutaClass::emergStop(void)
{
  t = false;
  digitalWrite(ENA_PIN, HIGH);
}

BLA::Matrix<2, 1> FurutaClass::sensorRead()
{
  if (wasCalibrated)
  {
    data(1) = as5600.readAngleRad();
  }
  else
  {
    AutomationShield.serialPrint("The device was *not calibrated*, please run the calibration() method.");
    return -1;
  }
  data(1) = data(1) + PI;
  data(1) = fmod(data(1), 2 * PI);

  if (data(1) >= 0 && data(1) < PI / 2)
    _quad = 1;
  else if (data(1) >= PI / 2 && data(1) < PI)
    _quad = 2;
  else if (data(1) >= PI && data(1) < 3 * PI / 2)
    _quad = 3;
  else if (data(1) >= 3 * PI / 2 && data(1) < 2 * PI)
    _quad = 4;

  if (_quad == 4 && _prevQ == 1)
    _turn--;
  if (_quad == 1 && _prevQ == 4)
    _turn++;
  _prevQ = _quad;
  data(1) = _turn * 2 * PI + data(1);

  data(0) = (float)steps * 1.8 * PI / (180 * _microstep);
  return data;
}

void FurutaClass::setMicrosteping(int microstepping)
{
  _microstep = microstepping;
  if (microstepping == 1)
  { // Full step
    digitalWrite(M0, 0);
    digitalWrite(M1, 0);
    digitalWrite(M2, 0);
  }
  else if (microstepping == 2)
  { // Half step
    digitalWrite(M0, 1);
    digitalWrite(M1, 0);
    digitalWrite(M2, 0);
  }
  else if (microstepping == 4)
  { // 1/4 step
    digitalWrite(M0, 0);
    digitalWrite(M1, 1);
    digitalWrite(M2, 0);
  }
  else if (microstepping == 8)
  { // 1/8 step
    digitalWrite(M0, 1);
    digitalWrite(M1, 1);
    digitalWrite(M2, 0);
  }
  else if (microstepping == 16)
  { // 1/16 step
    digitalWrite(M0, 0);
    digitalWrite(M1, 0);
    digitalWrite(M2, 1);
  }
  else if (microstepping == 32)
  { // 1/32 step
    digitalWrite(M0, 1);
    digitalWrite(M1, 1);
    digitalWrite(M2, 1);
  }

  _motorCon = (2 * M_PI * 1000000) / (_motorstepsPerRev * microstepping);
}

float FurutaClass::swingUp(float Ksu, float Kq, float Kdq, float Ke, float eta, BLA::Matrix<4, 1> &x, int wmax, int amax, float m, float l)
{
  usu = -Ksu * sign(x(3) * cos(x(2)));
  uq = Kq * sign(x(0)) * log10(1 - (abs(x(0)) / amax));
  udq = Kdq * sign(x(1)) * log10(1 - (abs(x(1)) / wmax));
  E = 0.5 * m * l * l * x(3) * x(3) + m * g * l * cos(x(2));
  ue = Ke * (exp(abs(E - eta * E0)) - 1) * sign(E - E0) * sign(x(3) * cos(x(2)));

  return (usu + uq + udq + ue);
}

int FurutaClass::sign(float x)
{
  return (x > 0 ? 1 : (x < 0) ? -1
                              : 0);
}

float FurutaClass::sat(float x, float up, float down)
{
  return (x > up ? up : (x < down ? down : x));
}

float FurutaClass::wrapToPi(float angle)
{
  angle = fmod(angle + M_PI, 2 * M_PI);
  if (angle < 0)
    angle += 2 * M_PI;
  return angle - M_PI;
}

BLA::Matrix<5, 1> FurutaClass::swingUpPar(void)
{
  while (!Serial)
  {
  }

  while (Serial.available() == 0)
  {
  }

  String data = Serial.readStringUntil('\n');

  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  int thirdComma = data.indexOf(',', secondComma + 1);
  int fourthComma = data.indexOf(',', thirdComma + 1);

  if (firstComma != -1 && secondComma != -1 && thirdComma != -1 && fourthComma != -1)
  {

    ControlData(0) = data.substring(0, firstComma).toFloat();
    ControlData(1) = data.substring(firstComma + 1, secondComma).toFloat();
    ControlData(2) = data.substring(secondComma + 1, thirdComma).toFloat();
    ControlData(3) = data.substring(thirdComma + 1, fourthComma).toFloat();
    ControlData(4) = data.substring(fourthComma + 1).toFloat();

    Serial.print("Ksu: ");
    Serial.println(ControlData(0));
    Serial.print("Kq: ");
    Serial.println(ControlData(1));
    Serial.print("Kdq: ");
    Serial.println(ControlData(2));
    Serial.print("Ke: ");
    Serial.println(ControlData(3));
    Serial.print("Eta: ");
    Serial.println(ControlData(4));
    return ControlData;
  }
}
/*
BLA::Matrix<5, 1> FurutaClass::swingUpParWifi(void)
{
  BLA::Matrix<5, 1> ControlData;

  while (!Serial3)
  {
  }

  while (Serial3.available() == 0)
  {
  }

  float param[5];
  int i;
  char buffer[100];
  String data;

  while (true)
  {
    while (Serial3.available() == 0)
    {
    }

    data = Serial3.readStringUntil('\n');
    data.trim();
    Serial.println(data);
    if (data.startsWith("swingUp"))
    {
      data.toCharArray(buffer, sizeof(buffer));

      i = 0;
      char *token = strtok(buffer, " ");
      while (token && i < 5)
      {

        const char *ptr = token;
        if (*ptr == '-' || *ptr == '+')
          ptr++;
        bool decimal = false;
        bool valid = true;

        while (*ptr)
        {
          if (*ptr == '.')
          {
            if (decimal)
            {
              valid = false;
              break;
            }
            decimal = true;
          }
          else if (!isdigit(*ptr))
          {
            valid = false;
            break;
          }
          ptr++;
        }

        if (!valid)
        {
          Serial.print(" Non-numeric token: ");
          Serial.println(token);
          i = -1;
          break;
        }

        param[i++] = atof(token);
        token = strtok(NULL, " ");
      }

      if (i == 5)
        break;

      Serial.println("Invalid input. Waiting for a correct message with 5 values...");
    }
  }

  for (int j = 0; j < 5; j++)
  {
    ControlData(j) = param[j];
  }

  Serial.print("Ksu: ");
  Serial.println(ControlData(0));
  Serial.print("Kq: ");
  Serial.println(ControlData(1));
  Serial.print("Kdq: ");
  Serial.println(ControlData(2));
  Serial.print("Ke: ");
  Serial.println(ControlData(3));
  Serial.print("Eta: ");
  Serial.println(ControlData(4));

  return ControlData;
}*/

float FurutaClass::referenceRead(void)
{
  return AutomationShield.mapFloat(analogRead(FURUTA_RPIN), 0.0, 1024.0, 0.0, 100.0);
}

FurutaClass FurutaShield;
