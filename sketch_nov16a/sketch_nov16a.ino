
#include <ECE3.h>

uint16_t sensorValues[8];
double weights[] = { -4, -2.5, -1.5, -1, 1, 1.5, 2.5, 4};
const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int right_nslp_pin = 11; // nslp ==> awake & ready for PWM

const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;
const int BASE_SPD = 60;

int leftSpd = 0;
int rightSpd = 0;
double prevError = 0;
unsigned long prevTime = 0;

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);
}

void PID(double error, double &prevError, int &leftSpd, int &rightSpd, unsigned long &prevTime) {
  double Kp = 7;
  long Up = error * Kp;
  double Kd = 5;
  long dE_dt = (error - prevError);
  //Serial.print(dE_dt);
  //Serial.print("\t");
  double Ud = (dE_dt) * Kd;
  long U = Ud + Up;
  //Serial.print(0.001 * U);
  //Serial.print("\t");
  leftSpd = -0.001 * U;
  rightSpd = 0.001 * U;
  prevError = error; \
  prevTime = millis();
}

void loop()
{
  double error = 0.00;
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  // sensor values are numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  // max value would be 7500ish, min is -8600ish
  for (int i = 0; i < 8; i++)
    error += weights[i] *  ((double)sensorValues[i]);
  //Serial.println(error);
  PID(error, prevError, leftSpd, rightSpd, prevTime);
/*
  Serial.print("Lft spd: ");
  Serial.print(leftSpd + BASE_SPD);
  Serial.print("\t\tRght spd: ");
  Serial.println(rightSpd + BASE_SPD);
*/
  analogWrite(right_pwm_pin, rightSpd + BASE_SPD);
  analogWrite(left_pwm_pin, leftSpd + BASE_SPD);
}
