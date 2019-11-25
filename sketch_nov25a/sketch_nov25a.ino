
#include <ECE3.h>

uint16_t sensorValues[8];
double weights[] = { -8.333, -7.5, -6.666, -4.166, 4.166, 6.666, 7.5, 8.333};
double prevErrors[5] = {0, 0, 0, 0, 0};
const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int right_nslp_pin = 11; // nslp ==> awake & ready for PWM

const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;
const int BASE_SPD = 100;

int state = 0;
int counterDoughnuts = 0;
int leftSpd = 0;
int rightSpd = 0;
//double prevError = 0;
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
  delay(2000);
}

void PID(double error, int &leftSpd, int &rightSpd) {
  double Kp = 6;
  long Up = error * Kp;
  double Kd = 24;
  long dE_dt = 0;
  for (int i = 0; i < 4; i++)
  {
    dE_dt += error - prevErrors[i + 1];
    //Serial.print(prevErrors[i]);
    //Serial.print("\t");
  }
  dE_dt = dE_dt / 4;
  double Ud = (dE_dt) * Kd;
  long U = Ud + Up;
  //Serial.print(0.001 * U);
  //Serial.print("\t");
  leftSpd = -0.001 * U;
  rightSpd = 0.001 * U;
}

void turnAround() {
  analogWrite(right_pwm_pin, 0);
  analogWrite(left_pwm_pin, 0);
  digitalWrite(right_dir_pin, HIGH);
  analogWrite(right_pwm_pin, 70);
  analogWrite(left_pwm_pin, 70);
  delay(770);
  digitalWrite(right_dir_pin, LOW);
  analogWrite(right_pwm_pin, 100);
  analogWrite(left_pwm_pin, 100);
  delay(100);
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
  if (sensorValues[3] > 1800 && sensorValues[4] > 1800 && sensorValues[2] > 1800 && sensorValues[5] > 1800)
  {
    if (counterDoughnuts > 3) {
      state++;
      if (state == 2)
        return;
      turnAround();
      counterDoughnuts = 0;
    }
    else
      counterDoughnuts++;
  }
  else
  {
    for (int i = 3; i >= 0; i--)
      prevErrors[i + 1] = prevErrors[i];
    prevErrors[0] = error;
    PID(error, leftSpd, rightSpd);
    analogWrite(right_pwm_pin, rightSpd + BASE_SPD);
    analogWrite(left_pwm_pin, leftSpd + BASE_SPD);
  }
  /*
    Serial.print("Lft spd: ");
    Serial.print(leftSpd + BASE_SPD);
    Serial.print("\t\tRght spd: ");
    Serial.println(rightSpd + BASE_SPD);
  */
}
