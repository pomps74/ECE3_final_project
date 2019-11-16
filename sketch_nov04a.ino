
#include <ECE3.h>

uint16_t sensorValues[8];
int weights[] = { -4, -3, -2, -1, 1, 2, 3, 4};
const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int right_nslp_pin = 11; // nslp ==> awake & ready for PWM

const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;
const int BASE_SPD = 20;

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);
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

void PID(double error, int &leftSpd, int &rightSpd) {
  int Kp = -2.5;
  int Up = error * Kp;
  int Ud = 0;
  int U = Ud + Up;
  if (U > 0)
    *leftSpd = 0.001 * U;
  else
    *rightSpd = 0.001 * U;
}

void loop()
{
  int leftSpd = 0;
  int rightSpd = 0;

  double error = 0.00;  // read raw sensor values
  ECE3_read_IR(sensorValues);

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  for (int i = 0; i < 8; i++)
  {
    error += weights[i] *  (int) sensorValues[i];;
    //Serial.print(sensorValues[i]);
    //Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  PID(error, &leftSpd, &rightSpd);
  //Serial.println();
  //Serial.println( error);
  analogWrite(left_pwm_pin, leftSpd + BASE_SPD);
  analogWrite(right_pwm_pin, rightSpd + BASE_SPD);
}
