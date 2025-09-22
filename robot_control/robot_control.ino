
#include <PID_v1.h>

#define L298N_enA 5
#define L298N_in1 6
#define L298N_in2 7

#define L298N_enB 10
#define L298N_in3 9
#define L298N_in4 8

#define right_encoder_phaseA 3
#define right_encoder_phaseB A3

#define left_encoder_phaseA 2
#define left_encoder_phaseB A2 


unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
String right_encoder_sign = "p";
String left_encoder_sign = "p";
double right_wheel_meas_vel = 0.0;
double left_wheel_meas_vel = 0.0;
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
char value[] = "00.00";
uint8_t value_idx = 0;
double cmd = 0.0;
bool is_cmd_complete = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
double right_wheel_cmd_vel = 0.0;
double left_wheel_cmd_vel = 0.0;

unsigned long last_millis = 0;
const unsigned long interval = 100;

double right_wheel_cmd = 0;
double left_wheel_cmd = 0.0;


double Kp_r = 11.5;
double Ki_r = 7.5;
double Kd_r = 0.1;
double Kp_l = 12.8;
double Ki_l = 8.3;
double Kd_l = 0.1;

PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

// void motor_fowward()
// {
//   digitalWrite(L298N_in1, HIGH);
//   digitalWrite(L298N_in2, LOW);

// }

void setup() 
{
  Serial.begin(115200);
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  pinMode(right_encoder_phaseA, INPUT);
  pinMode(right_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);

  pinMode(left_encoder_phaseA, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);


  digitalWrite(L298N_in1, LOW);
  digitalWrite(L298N_in2, HIGH);

  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  

}

void loop() 
{
  if(Serial.available())
  {
    char chr = Serial.read();
    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0 ;
      is_cmd_complete = false;

    }
    else if(chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0 ;

    }
    else if(chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_right_wheel_forward = true;

      }

    }
    else if(chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_right_wheel_forward = false;

      }

    }

    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);

      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    else
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }

    }
  }

  
  /*
  0.10472 = 2pi/60
  rpm * 0.10472 = rad/s
  */
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    right_wheel_meas_vel = 10 *right_encoder_counter * (60.0 / 350) * 0.10472;  // rad/s
    left_wheel_meas_vel = 10 *left_encoder_counter * (60.0 / 350) * 0.10472;  // rad/s

    rightMotor.Compute();
    leftMotor.Compute();

    if(right_wheel_cmd_vel == 0.0)
    {
      right_wheel_cmd = 0.0;
    }
    if(left_wheel_cmd_vel == 0.0)
    {
      left_wheel_cmd = 0.0;
    }

    String encoder_read = "r" + right_encoder_sign + String(right_wheel_meas_vel) + ",l" + left_encoder_sign + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    analogWrite(L298N_enA, right_wheel_cmd);
    analogWrite(L298N_enB, left_wheel_cmd);

  }




 // delay(100);
  




}


void rightEncoderCallback()
{
  right_encoder_counter++;
  if(digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_encoder_sign = "p";

  }
  else
  {
    right_encoder_sign = "n";

  }

}

void leftEncoderCallback()
{
  left_encoder_counter++;
  if(digitalRead(left_encoder_phaseB) == HIGH)
  {
    right_encoder_sign = "p";

  }
  else
  {
    right_encoder_sign = "n";

  }

}