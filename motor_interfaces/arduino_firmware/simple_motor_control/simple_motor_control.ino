#define L298N_enA 5
#define L298N_in1 6
#define L298N_in2 7
#define right_encoder_phaseA 3
#define right_encoder_phaseB A3
unsigned int right_encoder_counter = 0;
String right_encoder_sign = "p";
double right_wheel_meas_vel = 0.0;


// #define L298N_enB 10
// #define L298N_in3 9
// #define L298N_in4 8

double cmd = 0.0;

// void motor_fowward()
// {
//   digitalWrite(L298N_in1, HIGH);
//   digitalWrite(L298N_in2, LOW);

// }

void setup() 
{
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(right_encoder_phaseA, INPUT);
  pinMode(right_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);

  digitalWrite(L298N_in1, LOW);
  digitalWrite(L298N_in2, HIGH);
  Serial.begin(115200);

}

void loop() 
{
  /*
  0.10472 = 2pi/60
  rpm * 0.10472 = rad/s
  */
  right_wheel_meas_vel = 10 *right_encoder_counter * (60.0 / 350) * 0.10472;  // rad/s
  String encoder_read = right_encoder_sign + String(right_wheel_meas_vel);
  Serial.println(encoder_read);
  analogWrite(L298N_enA, 100);
  right_encoder_counter = 0;

  delay(100);
  


  // if(Serial.available())
  // {
  //   cmd = Serial.readString().toDouble();
  // }
  

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