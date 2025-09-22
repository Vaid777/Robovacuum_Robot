// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_in2 13  // Dir Motor A
#define L298N_in1 12  // Dir Motor A

#define left_encoder_phaseA 3  // Interrupt 
#define left_encoder_phaseB 5  

unsigned int left_encoder_counter = 0;
String left_encoder_sign = "p";
double left_wheel_meas_vel = 0.0;    // rad/s

void setup() {
  // Set pin modes
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  
  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  Serial.begin(115200);

  pinMode(left_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  left_wheel_meas_vel = (10 * left_encoder_counter * (60.0/76800.0)) * 0.10472;
  String encoder_read = "l" + left_encoder_sign + String(left_wheel_meas_vel);
  Serial.println(encoder_read);
  left_encoder_counter = 0;
  analogWrite(L298N_enA, 255);
  delay(100);
}

void leftEncoderCallback()
{
  if(digitalRead(left_encoder_phaseB) == HIGH)
  {
    left_encoder_sign = "n";
  }
  else
  {
    left_encoder_sign = "p";
  }
  left_encoder_counter++;
}