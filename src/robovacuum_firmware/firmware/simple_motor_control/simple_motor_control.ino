// L298N H-Bridge Connection PINs
#define L298N_enB 11  // PWM
#define L298N_in4 8  // Dir Motor A
#define L298N_in3 7  // Dir Motor A

float cmd = 0;

void setup() {
  // Set pin modes
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);
  
  // Set Motor Rotation Direction
  digitalWrite(L298N_in3, LOW);
  digitalWrite(L298N_in4, HIGH);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available())
  {
    cmd = Serial.readString().toFloat();
  }
  analogWrite(L298N_enB, cmd*100);
}