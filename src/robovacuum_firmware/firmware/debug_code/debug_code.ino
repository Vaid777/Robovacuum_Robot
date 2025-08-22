// Debug code to find actual encoder PPR
// Upload this first to determine your encoder specifications

// L298N H-Bridge Connection PINs (enA and enB are permanently high)
#define L298N_in1 6   // PWM Dir Motor A
#define L298N_in2 9   // PWM Dir Motor A
#define L298N_in3 10  // PWM Dir Motor B  
#define L298N_in4 11  // PWM Dir Motor B

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 3  // Interrupt 
#define right_encoder_phaseB 5  
#define left_encoder_phaseA 2   // Interrupt
#define left_encoder_phaseB 4

// Debug variables
volatile unsigned long right_encoder_counter = 0;
volatile unsigned long left_encoder_counter = 0;
unsigned long last_millis = 0;
const unsigned long interval = 1000; // 1 second for easier counting

void setup() {
  Serial.begin(115200);
  
  // Initialize motor pins
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);
  
  // Stop motors initially
  stopMotors();
  
  // Initialize encoder pins
  pinMode(right_encoder_phaseB, INPUT_PULLUP);
  pinMode(left_encoder_phaseB, INPUT_PULLUP);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
  
  Serial.println("=== ENCODER PPR DEBUG MODE ===");
  Serial.println("This will help determine your actual encoder PPR");
  Serial.println("Commands:");
  Serial.println("'r' - Test right motor");
  Serial.println("'l' - Test left motor"); 
  Serial.println("'s' - Stop motors");
  Serial.println("'c' - Clear counters");
  Serial.println("Motor will run at PWM 100 for easy counting");
  Serial.println("Count pulses for exactly 10 wheel rotations");
}

void loop() {
  // Handle serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case 'r':
        Serial.println("Testing RIGHT motor at PWM 100");
        testRightMotor();
        break;
      case 'l':
        Serial.println("Testing LEFT motor at PWM 100");
        testLeftMotor();
        break;
      case 's':
        Serial.println("Stopping all motors");
        stopMotors();
        break;
      case 'c':
        Serial.println("Clearing encoder counters");
        right_encoder_counter = 0;
        left_encoder_counter = 0;
        break;
    }
  }
  
  // Print encoder counts every second
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval) {
    Serial.print("Right encoder: ");
    Serial.print(right_encoder_counter);
    Serial.print(" pulses | Left encoder: ");
    Serial.print(left_encoder_counter);
    Serial.println(" pulses");
    
    // Reset counters for next interval
    right_encoder_counter = 0;
    left_encoder_counter = 0;
    last_millis = current_millis;
  }
}

void testRightMotor() {
  // Run right motor forward at PWM 100
  analogWrite(L298N_in1, 100);
  analogWrite(L298N_in2, 0);
  analogWrite(L298N_in3, 0);
  analogWrite(L298N_in4, 0);
}

void testLeftMotor() {
  // Run left motor forward at PWM 100
  analogWrite(L298N_in1, 0);
  analogWrite(L298N_in2, 0);
  analogWrite(L298N_in3, 100);
  analogWrite(L298N_in4, 0);
}

void stopMotors() {
  analogWrite(L298N_in1, 0);
  analogWrite(L298N_in2, 0);
  analogWrite(L298N_in3, 0);
  analogWrite(L298N_in4, 0);
}

// Encoder interrupt callbacks
void rightEncoderCallback() {
  right_encoder_counter++;
}

void leftEncoderCallback() {
  left_encoder_counter++;
}