#include <PID_v1.h>

// L298N H-Bridge Connection PINs (PWM on in1 & in3, direction on in2 & in4)
#define L298N_in1 10  // PWM Right Motor
#define L298N_in2 13  // Direction Right Motor
#define L298N_in3 6   // PWM Left Motor
#define L298N_in4 8   // Direction Left Motor

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 3  // Interrupt 
#define right_encoder_phaseB 5  
#define left_encoder_phaseA 2   // Interrupt
#define left_encoder_phaseB 4

// Encoders
unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
String right_wheel_sign = "p";  // 'p' = positive, 'n' = negative
String left_wheel_sign = "p";
unsigned long last_millis = 0;
const unsigned long interval = 100;

// Interpret Serial Messages
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;

// PID
double right_wheel_cmd_vel = 0.0;     // rad/s (desired)
double left_wheel_cmd_vel = 0.0;      // rad/s
double right_wheel_meas_vel = 0.0;    // rad/s (measured)
double left_wheel_meas_vel = 0.0;     // rad/s
double right_wheel_cmd = 0.0;         // 0-255 (PWM output)
double left_wheel_cmd = 0.0;          // 0-255

// PID tuning parameters
double Kp_r = 11.5, Ki_r = 7.5, Kd_r = 0.1;
double Kp_l = 12.8, Ki_l = 8.3, Kd_l = 0.1;

PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {
  // Motor pins
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // Initial direction: forward
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in4, LOW);

  // PID mode
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  Serial.begin(115200);

  // Encoders
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  // --- Serial command parsing ---
  if (Serial.available()) {
    char chr = Serial.read();

    if (chr == 'r') {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
    }
    else if (chr == 'l') {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    else if (chr == 'p') { // Positive direction
      if (is_right_wheel_cmd) {
        is_right_wheel_forward = true;
        digitalWrite(L298N_in2, LOW);
      }
      else if (is_left_wheel_cmd) {
        is_left_wheel_forward = true;
        digitalWrite(L298N_in4, LOW);
      }
    }
    else if (chr == 'n') { // Negative direction
      if (is_right_wheel_cmd) {
        is_right_wheel_forward = false;
        digitalWrite(L298N_in2, HIGH);
      }
      else if (is_left_wheel_cmd) {
        is_left_wheel_forward = false;
        digitalWrite(L298N_in4, HIGH);
      }
    }
    else if (chr == ',') { // End of value
      if (is_right_wheel_cmd) {
        right_wheel_cmd_vel = atof(value);
      }
      else if (is_left_wheel_cmd) {
        left_wheel_cmd_vel = atof(value);
      }
      value_idx = 0;
      strcpy(value, "00.00");
    }
    else { // Building the number
      if (value_idx < 5) {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // --- Encoder & PID update ---
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    // Calculate velocities
    right_wheel_meas_vel = (10 * right_encoder_counter * (60.0/1125.0)) * 0.10472;
    left_wheel_meas_vel  = (10 * left_encoder_counter  * (60.0/1125.0)) * 0.10472;

    // Run PID
    rightMotor.Compute();
    leftMotor.Compute();

    // Stop motors if target is 0
    if (right_wheel_cmd_vel == 0.0) right_wheel_cmd = 0;
    if (left_wheel_cmd_vel == 0.0) left_wheel_cmd = 0;

    // Send encoder feedback
    String encoder_read = "r" + right_wheel_sign + String(right_wheel_meas_vel) +
                          ",l" + left_wheel_sign + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);

    // Apply PWM
    analogWrite(L298N_in1, (int)right_wheel_cmd);
    analogWrite(L298N_in3, (int)left_wheel_cmd);

    // Reset counters
    right_encoder_counter = 0;
    left_encoder_counter = 0;
    last_millis = current_millis;
  }
}

// --- Encoder callbacks ---
void rightEncoderCallback() {
  right_wheel_sign = (digitalRead(right_encoder_phaseB) == HIGH) ? "p" : "n";
  right_encoder_counter++;
}

void leftEncoderCallback() {
  left_wheel_sign = (digitalRead(left_encoder_phaseB) == HIGH) ? "n" : "p";
  left_encoder_counter++;
}