#include <PID_v1.h>

// L298N H-Bridge Connection PINs
// enA and enB are permanently tied HIGH (3.3V or 5V)
// Motor control is done via IN pins only
#define L298N_in1 6  // Right Motor Direction A
#define L298N_in2 9  // Right Motor Direction B (used as PWM)
#define L298N_in3 10   // Left Motor Direction A  
#define L298N_in4 11   // Left Motor Direction B (used as PWM)

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 3  // Interrupt pin
#define right_encoder_phaseB 5  
#define left_encoder_phaseA 2   // Interrupt pin
#define left_encoder_phaseB 4

// Encoder variables
volatile unsigned int right_encoder_counter = 0;
volatile unsigned int left_encoder_counter = 0;
String right_wheel_sign = "p";  // 'p' = positive, 'n' = negative
String left_wheel_sign = "p";
unsigned long last_millis = 0;
const unsigned long interval = 100;  // 100ms update interval (10Hz)

// Serial command parsing variables
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
char value[10] = "00.00";
uint8_t value_idx = 0;
char current_sign = 'p';  // Track the sign for current command

// PID variables - these match what ROS2 driver expects
double right_wheel_cmd_vel = 0.0;  // rad/s (commanded velocity from ROS2)
double left_wheel_cmd_vel = 0.0;   // rad/s
double right_wheel_meas_vel = 0.0; // rad/s (measured velocity to send to ROS2)
double left_wheel_meas_vel = 0.0;  // rad/s
double right_wheel_cmd = 0.0;      // PWM output (0-255)
double left_wheel_cmd = 0.0;       // PWM output (0-255)

// PID tuning parameters - IMPROVED VALUES
double Kp_r = 15.0, Ki_r = 8.0, Kd_r = 0.2;
double Kp_l = 15.0, Ki_l = 8.0, Kd_l = 0.2;

// PID controllers
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {
  // Configure motor control pins as outputs
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);
  
  // Initialize motors to stopped state
  digitalWrite(L298N_in1, LOW);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, LOW);
  digitalWrite(L298N_in4, LOW);
  
  // Configure PID controllers
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  rightMotor.SetOutputLimits(0, 255);  // PWM range
  leftMotor.SetOutputLimits(0, 255);
  
  // Initialize serial communication (matches ROS2 driver baudrate)
  Serial.begin(115200);
  
  // Configure encoder pins with pull-ups
  pinMode(right_encoder_phaseB, INPUT_PULLUP);
  pinMode(left_encoder_phaseB, INPUT_PULLUP);
  
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
  
  Serial.println("Arduino robot control ready");
}

void loop() {
  // 1. Read and parse commands from ROS2 driver
  handleSerialCommands();
  
  // 2. Update motor control at fixed intervals
  updateMotorControl();
}

void handleSerialCommands() {
  // Parse commands sent by ROS2 driver in format: "rp01.23,ln00.45,"
  // 'r' = right wheel, 'l' = left wheel
  // 'p' = positive direction, 'n' = negative direction  
  // followed by velocity value in rad/s, ending with comma
  
  while (Serial.available()) {
    char chr = Serial.read();
    
    switch (chr) {
      case 'r':
        // Start of right wheel command
        is_right_wheel_cmd = true;
        is_left_wheel_cmd = false;
        value_idx = 0;
        memset(value, 0, sizeof(value));
        current_sign = 'p';  // Default to positive
        break;
        
      case 'l':
        // Start of left wheel command
        is_right_wheel_cmd = false;
        is_left_wheel_cmd = true;
        value_idx = 0;
        memset(value, 0, sizeof(value));
        current_sign = 'p';  // Default to positive
        break;
        
      case 'p':
        // Positive direction flag - store it
        current_sign = 'p';
        break;
        
      case 'n':
        // Negative direction flag - store it
        current_sign = 'n';
        break;
        
      case ',':
        // End of command - process the accumulated value
        double cmd_value = atof(value);
        
        // Apply sign to the command value
        if (current_sign == 'n') {
          cmd_value = -cmd_value;
        }
        
        if (is_right_wheel_cmd) {
          right_wheel_cmd_vel = cmd_value;
        } else if (is_left_wheel_cmd) {
          left_wheel_cmd_vel = cmd_value;
        }
        
        // Reset for next command
        value_idx = 0;
        memset(value, 0, sizeof(value));
        current_sign = 'p';
        break;
        
      default:
        // Accumulate numeric characters and decimal point
        if (value_idx < sizeof(value) - 1 && (isdigit(chr) || chr == '.')) {
          value[value_idx] = chr;
          value_idx++;
        }
        break;
    }
  }
}

void updateMotorControl() {
  unsigned long current_millis = millis();
  
  if (current_millis - last_millis >= interval) {
    // Calculate measured velocities from encoder counts
    // Convert encoder pulses to rad/s
    // Formula: (pulses * 60.0/total_pulses_per_rev) * (1000.0/interval_ms) * (2*PI/60)
    // Simplified: (pulses * conversion_factor) * frequency_multiplier
    right_wheel_meas_vel = (right_encoder_counter * (60.0/1125.0)) * (1000.0/interval) * 0.10472;
    left_wheel_meas_vel = (left_encoder_counter * (60.0/1125.0)) * (1000.0/interval) * 0.10472;
    
    // Apply direction sign
    if (right_wheel_sign == "n") {
      right_wheel_meas_vel = -right_wheel_meas_vel;
    }
    if (left_wheel_sign == "n") {
      left_wheel_meas_vel = -left_wheel_meas_vel;
    }
    
    // Run PID controllers
    rightMotor.Compute();
    leftMotor.Compute();
    
    // Control motors based on commanded velocities
    controlRightMotor();
    controlLeftMotor();
    
    // Send encoder feedback to ROS2 driver
    sendEncoderFeedback();
    
    // Reset encoder counters for next interval
    right_encoder_counter = 0;
    left_encoder_counter = 0;
    last_millis = current_millis;
  }
}

void controlRightMotor() {
  if (abs(right_wheel_cmd_vel) < 0.01) {
    // Stop motor - both pins LOW
    digitalWrite(L298N_in1, LOW);
    analogWrite(L298N_in2, 0);
    return;
  }
  
  int pwm_value = constrain(abs((int)right_wheel_cmd), 0, 255);
  
  // Add minimum PWM to overcome motor static friction
  if (pwm_value > 0 && pwm_value < 60) {
    pwm_value = 60;  // Minimum PWM to start motor movement
  }
  
  if (right_wheel_cmd_vel > 0) {
    // Forward: IN1=HIGH, IN2=PWM(inverted)
    digitalWrite(L298N_in1, HIGH);
    analogWrite(L298N_in2, 255 - pwm_value);  // Inverted PWM
  } else {
    // Backward: IN1=PWM(inverted), IN2=HIGH  
    analogWrite(L298N_in1, 255 - pwm_value);  // Inverted PWM
    digitalWrite(L298N_in2, HIGH);
  }
}

void controlLeftMotor() {
  if (abs(left_wheel_cmd_vel) < 0.01) {
    // Stop motor - both pins LOW
    digitalWrite(L298N_in3, LOW);
    analogWrite(L298N_in4, 0);
    return;
  }
  
  int pwm_value = constrain(abs((int)left_wheel_cmd), 0, 255);
  
  // Add minimum PWM to overcome motor static friction
  if (pwm_value > 0 && pwm_value < 60) {
    pwm_value = 60;  // Minimum PWM to start motor movement
  }
  
  // Left motor might be mounted in reverse - adjust accordingly
  if (left_wheel_cmd_vel > 0) {
    // Forward: IN3=HIGH, IN4=PWM(inverted)
    digitalWrite(L298N_in3, HIGH);
    analogWrite(L298N_in4, 255 - pwm_value);  // Inverted PWM
  } else {
    // Backward: IN3=PWM(inverted), IN4=HIGH
    analogWrite(L298N_in3, 255 - pwm_value);  // Inverted PWM  
    digitalWrite(L298N_in4, HIGH);
  }
}

void sendEncoderFeedback() {
  // Send feedback in exact format expected by ROS2 driver's read() function
  // Format: "rp1.234,ln0.567," 
  // This matches the parsing in robovacuum_interface.cpp
  
  String feedback = "";
  
  // Right wheel feedback
  feedback += "r";
  feedback += (right_wheel_meas_vel >= 0) ? "p" : "n";
  feedback += String(abs(right_wheel_meas_vel), 3);  // 3 decimal places
  feedback += ",";
  
  // Left wheel feedback
  feedback += "l"; 
  feedback += (left_wheel_meas_vel >= 0) ? "p" : "n";
  feedback += String(abs(left_wheel_meas_vel), 3);  // 3 decimal places
  feedback += ",";
  
  Serial.println(feedback);
}

// Encoder interrupt service routines
void rightEncoderCallback() {
  // Determine rotation direction from phase B
  if (digitalRead(right_encoder_phaseB) == HIGH) {
    right_wheel_sign = "p";  // Forward
  } else {
    right_wheel_sign = "n";  // Backward
  }
  right_encoder_counter++;
}

void leftEncoderCallback() {
  // Determine rotation direction from phase B
  if (digitalRead(left_encoder_phaseB) == HIGH) {
    left_wheel_sign = "n";  // Backward (inverted due to motor mounting)
  } else {
    left_wheel_sign = "p";  // Forward
  }
  left_encoder_counter++;
}