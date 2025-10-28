#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Motor pins
#define ENA 18
#define ENB 17
#define IN1 23
#define IN2 4
#define IN3 13
#define IN4 19

// Encoder pins
#define encA1 35
#define encB1 36
#define encA2 14
#define encB2 16

// Variables for IMU
const int calibration_samples = 500;
float gyroZ_calibration = 0;
float theta_imu = 90.0;
unsigned long last_time;

// Variables for encoder
volatile long val_R = 0;
volatile long val_L = 0;
long val_R_prev = 0;
long val_L_prev = 0;

// Robot parameters
float diameter = 6.7;
float wheel_k = (PI * diameter);  // cm
float L = 26.9;                   // Robot width in cm
float ppr = 11.0;
float gearbox_ratio = 45.0;
float theta = PI / 2;             // rad

// Control parameters
float Kp = 3.3;
float Ki = 0.03;  //0.4
float Kd = 1.3;   //0.6
float target_angle = 0;
int rotation_speed = 150;
float angle_threshold = 0.2;  // degrees
bool use_imu = 1;           // true = IMU, false = Encoder


// ============================
// PID-based Rotation Function
// ============================
void rotateToAngle(float target_deg) {
  float initial_angle = use_imu ? theta_imu : (theta * 180 / PI);
  float target_absolute = initial_angle + target_deg;

  // PID variables
  float integral = 0;
  float last_error = 0;
  unsigned long lastPIDTime = millis();

  while (true) {
    // Update sensor data
    if (use_imu) update_imu();
    else update_encoder();

    // Hitung error
    float current_angle = use_imu ? theta_imu : (theta * 180 / PI);
    float error = target_absolute - current_angle;

    // Normalisasi error ke -180..180
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    // Jika sudah mendekati target, berhenti
    if (abs(error) < angle_threshold) {
      setMotor(0, 0);
      Serial.println("✅ Target angle reached!");
      break;
    }

    // Hitung delta waktu
    unsigned long now = millis();
    float dt = (now - lastPIDTime) / 1000.0;
    lastPIDTime = now;

    // Komponen PID
    integral += error ;
    float derivative = (error - last_error);
    last_error = error;

    // Anti-windup
    if (integral > 2000) integral = 2000;
    if (integral < -2000) integral = -2000;

    // Hitung output PID
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Batasi output sesuai kecepatan rotasi maksimum
    int speed = constrain((int)output, -rotation_speed, rotation_speed);

    // Deadzone compensation (agar motor tetap bergerak di error kecil)
    if (speed > 0 && speed < 100) speed = 100;
    else if (speed < 0 && speed > -100) speed = -100;

    // Set motor (berlawanan arah untuk rotasi)
    setMotor(speed, -speed);

    // Debugging info
    Serial.print("Err: "); Serial.print(error);
    Serial.print(" | Out: "); Serial.print(speed);
    Serial.print(" | Int: "); Serial.print(integral);
    Serial.print(" | Deriv: "); Serial.println(derivative);

    delay(10);
  }
}


// ============================
// Motor Control
// ============================
void init_motor() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void setMotor(int right_speed, int left_speed) {
  // Motor kanan
  if (right_speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (right_speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, abs(right_speed));

  // Motor kiri
  if (left_speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (left_speed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, abs(left_speed));
}


// ============================
// Encoder Update
// ============================
void IRAM_ATTR Read_R() {
  if (digitalRead(encB1) == LOW) val_R++;
  else val_R--;
}

void IRAM_ATTR Read_L() {
  if (digitalRead(encB2) == LOW) val_L++;
  else val_L--;
}

void update_encoder() {
  long d_right = val_R - val_R_prev;
  long d_left = val_L - val_L_prev;
  val_R_prev = val_R;
  val_L_prev = val_L;

  float dTheta = (d_right - d_left) * (wheel_k) / (L * gearbox_ratio * ppr);
  theta += dTheta;

  if (theta > PI) theta -= 2 * PI;
  if (theta < -PI) theta += 2 * PI;
}


// ============================
// IMU Functions
// ============================
void calib_imu() {
  Wire.begin(21, 22, 10000);
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  Serial.println("Calibrating gyro...");
  for (int i = 0; i < calibration_samples; i++) {
    gyroZ_calibration += mpu.getRotationZ();
    delay(3);
  }
  gyroZ_calibration /= calibration_samples;
  Serial.println("Calibration done!");
  last_time = millis();
}

void update_imu() {
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  float gyroZ = (mpu.getRotationZ() - gyroZ_calibration) / 131.0;
  theta_imu -= gyroZ * dt;

  if (theta_imu < 0) theta_imu += 360;
  if (theta_imu >= 360) theta_imu -= 360;
}


// ============================
// Setup & Loop
// ============================
void setup() {
  Serial.begin(115200);

  // Initialize motors
  init_motor();

  // Initialize encoders
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(encA1, Read_R, RISING);
  attachInterrupt(encA2, Read_L, RISING);

  // Initialize IMU
  calib_imu();

  Serial.println("Enter angle in degrees (positive for CCW, negative for CW):");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    target_angle = input.toFloat();

    Serial.print("Rotating to ");
    Serial.print(target_angle);
    Serial.println(" degrees...");

    rotateToAngle(target_angle);
  }
}
