#define ENA 18
#define ENB 17
#define IN1 23
#define IN2 4
#define IN3 13
#define IN4 19

#define encA1 35
#define encB1 36
#define encA2 14
#define encB2 16

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
bool finish = false;

const int filter_window_size = 5;
const int calibration_samples = 700;
int filter_index = 0;
bool filter_full = false;
unsigned long last_time;
float theta_imu = PI / 2;
float theta_fuse = PI / 2;

float gyroZ_calibration = 0, gyroZ_avg = 0, gyroZ_values[filter_window_size];

volatile long val_R = 0;
volatile long val_L = 0;
long val_R_prev = 0;
long val_L_prev = 0;

// Robot parameters
float ppr = 11.0;
float gearbox_R = 49.0;
float gearbox_L = 50.0;
float L = 30.0;        // Jarak antar roda (cm)
float diameter = 7.0;  // Diameter roda (cm)
float wheel_k = (PI * diameter);

// Position variables
float x = 0.0;
float y = 0.0;
float theta = PI / 2;

// Ratio tics_to_cm (ttc)
float ttc_R = wheel_k / (gearbox_R * ppr);
float ttc_L = wheel_k / (gearbox_L * ppr);

SemaphoreHandle_t odomMutex;
SemaphoreHandle_t navMutex;

TaskHandle_t taskOdomHandle = NULL;
TaskHandle_t taskSerialHandle = NULL;
TaskHandle_t taskNavHandle = NULL;

enum NavigationState {
  ROTATING,
  MOVING_STRAIGHT,
  IDLE,
  MANUVER
};

NavigationState nav_state = IDLE;

bool command_active = false;
float command_target_distance = 0;
float command_target_angle = 0;
float initial_heading = 0;  // For maintaining heading during straight movement
float start_x = 0;
float start_y = 0;

enum CommandType {
  CMD_NONE,
  CMD_STRAIGHT,
  CMD_ROTATE,
  CMD_MANUVER,
  CMD_IDLE
};
CommandType current_command = CMD_NONE;

// Add near other global vars
float manuver_start_angle = 0;  // Starting IMU angle when maneuver begins
float manuver_target_delta = 180.0; // Always 180° CCW

// PID for ROTATION
float Kp_rot = 2;
float Ki_rot = 0.0;
float Kd_rot = 3.5;
float integral_rot = 0.0;
float prev_error_rot = 0.0;

// PID for LINEAR SPEED
float Kp_linear = 1.2;
float Ki_linear = 0.004;
float Kd_linear = 0.02;
float integral_linear = 0.0;
float prev_error_linear = 0.0;

// PID for ANGULAR CORRECTION (heading correction during straight movement)
float Kp_angular = 4.4;
float Ki_angular = 0.0;
float Kd_angular = 1;
float integral_angular = 0.0;
float prev_error_angular = 0.0;

// Speed constraints
float distance_threshold = 1.0;  // cm
float angle_threshold = 0.5;     // degrees
float rotation_speed = 150;
float max_speed = 170;
float min_speed = 110;

// windup limits
float integral_max = 2000.0;
float integral_min = -2000.0;

volatile int current_pwm_right = 0;
volatile int current_pwm_left = 0;

// Function declarations
void IRAM_ATTR Read_R();
void IRAM_ATTR Read_L();
void update_odom();
void update_imu();
void calib_imu();
void taskOdometry(void *parameter);
void taskSerialPrint(void *parameter);
void taskNavigation(void *parameter);
void init_motor();
void setMotor(int spdKanan, int spdKiri);

void straight(float distance);
void rotate(float degrees);
void idle();
bool is_command_done();

void execute_straight(float current_x, float current_y, float current_theta_enc);
void execute_rotate(float current_theta_imu);

float normalize_angle(float angle);
float calculate_distance(float x1, float y1, float x2, float y2);
float rad2deg(float i);

// ==================== SETUP & LOOP ====================

void setup() {
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  attachInterrupt(encA1, Read_R, RISING);

  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(encA2, Read_L, RISING);

  Serial.begin(115200);
  init_motor();
  uint16_t start_time = millis();
  while(millis()-start_time < 6000){
    continue;
  }
  calib_imu();

  odomMutex = xSemaphoreCreateMutex();
  navMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskOdometry, "OdometryTask", 4096, NULL, 2, &taskOdomHandle, 1);
  xTaskCreatePinnedToCore(taskNavigation, "NavigationTask", 4096, NULL, 2, &taskNavHandle, 1);
  // uint16_t start_time = millis();
  xTaskCreatePinnedToCore(taskSerialPrint, "SerialTask", 4096, NULL, 0, &taskSerialHandle, 0);

  
}

void loop() {
  // Example usage:
  // Rotate to absolute angles and move straight
  if(!finish){

  // delay(1000);
  // rotate(135);
  // delay(1000);
  // rotate(180);
  // delay(1000);
  straight(180, 90);  // Move forward 50 cm
  // manuver();
  delay(1000);
  straight(120, -90);
  delay(1000);
  rotate(90);
  // delay(1000);
  // rotate(90);  // Rotate to 90° (East) - absolute angle
  // delay(1000);
  // manuver();
  finish = true;
  }

  // rotate(135);  // Rotate to 0° (North) - absolute angle
  // rotate(0);
  // delay(1000);;
  // straight(120);
  // delay(1000);
  // rotate(90);

  // delay(1000);
  // straight(50);  // Move forward 50 cm

  // // rotate(180);   // Rotate to 180° (South) - absolute angle
  // // delay(1000);

  // straight(-60);  // Move forward 50 cm
  // delay(1000);

  // rotate(90);   // Rotate to -90° (West) - absolute angle
  // delay(1000);

  // straight(50);  // Move forward 50 cm
  // delay(1000);

  // vTaskSuspend(NULL);  // Hentikan loop utama
}

// ==================== HELPER FUNCTIONS ====================

float rad2deg(float i) {
  return (180.0 / PI) * i;
}

void IRAM_ATTR Read_R() {
  if (digitalRead(encB1) == LOW) val_R += 1;
  else val_R -= 1;
}

void IRAM_ATTR Read_L() {
  if (digitalRead(encB2) == LOW) val_L += 1;
  else val_L -= 1;
}

void calib_imu() {
  Wire.begin(21, 22, 100000);  // SCL=21, SDA=22
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  while (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    delay(500);
  }
  Serial.println("MPU6050 connection successful");

  Serial.println("Calibrating gyro Z...");
  for (int i = 0; i < calibration_samples; i++) {
    gyroZ_calibration += mpu.getRotationZ();
    delay(3);
  }
  gyroZ_calibration /= calibration_samples;
  Serial.println("Calibration complete");
  Serial.println(gyroZ_calibration);

  for (int i = 0; i < filter_window_size; i++) {
    gyroZ_values[i] = 0;
  }

  last_time = millis();
}

void update_imu() {
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  int16_t gyroZ_raw = mpu.getRotationZ();
  float gyroZ = (gyroZ_raw - gyroZ_calibration) / 131.0;
    gyroZ_values[filter_index] = gyroZ;
  filter_index = (filter_index + 1) % filter_window_size;
  if (filter_index == 0) filter_full = true;

  gyroZ_avg = 0;
  int count = filter_full ? filter_window_size : filter_index;
  for (int i = 0; i < count; i++) {
    gyroZ_avg += gyroZ_values[i];
  }
  gyroZ_avg /= count;

  theta_imu -= (PI / 180.0) * gyroZ_avg * dt;
  if (theta_imu >= 2 * PI) theta_imu -= 2 * PI;
  if (theta_imu < 0) theta_imu += 2 * PI;
}

void update_odom() {
  long d_right_val = val_R - val_R_prev;
  long d_left_val = val_L - val_L_prev;
  val_R_prev = val_R;
  val_L_prev = val_L;

  float dLeft = d_left_val * ttc_L;
  float dRight = d_right_val * ttc_R;

  float dAvg = (dRight + dLeft) / 2.0;
  float dTheta = (dRight - dLeft) / L;

  theta += dTheta;
  if (theta >= 2 * PI) theta -= 2 * PI;
  if (theta < 0) theta += 2 * PI;

  x -= dAvg * cos(theta_imu);
  y -= dAvg * sin(theta_imu);
}

void init_motor() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void setMotor(int spdKanan, int spdKiri) {
  // Batasi kecepatan
  // if (spdKiri != 0) {
  //   if (spdKiri > 0) spdKiri = constrain(spdKiri, min_speed, max_speed);
  //   else spdKiri = constrain(spdKiri, -max_speed, -min_speed);
  // }
  // if (spdKanan != 0) {
  //   if (spdKanan > 0) spdKanan = constrain(spdKanan, min_speed, max_speed);
  //   else spdKanan = constrain(spdKanan, -max_speed, -min_speed);
  // }

  current_pwm_right = spdKanan;
  current_pwm_left = spdKiri;

  // Motor kanan
  if (spdKanan > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (spdKanan < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, constrain(abs(spdKanan), 0, 255));

  // Motor kiri
  if (spdKiri > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (spdKiri < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, constrain(abs(spdKiri), 0, 255));
}

float normalize_angle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

float calculate_distance(float x1, float y1, float x2, float y2) {
  return sqrt(sq(x2 - x1) + sq(y2 - y1));
}

// ==================== TASKS ====================

void taskOdometry(void *parameter) {
  TickType_t xLastTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  for (;;) {
    if (xSemaphoreTake(odomMutex, portMAX_DELAY) == pdTRUE) {
      update_imu();
      update_odom();
      xSemaphoreGive(odomMutex);
    }
    vTaskDelayUntil(&xLastTime, xFrequency);
  }
}

void taskNavigation(void *parameter) {
  TickType_t xLastTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(15);

  for (;;) {
    float local_x, local_y, local_theta_imu, local_theta_enc;
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      local_x = x;
      local_y = y;
      local_theta_imu = rad2deg(theta_imu);
      local_theta_enc = rad2deg(theta);
      xSemaphoreGive(odomMutex);
    } else {
      vTaskDelayUntil(&xLastTime, xFrequency);
      continue;
    }

    if (xSemaphoreTake(navMutex, portMAX_DELAY) == pdTRUE) {
      if (command_active) {
        switch (current_command) {
          case CMD_STRAIGHT:
            execute_straight(local_x, local_y, local_theta_imu);
            break;
          case CMD_ROTATE:
            execute_rotate(local_theta_imu);
            break;
          case CMD_IDLE:
            setMotor(0, 0);
            break;
          case CMD_MANUVER:
             exec_manuver(local_theta_imu);

          default:
            break;
        }
      }
      xSemaphoreGive(navMutex);
    }

    vTaskDelayUntil(&xLastTime, xFrequency);
  }
}

void taskSerialPrint(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(200);

  for (;;) {
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      Serial.print("Encoder R: ");
      Serial.print(val_R);
      Serial.print(" | Encoder L: ");
      Serial.print(val_L);
      Serial.print(" | X: ");
      Serial.print(x, 2);
      Serial.print(" cm | Y: ");
      Serial.print(y, 2);
      Serial.print(" cm | Theta: ");
      Serial.print(rad2deg(theta), 2);
      Serial.print(" deg | Theta IMU: ");
      Serial.print(rad2deg(theta_imu), 2);
      Serial.print(" deg | PWM_R: ");
      Serial.print(current_pwm_right);
      Serial.print(" | PWM_L: ");
      Serial.print(current_pwm_left);
      Serial.print(" | State: ");

      if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        switch (nav_state) {
          case ROTATING: Serial.print("ROTATING"); break;
          case MOVING_STRAIGHT: Serial.print("MOVING_STRAIGHT"); break;
          case IDLE: Serial.print("IDLE"); break;
        }
        Serial.println();
        xSemaphoreGive(navMutex);
      } else {
        Serial.println("---");
      }

      xSemaphoreGive(odomMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ==================== COMMAND FUNCTIONS ====================

void straight(float distance, float heading) {
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
      // Store starting position and heading (using encoder theta)
      start_x = x;
      start_y = y;
      initial_heading = heading;  // Use encoder theta for straight movement
      xSemaphoreGive(odomMutex);
    }

    command_target_distance = distance;
    current_command = CMD_STRAIGHT;
    command_active = true;
    nav_state = MOVING_STRAIGHT;

    // Reset PID integrals
    integral_linear = integral_angular = 0;
    prev_error_linear = prev_error_angular = 0;

    xSemaphoreGive(navMutex);
  }

  while (!is_command_done()) {
    delay(100);
  }
}

// void manuver()

void rotate(float degrees) {
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    // Set target angle as absolute global angle (not relative)
    command_target_angle = degrees;

    // Normalize to -180 to 180 range
    while (command_target_angle > 180) command_target_angle -= 360;
    while (command_target_angle < -180) command_target_angle += 360;

    current_command = CMD_ROTATE;
    command_active = true;
    nav_state = ROTATING;

    // Reset PID integrals
    integral_rot = 0;
    prev_error_rot = 0;

    xSemaphoreGive(navMutex);
  }

  while (!is_command_done()) {
    delay(100);
  }
}
void manuver() {  // No parameter needed!
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    // Capture current IMU heading as start
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
      manuver_start_angle = rad2deg(theta_imu);
      xSemaphoreGive(odomMutex);
    }

    current_command = CMD_MANUVER;
    command_active = true;
    nav_state = MANUVER;

    // Reset PID
    integral_rot = 0;
    prev_error_rot = 0;

    xSemaphoreGive(navMutex);
  }

  while (!is_command_done()) {
    delay(100);
  }
}
void idle() {
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
    current_command = CMD_IDLE;
    command_active = false;
    nav_state = IDLE;
    setMotor(0, 0);
    xSemaphoreGive(navMutex);
  }
}

bool is_command_done() {
  bool done = false;
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    done = !command_active;
    xSemaphoreGive(navMutex);
  }
  return done;
}

// ==================== EXECUTION FUNCTIONS ====================

void execute_straight(float current_x, float current_y, float current_theta_enc) {
  // Calculate distance traveled from start position
  float distance_traveled = calculate_distance(start_x, start_y, current_x, current_y);
  float remaining_distance = abs(command_target_distance) - distance_traveled;

  // Determine direction (forward or backward)
  int direction = (command_target_distance > 0) ? 1 : -1;

  // Check if reached target
  if (remaining_distance < distance_threshold) {
    setMotor(0, 0);
    command_active = false;
    nav_state = IDLE;
    integral_linear = integral_angular = 0;
    prev_error_linear = prev_error_angular = 0;
    return;
  }

  // PID for linear speed based on remaining distance
  integral_linear += remaining_distance;
  integral_linear = constrain(integral_linear, integral_min, integral_max);
  float derivative_linear = (remaining_distance - prev_error_linear);
  prev_error_linear = remaining_distance;
  float linear_speed = (Kp_linear * remaining_distance) + (Ki_linear * integral_linear) + (Kd_linear * derivative_linear);
  linear_speed = constrain(linear_speed, min_speed, max_speed) * direction;

  // PID for heading correction using ENCODER theta (keep initial heading)
  float heading_error = normalize_angle(initial_heading - current_theta_enc);
  integral_angular += heading_error;
  integral_angular = constrain(integral_angular, integral_min, integral_max);
  float derivative_angular = (heading_error - prev_error_angular);
  prev_error_angular = heading_error;
  float angular_correction = (Kp_angular * heading_error) + (Ki_angular * integral_angular) + (Kd_angular * derivative_angular);

  // Calculate individual wheel speeds
  float left_speed = linear_speed - angular_correction;
  float right_speed = linear_speed + angular_correction;

  // Constrain speeds
  if (left_speed > 0) left_speed = constrain(left_speed, min_speed, max_speed);
  else if (left_speed < 0) left_speed = constrain(left_speed, -max_speed, -min_speed);
  if (right_speed > 0) right_speed = constrain(right_speed, min_speed, max_speed);
  else if (right_speed < 0) right_speed = constrain(right_speed, -max_speed, -min_speed);

  setMotor(right_speed, left_speed);
}

void execute_rotate(float current_theta_imu) {
  float angle_error = normalize_angle(command_target_angle - current_theta_imu);

  if (abs(angle_error) > angle_threshold) {
    integral_rot += angle_error;
    integral_rot = constrain(integral_rot, integral_min, integral_max);

    float derivative_rot = (angle_error - prev_error_rot);
    prev_error_rot = angle_error;

    float pid_output = (Kp_rot * angle_error) + (Ki_rot * integral_rot) + (Kd_rot * derivative_rot);
    int rotation_correction = constrain((int)pid_output, -rotation_speed, rotation_speed);

    if (rotation_correction > 0 && rotation_correction < 92) rotation_correction = 92;
    else if (rotation_correction < 0 && rotation_correction > -92) rotation_correction = -92;

    setMotor(rotation_correction, -rotation_correction);
  } else {
    setMotor(0, 0);
    command_active = false;
    nav_state = IDLE;
    integral_rot = 0;
    prev_error_rot = 0;
  }
}void exec_manuver(float current_theta_imu) {
  // Compute how much we've turned CCW from start
  float delta = current_theta_imu - manuver_start_angle;
  
  // Normalize delta to [-180, 180], but we want positive CCW
  // Since we're turning CCW, delta should increase
  // Handle wrap-around (e.g., from 170° to -170° = +20° turn)
  if (delta < -180) delta += 360;
  if (delta > 180) delta -= 360;

  // We want +180° turn (CCW), so error = 180 - current_delta
  float angle_error = manuver_target_delta - delta;

  if (abs(angle_error) > angle_threshold) {
    integral_rot += angle_error;
    integral_rot = constrain(integral_rot, integral_min, integral_max);

    float derivative_rot = (angle_error - prev_error_rot);
    prev_error_rot = angle_error;

    float pid_output = (Kp_rot * angle_error) + (Ki_rot * integral_rot) + (Kd_rot * derivative_rot);
    int left_speed = constrain((int)pid_output, min_speed, max_speed);

    // RIGHT WHEEL STOPPED, LEFT WHEEL MOVES FORWARD (CCW turn)
    setMotor(0, -left_speed);  // Right=0, Left=positive
  } else {
    setMotor(0, 0);
    command_active = false;
    nav_state = IDLE;
    integral_rot = 0;
    prev_error_rot = 0;
  }
}