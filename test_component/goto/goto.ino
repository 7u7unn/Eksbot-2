/*
TAMBAHAN FITUR
1. MANUVER
2. POINT LEBIH DEKAT KE GARIS
3. PID TANPA dt
4. LINEFOL
5. ADD LIFT
6. WAYPOINT TO PLACE->MANUVER->PLACE->HOME
MODIFIED: Navigation as function calls instead of continuous waypoint following
*/

#define ENA 18
#define ENB 17
#define IN1 23
#define IN2 4
#define IN3 13
#define IN4 19
void init_motor();
volatile int current_pwm_right = 0;
volatile int current_pwm_left = 0;

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const int filter_window_size = 15;
const int calibration_samples = 500;
int filter_index = 0;
bool filter_full = false;
unsigned long last_time;
float theta_imu = PI / 2;
float theta_fuse = PI / 2;

float gyroZ_calibration = 0, gyroZ_avg = 0, gyroZ_values[filter_window_size];

#define encA1 35
#define encB1 36
#define encA2 14
#define encB2 16

volatile long val_R = 0;
volatile long val_L = 0;
long val_R_prev = 0;
long val_L_prev = 0;

// Robot parameters
float ppr = 11.0;
float gearbox_R = 45.0;
float gearbox_L = 45.0;
float L = 27.5;
float diameter = 6.9;
float wheel_k = (PI * diameter);

// Position variables
float x = 0.0;
float y = 0.0;
float theta = PI / 2;

float percent_imu = 1.0;

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
  MOVING,
  TURN_TO_FINAL_ANGLE,
  IDLE,
  MANUVER
};

NavigationState nav_state = IDLE;

// Navigation control variables
bool nav_active = false;
float target_x_global = 0;
float target_y_global = 0;
float target_theta_global = 0;

float sudut_manuver =0;

bool dir = 1;

// Function declarations
void IRAM_ATTR Read_R();
void IRAM_ATTR Read_L();
void update_odom();
void update_imu();
void calib_imu();
void taskOdometry(void *parameter);
void taskSerialPrint(void *parameter);
void taskNavigation(void *parameter);
void navigate_to_target(float current_x, float current_y, float current_theta_imu, float current_theta_enc);
void gotoXY(float x_target, float y_target, float theta_target);
void manuver();
void stopNav();
bool isNavigationComplete();

// PID for ROTATION (angular control)
float Kp_rot = 3.2;
float Ki_rot = 0.03;  //0.4
float Kd_rot = 1;     //0.6
float integral_rot = 0.0;
float prev_error_rot = 0.0;

// PID for LINEAR SPEED (distance-based speed control)
float Kp_linear = 5.5;
float Ki_linear = 0.004;  //0.04
float Kd_linear = 0.01;   //0.1
float integral_linear = 0.0;
float prev_error_linear = 0.0;

// PID for ANGULAR CORRECTION while moving
float Kp_angular = 2.2;
float Ki_angular = 0.001;  //0.01
float Kd_angular = 0.01;   //0.1
float integral_angular = 0.0;
float prev_error_angular = 0.0;

unsigned long last_pid_time = 0;

// Speed constraints
float distance_threshold = 1.0;  // cm
float angle_threshold = 0.5;     // 0.5 degrees
float rotation_speed = 150;
float max_speed = 200;
float min_speed = 120;

// windup limits
float integral_max = 2000.0;
float integral_min = -2000.0;

// LINE FOLLOWING
//Mux control pins
int s0 = 25;
int s1 = 33;
int s2 = 32;
int s3 = 26;
int SIG_pin = 34;

float normalize_angle(float angle);
float calculate_distance(float x1, float y1, float x2, float y2);
void reset(float x_u, float y_u, int dir);

void setup() {
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  attachInterrupt(encA1, Read_R, RISING);

  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(encA2, Read_L, RISING);

  Serial.begin(115200);
  calib_imu();
  init_motor();
  last_pid_time = millis();

  odomMutex = xSemaphoreCreateMutex();
  navMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    taskOdometry,
    "OdometryTask",
    4096,
    NULL,
    2,
    &taskOdomHandle,
    1);

  xTaskCreatePinnedToCore(
    taskNavigation,
    "NavigationTask",
    4096,
    NULL,
    2,
    &taskNavHandle,
    1);

  xTaskCreatePinnedToCore(
    taskSerialPrint,
    "SerialTask",
    4096,
    NULL,
    0,
    &taskSerialHandle,
    0);
}

void loop() {

  // Go to position
  gotoXY(-70.0, 165.0, 90.0);
  while (!isNavigationComplete()) delay(100);
  
  manuver(270);
  while (!isNavigationComplete()) delay(100);
  
  // Reset position
  reset(-42, 160);
  
  // Go to position
  gotoXY(-42, 20, 90);
  while (!isNavigationComplete()) delay(100);
  
  manuver(90);
  while (!isNavigationComplete()) delay(100);
  
  reset(-42, 23);
  
  //home 
  gotoXY(0, 0, 90);
  while (!isNavigationComplete()) delay(100);
  

  delay(5000);
  
}

float rad2deg(float i) {
  return (180 / PI) * i;
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
  Wire.begin(21, 22, 10000);
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  while (1) {
    if (mpu.testConnection()) {
      Serial.println("MPU6050 connection successful");
      break;
    } else {
      Serial.println("MPU6050 connection failed");
    }
  }

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

  if (filter_index == 0) {
    filter_full = true;
  }

  gyroZ_avg = 0;
  int count = filter_full ? filter_window_size : filter_index;
  for (int i = 0; i < count; i++) {
    gyroZ_avg += gyroZ_values[i];
  }
  gyroZ_avg /= count;

  theta_imu -= (PI / 180) * gyroZ_avg * dt;
  if (theta_imu > PI) theta_imu -= (2 * PI);
  if (theta_imu < -PI) theta_imu += (2 * PI);
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

  // Update position
  theta += dTheta;
  if (theta > PI) theta -= (2 * PI);
  if (theta < -PI) theta += (2 * PI);

  x -= dAvg * cos(theta);
  y -= dAvg * sin(theta);
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
  // Apply constraints with deadzone compensation
  if (spdKiri != 0) {
    if (spdKiri > 0) {
      spdKiri = constrain(spdKiri, min_speed, max_speed);
    } else {
      spdKiri = constrain(spdKiri, -max_speed, -min_speed);
    }
  }

  if (spdKanan != 0) {
    if (spdKanan > 0) {
      spdKanan = constrain(spdKanan, min_speed, max_speed);
    } else {
      spdKanan = constrain(spdKanan, -max_speed, -min_speed);
    }
  }

  // Store PWM values for monitoring
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
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void taskOdometry(void *parameter) {
  TickType_t xLastTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  for (;;) {
    if (xSemaphoreTake(odomMutex, portMAX_DELAY)) {
      update_imu();
      update_odom();
      xSemaphoreGive(odomMutex);
    }
    vTaskDelayUntil(&xLastTime, xFrequency);
  }
}

void taskNavigation(void *parameter) {
  TickType_t xLastTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);  // 50Hz

  for (;;) {
    if (!nav_active) {
      // If navigation is not active, just idle
      setMotor(0, 0);
      vTaskDelayUntil(&xLastTime, xFrequency);
      continue;
    }

    float local_x, local_y, local_theta_imu, local_theta_enc;
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(10))) {
      local_x = x;
      local_y = y;
      local_theta_imu = rad2deg(theta_imu);
      local_theta_enc = rad2deg(theta);
      xSemaphoreGive(odomMutex);
    } else {
      vTaskDelayUntil(&xLastTime, xFrequency);
      continue;
    }

    if (xSemaphoreTake(navMutex, portMAX_DELAY)) {
      navigate_to_target(local_x, local_y, local_theta_imu, local_theta_enc);
      xSemaphoreGive(navMutex);
    }

    vTaskDelayUntil(&xLastTime, xFrequency);
  }
}

void navigate_to_target(float current_x, float current_y, float current_theta_imu, float current_theta_enc) {
  if (!nav_active) {
    setMotor(0, 0);
    return;
  }

  float target_x = target_x_global;
  float target_y = target_y_global;
  float target_theta = target_theta_global;

  // Calculate distance and angle to target
  float error_x = target_x - current_x;
  float error_y = target_y - current_y;
  float distance = calculate_distance(current_x, current_y, target_x, target_y);
  float angle_to_target = rad2deg(atan2(error_y, error_x));
  float angular_error = normalize_angle(angle_to_target - current_theta_imu);
  float current_theta = current_theta_enc;

  switch (nav_state) {
    case ROTATING:
      {
        if (abs(angular_error) > angle_threshold) {
          integral_rot += angular_error;
          integral_rot = constrain(integral_rot, integral_min, integral_max);

          float derivative_rot = (angular_error - prev_error_rot);
          prev_error_rot = angular_error;

          float pid_output = (Kp_rot * angular_error) + (Ki_rot * integral_rot) + (Kd_rot * derivative_rot);
          int rotation_correction = constrain((int)(pid_output), -rotation_speed, rotation_speed);

          if (rotation_correction > 0 && rotation_correction < min_speed) rotation_correction = min_speed;
          else if (rotation_correction < 0 && rotation_correction > -min_speed) rotation_correction = -min_speed;

          setMotor(rotation_correction, -rotation_correction);
        } else {
          nav_state = MOVING;
          integral_rot = 0;
          integral_linear = 0;
          integral_angular = 0;
          prev_error_rot = 0;
          prev_error_linear = 0;
          prev_error_angular = 0;
          setMotor(0, 0);
          delay(150);
        }
        break;
      }

    case MOVING:
      {
        if (distance < distance_threshold) {
          nav_state = TURN_TO_FINAL_ANGLE;
          integral_rot = 0;
          integral_linear = 0;
          integral_angular = 0;
          prev_error_rot = 0;
          prev_error_linear = 0;
          prev_error_angular = 0;
          setMotor(0, 0);
          delay(200);
          break;
        }

        float distance_error = distance;
        integral_linear += distance_error;
        integral_linear = constrain(integral_linear, integral_min, integral_max);

        float derivative_linear = (distance_error - prev_error_linear);
        prev_error_linear = distance_error;

        float linear_speed = (Kp_linear * distance_error) + (Ki_linear * integral_linear) + (Kd_linear * derivative_linear);
        linear_speed = constrain(linear_speed, min_speed, max_speed);

        integral_angular += angular_error;
        integral_angular = constrain(integral_angular, integral_min, integral_max);

        float derivative_angular = (angular_error - prev_error_angular);
        prev_error_angular = angular_error;

        float angular_correction = (Kp_angular * angular_error) + (Ki_angular * integral_angular) + (Kd_angular * derivative_angular);

        float left_speed = linear_speed - angular_correction;
        float right_speed = linear_speed + angular_correction;

        if (left_speed > 0) {
          left_speed = constrain(left_speed, min_speed, max_speed);
        } else if (left_speed < 0) {
          left_speed = constrain(left_speed, -max_speed, -min_speed);
        }

        if (right_speed > 0) {
          right_speed = constrain(right_speed, min_speed, max_speed);
        } else if (right_speed < 0) {
          right_speed = constrain(right_speed, -max_speed, -min_speed);
        }

        setMotor(right_speed, left_speed);
        break;
      }

    case TURN_TO_FINAL_ANGLE:
      {
        float final_angle_error = normalize_angle(target_theta - current_theta_imu);

        if (abs(final_angle_error) > angle_threshold) {
          integral_rot += final_angle_error;
          integral_rot = constrain(integral_rot, integral_min, integral_max);

          float derivative_rot = final_angle_error - prev_error_rot;
          prev_error_rot = final_angle_error;

          float pid_output = (Kp_rot * final_angle_error) + (Ki_rot * integral_rot) + (Kd_rot * derivative_rot);
          int rotation_correction = constrain((int)(pid_output), -rotation_speed, rotation_speed);

          if (rotation_correction > 0 && rotation_correction < min_speed) rotation_correction = min_speed;
          else if (rotation_correction < 0 && rotation_correction > -min_speed) rotation_correction = -min_speed;

          setMotor(rotation_correction, -rotation_correction);
        } else {
          // Navigation complete
          setMotor(0, 0);
          nav_active = false;
          nav_state = IDLE;
          
          integral_rot = 0;
          integral_linear = 0;
          integral_angular = 0;
          prev_error_rot = 0;
          prev_error_linear = 0;
          prev_error_angular = 0;
        }
        break;
      }

    case MANUVER:
      {
        float teta = current_theta_imu;
        if (teta <= 0) {
          teta += 360;
        }
        float final_angle_error = sudut_manuver - teta;

        if (abs(final_angle_error) > angle_threshold) {
          integral_rot += final_angle_error;
          integral_rot = constrain(integral_rot, integral_min, integral_max);

          float derivative_rot = (final_angle_error - prev_error_rot);
          prev_error_rot = final_angle_error;

          float pid_output = (Kp_rot * final_angle_error) + (Ki_rot * integral_rot) + (Kd_rot * derivative_rot);
          int rotation_correction = constrain((int)(pid_output), -255, 255);

          if (rotation_correction > 0 && rotation_correction < min_speed) rotation_correction = min_speed;
          else if (rotation_correction < 0 && rotation_correction > -min_speed) rotation_correction = -min_speed;

          setMotor(0, -rotation_correction);
        } else {
          // Maneuver complete
          setMotor(0, 0);
          nav_active = false;
          nav_state = IDLE;
          
          integral_rot = 0;
          integral_linear = 0;
          integral_angular = 0;
          prev_error_rot = 0;
          prev_error_linear = 0;
          prev_error_angular = 0;
        }
        break;
      }

    case IDLE:
      {
        setMotor(0, 0);
        nav_active = false;
        break;
      }
  }
}


void gotoXY(float x_target, float y_target, float theta_target) {
  if (xSemaphoreTake(navMutex, portMAX_DELAY)) {
    target_x_global = x_target;
    target_y_global = y_target;
    target_theta_global = theta_target;
    
    nav_state = ROTATING;
    nav_active = true;
    
    // Reset PID variables
    integral_rot = 0;
    integral_linear = 0;
    integral_angular = 0;
    prev_error_rot = 0;
    prev_error_linear = 0;
    prev_error_angular = 0;
    
    xSemaphoreGive(navMutex);
  }
}

// Function to perform maneuver
void manuver(float sudut) {
  if (xSemaphoreTake(navMutex, portMAX_DELAY)) {
    sudut_manuver = sudut;
    nav_state = MANUVER;
    nav_active = true;

    
    // Reset PID variables
    integral_rot = 0;
    integral_linear = 0;
    integral_angular = 0;
    prev_error_rot = 0;
    prev_error_linear = 0;
    prev_error_angular = 0;
    
    xSemaphoreGive(navMutex);
  }
}

// Function to stop navigation
void stopNav() {
  if (xSemaphoreTake(navMutex, portMAX_DELAY)) {
    nav_active = false;
    nav_state = IDLE;
    setMotor(0, 0);
    xSemaphoreGive(navMutex);
  }
}

// Function to check if navigation is complete
bool isNavigationComplete() {
  bool complete = false;
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(10))) {
    complete = !nav_active;
    xSemaphoreGive(navMutex);
  }
  return complete;
}

void taskSerialPrint(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(200);

  for (;;) {
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(10))) {
      Serial.print("X: ");
      Serial.print(x, 2);
      Serial.print(" | Y: ");
      Serial.print(y, 2);
      Serial.print(" | θ: ");
      Serial.print(theta_imu * (180 / PI), 2);
      Serial.print("° | PWM_R: ");
      Serial.print(current_pwm_right);
      Serial.print(" | PWM_L: ");
      Serial.print(current_pwm_left);

      // Calculate and print current distance to target if nav is active
      if (nav_active) {
        float dist = calculate_distance(x, y, target_x_global, target_y_global);
        Serial.print(" | Dist: ");
        Serial.print(dist, 1);
        Serial.print("cm");
      }

      Serial.print(" | State: ");

      if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(5))) {
        switch (nav_state) {
          case ROTATING: Serial.print("ROT"); break;
          case MOVING: Serial.print("MOV"); break;
          case TURN_TO_FINAL_ANGLE: Serial.print("FIN"); break;
          case IDLE: Serial.print("IDLE"); break;
          case MANUVER: Serial.print("MANUVER"); break;
        }
        Serial.print(" | Active: ");
        Serial.println(nav_active ? "YES" : "NO");
        xSemaphoreGive(navMutex);
      } else {
        Serial.println("---");
      }

      xSemaphoreGive(odomMutex);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void reset(float x_u, float y_u) {
  if (xSemaphoreTake(odomMutex, portMAX_DELAY)) {
    x = x_u;
    y = y_u;
    xSemaphoreGive(odomMutex);
  }
}