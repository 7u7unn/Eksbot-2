#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Variables for calibration and filtering
const int filter_window_size = 10;
const int calibration_samples = 500;
int filter_index = 0;
bool filter_full = false;
unsigned long last_time;
float theta_imu = 90.0;

// float accelZ_calibration = 0, accelZ_avg = 0, accelZ_values[filter_window_size];
float gyroZ_calibration = 0, gyroZ_avg = 0, gyroZ_values[filter_window_size];

#define encA1 35  //interrupt
#define encB1 36
#define encA2 14  //interrupt
#define encB2 16

volatile long val_R = 0;
volatile long val_L = 0;
long val_R_prev = 0;
long val_L_prev = 0;

// Robot parameters
float ppr = 11.0;
float gearbox_R = 45.0;
float gearbox_L = 45.0;
float L = 26.9;
float diameter = 6.7;
float wheel_k = (PI * diameter);  // cm

// Position variables
float x = 0.0;    // cm
float y = 0.0;    // cm
float theta = PI/2;  // rad

// Ratio tics_to_cm (ttc)
float ttc_R = wheel_k / (gearbox_R * ppr);
float ttc_L = wheel_k / (gearbox_L * ppr);

// Motor control pins
#define ENA 18
#define ENB 17
#define IN1 23
#define IN2 4
#define IN3 13
#define IN4 19

// Motor speed constraints
const int min_speed = 95;
const int max_speed = 255;
int current_pwm_right = 0;
int current_pwm_left = 0;

// Navigation variables
struct TargetPoint {
  float x;
  float y;
  float theta;
  bool active;
};

volatile TargetPoint target = {0, 0, 0, false};

// Navigation parameters
const float dist_tolerance = 1.0;      // cm
const float angular_tolerance = 1.0;   // degrees
const float max_linear_speed = 15.0;   // cm/s
const float max_angular_speed = 30.0;  // deg/s
const float linear_accel = 5.0;        // cm/s^2
const float angular_accel = 60.0;      // deg/s^2
const float kp_linear = 2.0;           // Proportional gain for linear movement
const float kp_angular = 3.0;          // Proportional gain for angular movement
const float max_steer_angle = 10.0;    // Maximum steering angle in degrees

// Navigation state machine
enum NavigationState {
  IDLE,
  TURN_TO_TARGET_ANGLE,
  MOVE_TO_POINT,
  TURN_TO_FINAL_ANGLE,
  COMPLETE
};

volatile NavigationState nav_state = IDLE;

SemaphoreHandle_t odomMutex;
TaskHandle_t taskOdomHandle = NULL;
TaskHandle_t taskSerialHandle = NULL;
TaskHandle_t taskNavigationHandle = NULL;

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
void setTarget(float target_x, float target_y, float target_theta);
bool checkTargetReached();
float normalizeAngle(float angle);
void stop();

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

  // Create mutex
  odomMutex = xSemaphoreCreateMutex();
  
  xTaskCreatePinnedToCore(
    taskOdometry,     // Task function
    "OdometryTask",   // Name
    4096,             // Stack size
    NULL,             // Parameters
    2,                // Priority (higher)
    &taskOdomHandle,  // Task handle
    1                 // Core 1
  );

  // Task for navigation
  xTaskCreatePinnedToCore(
    taskNavigation,    // Task function
    "NavigationTask",  // Name
    4096,              // Stack size
    NULL,              // Parameters
    1,                 // Priority (medium)
    &taskNavigationHandle,  // Task handle
    1                  // Core 1
  );

  // Task for serial printing (lower priority)
  xTaskCreatePinnedToCore(
    taskSerialPrint,   // Task function
    "SerialTask",      // Name
    4096,              // Stack size
    NULL,              // Parameters
    0,                 // Priority (lower)
    &taskSerialHandle, // Task handle
    0                  // Core 0
  );


  setTarget(30.0, 30.0, 0.0);  // Move to (30cm, 30cm) with final heading 0 degrees
}

void loop() {
  // Main loop can be used for other high-level commands
  // delay(100);
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
  Wire.begin(21,22,10000);
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

  theta_imu -= gyroZ_avg * dt;
  if(theta_imu < 0) theta_imu += 360;
  if(theta_imu >= 360) theta_imu -= 360;
  float theta_imu_rad = theta_imu * (PI/180);
}

void update_odom() {
  long d_right_val = val_R - val_R_prev;
  long d_left_val = val_L - val_L_prev;
  val_R_prev = val_R;
  val_L_prev = val_L;

  // Convert ticks (val) to dist
  float dLeft = d_left_val * ttc_L;
  float dRight = d_right_val * ttc_R;

  // Linear and angular displacement
  float dAvg = (dRight + dLeft) / 2.0;
  float dTheta = (dRight - dLeft) / L;

  // Update position
  theta += dTheta;
  if (theta > PI) theta -= (2 * PI);
  if (theta < -PI) theta += (2 * PI);
  x -= dAvg * cos(theta_imu_rad);
  y -= dAvg * sin(theta_imu_rad);
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
  if (spdKiri < 0) {
    spdKiri = constrain(spdKiri, -max_speed, -min_speed);
  } else if (spdKiri > 0) {
    spdKiri = constrain(spdKiri, min_speed, max_speed);
  }

  if (spdKanan < 0) {
    spdKanan = constrain(spdKanan, -max_speed, -min_speed);
  } else if (spdKanan > 0) {
    spdKanan = constrain(spdKanan, min_speed, max_speed);
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
  } else {  // berhenti
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
  } else {  // berhenti
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, constrain(abs(spdKiri), 0, 255));
}

void setTarget(float target_x, float target_y, float target_theta) {
  if (xSemaphoreTake(odomMutex, portMAX_DELAY)) {
    target.x = target_x;
    target.y = target_y;
    target.theta = target_theta;
    target.active = true;
    nav_state = TURN_TO_TARGET_ANGLE;
    xSemaphoreGive(odomMutex);
  }
}

bool checkTargetReached() {
  float dx = target.x - x;
  float dy = target.y - y;
  float dist = sqrt(dx*dx + dy*dy);
  
  if (dist <= dist_tolerance) {
    float angle_diff = normalizeAngle(target.theta - theta_imu);
    if (abs(angle_diff) <= angular_tolerance) {
      return true;
    }
  }
  return false;
}

float normalizeAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle <= -180.0) angle += 360.0;
  return angle;
}

void stop() {
  setMotor(0, 0);
}

// Trapezoidal speed profile calculation
float calculateSpeed(float error, float max_speed, float max_accel, float dt) {
  // Calculate required speed based on error
  float req_speed = min(abs(error) * kp_linear, max_speed);
  
  // Apply acceleration limits
  static float last_speed = 0;
  float max_delta_speed = max_accel * dt;
  
  if (req_speed > last_speed + max_delta_speed) {
    req_speed = last_speed + max_delta_speed;
  } else if (req_speed < last_speed - max_delta_speed) {
    req_speed = last_speed - max_delta_speed;
  }
  
  last_speed = req_speed;
  return req_speed * (error > 0 ? 1 : -1);  // Direction based on error sign
}

void taskNavigation(void *parameter) {
  TickType_t xLastTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz
  unsigned long last_cmd_time = millis();
  const unsigned long timeout = 1000; // 30 seconds timeout
  
  float prev_dist_error = 0;
  float prev_angle_error = 0;
  float last_linear_speed = 0;
  float last_angular_speed = 0;
  
  for (;;) {
    if (xSemaphoreTake(odomMutex, portMAX_DELAY)) {
      if (target.active) {
        float dx = target.x - x;
        float dy = target.y - y;
        float dist_to_target = sqrt(dx*dx + dy*dy);
        float target_angle = atan2(dy, dx) * 180.0 / PI;
        
        switch (nav_state) {
          case TURN_TO_TARGET_ANGLE:
          {
            float angle_error = normalizeAngle(target_angle - theta_imu);
            
            // Check if we've been trying too long
            if (millis() - last_cmd_time > timeout) {
              Serial.println("Navigation timeout - turning to target");
              nav_state = IDLE;
              target.active = false;
              stop();
              break;
            }
            
            if (abs(angle_error) <= angular_tolerance) {
              nav_state = MOVE_TO_POINT;
              last_cmd_time = millis();
              last_linear_speed = 0;
              prev_dist_error = dist_to_target;
            } else {
              // Calculate angular speed with trapezoidal profile
              float dt = (float)(millis() - last_cmd_time) / 1000.0;
              last_cmd_time = millis();
              
              float angular_speed = calculateSpeed(angle_error, max_angular_speed, angular_accel, dt);
              float left_speed = -angular_speed * (L/2.0) / 10.0;  // Convert to PWM units
              float right_speed = angular_speed * (L/2.0) / 10.0;
              
              setMotor((int)right_speed, (int)left_speed);
            }
            break;
          }
          
          case MOVE_TO_POINT:
          {
            float angle_error = normalizeAngle(target_angle - theta_imu);
            float dist_error = dist_to_target;
            
            // Check timeout
            if (millis() - last_cmd_time > timeout) {
              Serial.println("Navigation timeout - moving to point");
              nav_state = IDLE;
              target.active = false;
              stop();
              break;
            }
            
            // Check if we overshot the target (distance increasing)
            if (dist_error > prev_dist_error + 2.0) {  // Allow some tolerance
              Serial.println("Overshot target, stopping");
              nav_state = IDLE;
              target.active = false;
              stop();
              break;
            }
            
            if (dist_error <= dist_tolerance) {
              nav_state = TURN_TO_FINAL_ANGLE;
              last_cmd_time = millis();
              last_angular_speed = 0;
            } else {
              // Calculate linear speed with trapezoidal profile
              float dt = (float)(millis() - last_cmd_time) / 1000.0;
              last_cmd_time = millis();
              
              // Adjust heading if we're off course
              if (abs(angle_error) > max_steer_angle) {
                // First correct orientation
                float angular_speed = calculateSpeed(angle_error, max_angular_speed/2.0, angular_accel/2.0, dt);
                float left_speed = -angular_speed * (L/2.0) / 10.0;
                float right_speed = angular_speed * (L/2.0) / 10.0;
                
                setMotor((int)right_speed, (int)left_speed);
              } else {
                // Move forward while maintaining heading
                float linear_speed = calculateSpeed(dist_error, max_linear_speed, linear_accel, dt);
                
                // Apply heading correction
                float heading_correction = angle_error * 0.5;  // Proportional correction
                float left_speed = linear_speed - heading_correction;
                float right_speed = linear_speed + heading_correction;
                
                setMotor((int)right_speed, (int)left_speed);
              }
            }
            prev_dist_error = dist_error;
            break;
          }
          
          case TURN_TO_FINAL_ANGLE:
          {
            float angle_error = normalizeAngle(target.theta - theta_imu);
            
            // Check timeout
            if (millis() - last_cmd_time > timeout) {
              Serial.println("Navigation timeout - turning to final angle");
              nav_state = IDLE;
              target.active = false;
              stop();
              break;
            }
            
            if (abs(angle_error) <= angular_tolerance) {
              nav_state = COMPLETE;
              stop();
              Serial.println("Target reached!");
            } else {
              // Calculate angular speed with trapezoidal profile
              float dt = (float)(millis() - last_cmd_time) / 1000.0;
              last_cmd_time = millis();
              
              float angular_speed = calculateSpeed(angle_error, max_angular_speed, angular_accel, dt);
              float left_speed = -angular_speed * (L/2.0) / 10.0;
              float right_speed = angular_speed * (L/2.0) / 10.0;
              
              setMotor((int)right_speed, (int)left_speed);
            }
            break;
          }
          
          case COMPLETE:
            stop();
            nav_state = IDLE;
            target.active = false;
            Serial.println("Navigation complete");
            break;
            
          case IDLE:
            stop();
            break;
        }
      } else {
        // No active target, stop the robot
        if (nav_state != IDLE) {
          stop();
          nav_state = IDLE;
        }
      }
      xSemaphoreGive(odomMutex);
    }
    
    vTaskDelayUntil(&xLastTime, xFrequency);
  }
}

void taskOdometry(void *parameter) {
  TickType_t xLastTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); //50Hz 

  for (;;) {
    if (xSemaphoreTake(odomMutex, portMAX_DELAY)) {
      update_odom();
      update_imu();
      xSemaphoreGive(odomMutex);
    }

    vTaskDelayUntil(&xLastTime, xFrequency);
  }
}

void taskSerialPrint(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(200);  // 5Hz (200ms)

  for (;;) {
    if (xSemaphoreTake(odomMutex, portMAX_DELAY)) {
      Serial.print("Encoder R: ");
      Serial.print(val_R);
      Serial.print(" | Encoder L: ");
      Serial.print(val_L);
      Serial.print(" | X: ");
      Serial.print(x, 2);
      Serial.print(" cm | Y: ");
      Serial.print(y, 2);
      Serial.print(" cm | Theta: ");
      Serial.print(theta * (180 / PI), 2);
      Serial.print(" deg | Theta IMU: ");
      Serial.print(theta_imu, 2);
      Serial.print(" deg | State: ");
      Serial.print(nav_state);
      Serial.print(" | Target: (");
      Serial.print(target.x, 1);
      Serial.print(",");
      Serial.print(target.y, 1);
      Serial.print(",");
      Serial.print(target.theta, 1);
      Serial.println(")");
      xSemaphoreGive(odomMutex);

      // Wait for next cycle
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
}