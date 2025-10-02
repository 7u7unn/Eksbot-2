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

// Variables for calibration and filtering
const int filter_window_size = 10;
const int calibration_samples = 700;
int filter_index = 0;
bool filter_full = false;
unsigned long last_time;
float theta_imu = PI / 2;
float theta_fuse = PI / 2;
// float theta = 90.0;

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
float x = 0.0;         // cm
float y = 0.0;         // cm
float theta = PI / 2;  // rad

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
  MOVING
};

NavigationState nav_state = ROTATING;
float angle_threshold = 5.0 * (PI / 180.0);
float rotation_speed = 160;

// Function declarations - ADDED missing taskIMU declaration
void IRAM_ATTR Read_R();
void IRAM_ATTR Read_L();
void update_odom();
void update_imu();
void calib_imu();
void taskOdometry(void *parameter);
void taskSerialPrint(void *parameter);

struct Waypoint {
  float x;
  float y;
};

#define MAX_WAYPOINTS 5
Waypoint waypoints[MAX_WAYPOINTS] = {
  { 0.0, 60.0 },  // Waypoint coordinates in cm
  { 60.0, 60.0 },
  { 60.0, 0.0 },
  { 0.0, 0.0 }
};
int current_waypoint = 0;
int total_waypoints = 4;

// PID Control parameters
float Kp = 0.5;
float Ki = 0.0;
float Kd = 0.05;
float prev_error = 0.0;
float integral = 0.0;
unsigned long last_pid_time = 0;

// Navigation parameters
float distance_threshold = 10.0;  // cm
float base_speed = 150;           // PWM value
float max_speed = 200;            // PWM value
float min_speed = 110;            // PWM value

// Add these function declarations
void navigate_to_waypoint();
float normalize_angle(float angle);
float calculate_distance(float x1, float y1, float x2, float y2);

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

  // Create mutex
  odomMutex = xSemaphoreCreateMutex();
  navMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    taskOdometry,     // Task function
    "OdometryTask",   // Name
    4096,             // Stack size
    NULL,             // Parameters
    2,                // Priority (higher)
    &taskOdomHandle,  // Task handle
    1                 // Core 1
  );
  xTaskCreatePinnedToCore(
    taskNavigation,
    "NavigationTask",
    4096,
    NULL,
    2,  // Medium priority
    &taskNavHandle,
    1  // Core 1 (same core for better cache usage)
  );

  // Task for serial printing (lower priority)
  xTaskCreatePinnedToCore(
    taskSerialPrint,    // Task function
    "SerialTask",       // Name
    4096,               // Stack size
    NULL,               // Parameters
    0,                  // Priority (lower)
    &taskSerialHandle,  // Task handle
    0);
}

void loop() {
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
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  //bisa ganti ke 500, 1000, 2000
    // mpu.setSleepEnabled(false);

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

// Ambil nilai-nilai IMU
void update_imu() {
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  int16_t gyroZ_raw = mpu.getRotationZ();
  float gyroZ = (gyroZ_raw - gyroZ_calibration) / 131.0;  //

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
  //   float theta_imu_rad = theta_imu*(PI/180);
  // Serial.println(gyroZ_raw);
  // Serial.print("Deg: ");
  // Serial.print(theta);
  // Serial.print(" ");
  // Serial.print("Rad: ");
  // Serial.println(theta_rad);
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
  ;
  // theta_fuse = (0.4 * theta) + (0.6 * theta_imu);
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


float normalize_angle(float angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

float calculate_distance(float x1, float y1, float x2, float y2) {
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// void navigate_to_waypoint() {
//   if (current_waypoint >= total_waypoints) {
//     setMotor(0, 0);  // Stop robot
//     return;
//   }

//   float target_x = waypoints[current_waypoint].x;
//   float target_y = waypoints[current_waypoint].y;

//   // Calculate distance and angle to target
//   float error_x = target_x - x;
//   float error_y = target_y - y;
//   float distance = calculate_distance(x, y, target_x, target_y);
//   float angle_to_target = atan2(error_y, error_x);

//   // If reached waypoint
//   if (distance < distance_threshold) {
//     current_waypoint++;
//     integral = 0;  // Reset PID
//     prev_error = 0;
//     return;
//   }

//   // PID for angular control
//   float angular_error = normalize_angle(angle_to_target - theta);
//   unsigned long current_time = millis();
//   float dt = (current_time - last_pid_time) / 1000.0;
//   last_pid_time = current_time;

//   integral += angular_error * dt;
//   float derivative = (angular_error - prev_error) / dt;
//   float angular_correction = Kp * angular_error + Ki * integral + Kd * derivative;

//   // Calculate motor speeds
//   float left_speed = base_speed - angular_correction;
//   float right_speed = base_speed + angular_correction;

//   // Constrain speeds
//   left_speed = constrain(left_speed, -max_speed, max_speed);
//   right_speed = constrain(right_speed, -max_speed, max_speed);

//   setMotor(right_speed, left_speed);
//   prev_error = angular_error;
// }


void taskOdometry(void *parameter) {
  TickType_t xLastTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);  //50Hz

  for (;;) {
    if (xSemaphoreTake(odomMutex, portMAX_DELAY)) {
      update_imu();
      update_odom();
      // navigate_to_waypoint();
      xSemaphoreGive(odomMutex);
    }

    vTaskDelayUntil(&xLastTime, xFrequency);
  }
}

void taskNavigation(void *parameter) {
  TickType_t xLastTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50);  // 20Hz

  for (;;) {
    // Read odometry data
    float local_x, local_y, local_theta;
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(10))) {
      local_x = x;
      local_y = y;
      local_theta = theta;
      xSemaphoreGive(odomMutex);
    } else {
      // Couldn't get odometry, skip this cycle
      vTaskDelayUntil(&xLastTime, xFrequency);
      continue;
    }

    // Navigation logic (no mutex needed, uses local copies)
    if (xSemaphoreTake(navMutex, portMAX_DELAY)) {
      navigate_to_waypoint_threaded(local_x, local_y, local_theta);
      xSemaphoreGive(navMutex);
    }

    vTaskDelayUntil(&xLastTime, xFrequency);
  }
}
void navigate_to_waypoint_threaded(float current_x, float current_y, float current_theta) {
    if (current_waypoint >= total_waypoints) {
        setMotor(0, 0);
        return;
    }
    
    float target_x = waypoints[current_waypoint].x;
    float target_y = waypoints[current_waypoint].y;
    
    // Calculate distance and angle to target
    float error_x = target_x - current_x;
    float error_y = target_y - current_y;
    float distance = calculate_distance(current_x, current_y, target_x, target_y);
    float angle_to_target = atan2(error_y, error_x);
    float angular_error = normalize_angle(angle_to_target - current_theta);
    
    // If reached waypoint
    if (distance < distance_threshold) {
        current_waypoint++;
        integral = 0;
        prev_error = 0;
        nav_state = ROTATING;  // Reset to ROTATING for next waypoint
        setMotor(0, 0);
        return;
    }
    
    // Calculate dt for PID
    unsigned long current_time = millis();
    float dt = (current_time - last_pid_time) / 1000.0;
    last_pid_time = current_time;
    
    // State machine for navigation
    switch (nav_state) {
        case ROTATING:
            // Only rotate on first move to new waypoint
            if (abs(angular_error) > angle_threshold) {
                float rotation_correction = constrain(Kp * angular_error * 650, -rotation_speed, rotation_speed);
                setMotor(rotation_correction, -rotation_correction);
                integral = 0;
                prev_error = angular_error;
            } else {
                // Switch to MOVING after initial rotation
                nav_state = MOVING;
                integral = 0;
                prev_error = 0;
                setMotor(0, 0);
            }
            break;
        
        case MOVING:
            // Dynamic angle correction while moving (no switching back to ROTATING)
            integral += angular_error * dt;
            float derivative = (angular_error - prev_error) / dt;
            float angular_correction = Kp * angular_error + Ki * integral + Kd * derivative;
            
            // Apply correction to motor speeds
            float left_speed = base_speed - angular_correction;
            float right_speed = base_speed + angular_correction;
            
            // Constrain speeds to valid range
            left_speed = constrain(left_speed, -255, 255);
            right_speed = constrain(right_speed, -255, 255);
            
            setMotor(right_speed, left_speed);
            prev_error = angular_error;
            break;
    }
}

void taskSerialPrint(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(200);  // 5Hz

  for (;;) {
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(10))) {
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
      Serial.print(theta_imu * (180 / PI), 2);
      Serial.print(" deg | PWM_R: ");
      Serial.print(current_pwm_right);
      Serial.print(" | PWM_L: ");
      Serial.print(current_pwm_left);
      Serial.print(" | State: ");

      // Read nav state safely
      if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(5))) {
        Serial.print(nav_state == ROTATING ? "ROTATING" : "MOVING");
        Serial.print(" | WP: ");
        Serial.print(current_waypoint);
        Serial.print("/");
        Serial.println(total_waypoints);
        xSemaphoreGive(navMutex);
      } else {
        Serial.println("---");
      }

      xSemaphoreGive(odomMutex);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}