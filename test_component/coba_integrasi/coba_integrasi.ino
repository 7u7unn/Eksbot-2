// ==================== LINE SENSOR ====================
#define NUM_LINE_SENSORS 12
int line_threshold[NUM_LINE_SENSORS];
int line_values[NUM_LINE_SENSORS];
bool line_calibrated = false;

int s0 = 25, s1 = 33, s2 = 32, s3 = 26, SIG_pin = 34;

// ==================== MOTOR & ENCODER ====================
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

// ==================== LIBRARIES ====================
#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <EEPROM.h>

// ==================== WIFI & MICRO-ROS ====================
const char *ssid = "hay";
const char *password = "jujundial7";
const char *agent_ip = "10.241.196.69";  // Ganti sesuai PC Anda
const int agent_port = 8888;

// ==================== MICRO-ROS OBJECTS ====================
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
std_msgs__msg__String sub_msg;
std_msgs__msg__String pub_msg;

// ==================== ROBOT STATE ====================
enum RobotState {
  ROBOT_STOPPED,
  ROBOT_RUNNING
};
RobotState robot_state = ROBOT_STOPPED;

// ==================== MISSION STATE ====================
enum MissionType {
  MISSION_NONE,
  MISSION_KANAN,
  MISSION_KIRI
};
MissionType current_mission = MISSION_NONE;
int mission_step = 0;

// ==================== IMU & ODOMETRY ====================
MPU6050 mpu;
bool finish = false;

const int filter_window_size = 5;
const int calibration_samples = 700;
int filter_index = 0;
bool filter_full = false;
unsigned long last_time;
float theta_imu = PI / 2;
float gyroZ_calibration = 0, gyroZ_avg = 0, gyroZ_values[filter_window_size];


volatile long val_R = 0;
volatile long val_L = 0;
long val_R_prev = 0;
long val_L_prev = 0;

// Robot parameters
float ppr = 11.0;
float gearbox_R = 49.0;
float gearbox_L = 49.0;
float L = 30.0;
float diameter = 7.0;
float wheel_k = (PI * diameter);

float x = 0.0, y = 0.0, theta = PI / 2;
float ttc_R = wheel_k / (gearbox_R * ppr);
float ttc_L = wheel_k / (gearbox_L * ppr);

// ==================== RTOS & MUTEX ====================
SemaphoreHandle_t odomMutex;
SemaphoreHandle_t navMutex;
SemaphoreHandle_t stateMutex;

TaskHandle_t taskOdomHandle = NULL;
TaskHandle_t taskNavHandle = NULL;
TaskHandle_t taskMicroRosHandle = NULL;
// ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'start'"


// ==================== NAVIGATION STATE ====================
enum NavigationState {
  NAV_IDLE,
  NAV_ROTATING,
  NAV_MOVING_STRAIGHT,
  NAV_MANUVER,
  NAV_LINEFOLLOW
};

NavigationState nav_state = NAV_IDLE;

bool command_active = false;
float command_target_distance = 0;
float command_target_angle = 0;
float initial_heading = 0;
float start_x = 0;
float start_y = 0;

enum CommandType {
  CMD_NONE,
  CMD_STRAIGHT,
  CMD_ROTATE,
  CMD_MANUVER,
  CMD_IDLE,
  CMD_LINEFOL
};
CommandType current_command = CMD_NONE;

float manuver_start_angle = 0;
float manuver_target_delta = 180.0;
bool kanan_direction = false; // true = kanan, false = kiri

// ==================== PID PARAMETERS ====================
// Rotation
float Kp_rot = 2, Ki_rot = 0.0, Kd_rot = 3.5;
float integral_rot = 0.0, prev_error_rot = 0.0;

// Linear motion
float Kp_linear = 1.6, Ki_linear = 0.0, Kd_linear = 0.02;
float integral_linear = 0.0, prev_error_linear = 0.0;

// Angular correction
float Kp_angular = 4.4, Ki_angular = 0.0, Kd_angular = 1;
float integral_angular = 0.0, prev_error_angular = 0.0;

// Line follower PID
float Kp_lf = 0.005, Ki_lf = 0.0, Kd_lf = 0.0;
float error_lf = 0, lastError_lf = 0, integral_lf = 0, lastPosition_lf = 2500;

// Constraints
float distance_threshold = 1.0;
float angle_threshold = 0.5;
float rotation_speed = 150;
float max_speed = 230;
float min_speed = 125;
float integral_max = 2000.0;
float integral_min = -2000.0;

volatile int current_pwm_right = 0;
volatile int current_pwm_left = 0;

// ==================== FUNCTION DECLARATIONS ====================
void IRAM_ATTR Read_R();
void IRAM_ATTR Read_L();
void update_odom();
void update_imu();
void calib_imu();
void taskOdometry(void *parameter);
void taskNavigation(void *parameter);
void taskMicroRos(void *parameter);
void init_motor();
void setMotor(int spdKanan, int spdKiri);

// Navigation commands (non-blocking: only set targets)
void start_straight(float distance, float heading);
void start_rotate(float degrees);
void start_manuver(bool ccw);
void stop_motors();

// Execution functions (called from taskNavigation)
void execute_straight(float x, float y, float theta_imu);
void execute_rotate(float theta_imu);
void execute_manuver(float theta_imu);
void execute_linefol();

// Line sensor
void init_mux();
int readMux(int ch);
void read_all_line_sensors();
void calibrate_line_sensors(int samples = 2000);
void load_line_thresholds();
void save_line_thresholds();
bool is_on_line(int i);
float calculatePosition(bool depan);
void auto_calib(int step);

// MicroROS
void subscription_callback(const void *msgin);
bool setup_microros();

// Helpers
float normalize_angle(float angle);
float calculate_distance(float x1, float y1, float x2, float y2);
float rad2deg(float i);

// ==================== SETUP ====================
void setup() {
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  attachInterrupt(encA1, Read_R, RISING);

  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(encA2, Read_L, RISING);

  Serial.begin(115200);
  init_motor();
  EEPROM.begin(512);
  init_mux();

  load_line_thresholds();
  Serial.println("Line thresholds loaded.");

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  delay(3000); // Stabilisasi sebelum kalibrasi IMU
  calib_imu();

  odomMutex = xSemaphoreCreateMutex();
  navMutex = xSemaphoreCreateMutex();
  stateMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskOdometry, "OdometryTask", 4096, NULL, 2, &taskOdomHandle, 1);
  xTaskCreatePinnedToCore(taskNavigation, "NavigationTask", 8192, NULL, 2, &taskNavHandle, 1);
  xTaskCreatePinnedToCore(taskMicroRos, "MicroRosTask", 8192, NULL, 1, &taskMicroRosHandle, 0);

  Serial.println("Robot ready.");
}

void loop() {
  // Tidak ada logika di sini — semua di taskNavigation
  delay(1);
}

// ==================== HELPER FUNCTIONS ====================
float rad2deg(float i) { return (180.0 / PI) * i; }
float deg2rad(float i) { return (PI / 180.0) * i; }

void IRAM_ATTR Read_R() {
  if (digitalRead(encB1) == LOW) val_R++; else val_R--;
}
void IRAM_ATTR Read_L() {
  if (digitalRead(encB2) == LOW) val_L++; else val_L--;
}

void calib_imu() {
  Wire.begin(21, 22, 100000);
  mpu.initialize();
  while (!mpu.testConnection()) delay(500);

  Serial.println("Calibrating IMU...");
  for (int i = 0; i < calibration_samples; i++) {
    gyroZ_calibration += mpu.getRotationZ();
    delay(3);
  }
  gyroZ_calibration /= calibration_samples;

  for (int i = 0; i < filter_window_size; i++) gyroZ_values[i] = 0;
  last_time = millis();
  Serial.println("IMU calibrated.");
}

void update_imu() {
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  last_time = now;

  int16_t gyroZ_raw = mpu.getRotationZ();
  float gyroZ = (gyroZ_raw - gyroZ_calibration) / 131.0;
  gyroZ_values[filter_index] = gyroZ;
  filter_index = (filter_index + 1) % filter_window_size;
  if (filter_index == 0) filter_full = true;

  int count = filter_full ? filter_window_size : filter_index;
  gyroZ_avg = 0;
  for (int i = 0; i < count; i++) gyroZ_avg += gyroZ_values[i];
  gyroZ_avg /= count;

  theta_imu -= deg2rad(gyroZ_avg) * dt;
  if (theta_imu >= 2 * PI) theta_imu -= 2 * PI;
  if (theta_imu < 0) theta_imu += 2 * PI;
}

void update_odom() {
  long dR = val_R - val_R_prev;
  long dL = val_L - val_L_prev;
  val_R_prev = val_R;
  val_L_prev = val_L;

  float dLeft = dL * ttc_L;
  float dRight = dR * ttc_R;
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
  current_pwm_right = spdKanan;
  current_pwm_left = spdKiri;

  digitalWrite(IN1, spdKanan > 0 ? HIGH : (spdKanan < 0 ? LOW : LOW));
  digitalWrite(IN2, spdKanan < 0 ? HIGH : LOW);
  analogWrite(ENA, constrain(abs(spdKanan), 0, 255));

  digitalWrite(IN3, spdKiri > 0 ? HIGH : (spdKiri < 0 ? LOW : LOW));
  digitalWrite(IN4, spdKiri < 0 ? HIGH : LOW);
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

// ==================== LINE SENSOR ====================
void init_mux() {
  pinMode(s0, OUTPUT); pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT); pinMode(s3, OUTPUT);
}

int readMux(int channel) {
  int sel[12][4] = {
    {0,0,0,0},{1,0,0,0},{0,1,0,0},{1,1,0,0},
    {0,0,1,0},{1,0,1,0},{0,1,1,0},{1,1,1,0},
    {0,0,0,1},{1,0,0,1},{0,1,0,1},{1,1,0,1}
  };
  digitalWrite(s0, sel[channel][0]);
  digitalWrite(s1, sel[channel][1]);
  digitalWrite(s2, sel[channel][2]);
  digitalWrite(s3, sel[channel][3]);
  return analogRead(SIG_pin);
}

void read_all_line_sensors() {
  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    line_values[i] = readMux(i);
    delay(2);
  }
}

bool is_on_line(int i) {
  return line_values[i] < line_threshold[i];
}

float calculatePosition(bool depan) {
  float wSum = 0; int sum = 0;
  int start = depan ? 6 : 0;
  for (int i = 0; i < 6; i++) {
    if (is_on_line(start + i)) {
      wSum += i * 1000;
      sum++;
    }
  }
  return (sum == 0) ? 2500 : wSum / sum;
}

void calibrate_line_sensors(int samples) {
  int minv[12], maxv[12];
  for (int i = 0; i < 12; i++) { minv[i] = 4095; maxv[i] = 0; }
  for (int s = 0; s < samples; s++) {
    read_all_line_sensors();
    for (int i = 0; i < 12; i++) {
      if (line_values[i] < minv[i]) minv[i] = line_values[i];
      if (line_values[i] > maxv[i]) maxv[i] = line_values[i];
    }
    delay(10);
  }
  for (int i = 0; i < 12; i++) {
    line_threshold[i] = (minv[i] + maxv[i]) / 2.5;
  }
  save_line_thresholds();
  line_calibrated = true;
}

void load_line_thresholds() {
  for (int i = 0; i < 12; i++) {
    EEPROM.get(i * sizeof(int), line_threshold[i]);
  }
}

void save_line_thresholds() {
  for (int i = 0; i < 12; i++) {
    EEPROM.put(i * sizeof(int), line_threshold[i]);
  }
  EEPROM.commit();
}

void auto_calib(int step) {
  for (int i = 0; i < step; i++) {
    unsigned long start = millis();
    while (millis() - start <= 1500) {
      if (millis() - start <= 700) setMotor(100, 100);
      else if (millis() - start <= 750) setMotor(0, 0);
      else if (millis() - start <= 1500) setMotor(-100, -100);
      else setMotor(0, 0);
      calibrate_line_sensors(1);
    }
  }
}

// ==================== NON-BLOCKING COMMANDS ====================
void start_straight(float distance, float heading) {
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
      start_x = x; start_y = y;
      xSemaphoreGive(odomMutex);
    }
    command_target_distance = distance;
    initial_heading = heading;
    current_command = CMD_STRAIGHT;
    command_active = true;
    nav_state = NAV_MOVING_STRAIGHT;
    integral_linear = integral_angular = 0;
    prev_error_linear = prev_error_angular = 0;
    xSemaphoreGive(navMutex);
  }
}

void start_rotate(float degrees) {
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    command_target_angle = normalize_angle(degrees);
    current_command = CMD_ROTATE;
    command_active = true;
    nav_state = NAV_ROTATING;
    integral_rot = 0;
    prev_error_rot = 0;
    xSemaphoreGive(navMutex);
  }
}

void start_manuver(bool ccw) {
  kanan_direction = ccw;
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
      manuver_start_angle = rad2deg(theta_imu);
      xSemaphoreGive(odomMutex);
    }
    current_command = CMD_MANUVER;
    command_active = true;
    nav_state = NAV_MANUVER;
    integral_rot = 0;
    prev_error_rot = 0;
    xSemaphoreGive(navMutex);
  }
}

void stop_motors() {
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    current_command = CMD_IDLE;
    command_active = false;
    nav_state = NAV_IDLE;
    setMotor(0, 0);
    xSemaphoreGive(navMutex);
  }
}

// ==================== EXECUTION FUNCTIONS ====================
void execute_straight(float current_x, float current_y, float current_theta_imu) {
  float dist_traveled = calculate_distance(start_x, start_y, current_x, current_y);
  float remaining = abs(command_target_distance) - dist_traveled;
  int dir = (command_target_distance > 0) ? 1 : -1;

  if (remaining < distance_threshold) {
    setMotor(0, 0);
    command_active = false;
    nav_state = NAV_IDLE;
    return;
  }

  integral_linear = constrain(integral_linear + remaining, integral_min, integral_max);
  float deriv_lin = remaining - prev_error_linear;
  prev_error_linear = remaining;
  float lin_speed = (Kp_linear * remaining) + (Ki_linear * integral_linear) + (Kd_linear * deriv_lin);
  lin_speed = constrain(lin_speed, min_speed + 5, max_speed + 5) * dir;

  float head_err = normalize_angle(initial_heading - current_theta_imu);
  integral_angular = constrain(integral_angular + head_err, integral_min, integral_max);
  float deriv_ang = head_err - prev_error_angular;
  prev_error_angular = head_err;
  float ang_corr = (Kp_angular * head_err) + (Ki_angular * integral_angular) + (Kd_angular * deriv_ang);

  float ls = lin_speed - ang_corr;
  float rs = lin_speed + ang_corr;
  ls = (ls > 0) ? constrain(ls, min_speed, max_speed) : constrain(ls, -max_speed, -min_speed);
  rs = (rs > 0) ? constrain(rs, min_speed, max_speed) : constrain(rs, -max_speed, -min_speed);
  setMotor(rs, ls);
}

void execute_rotate(float current_theta_imu) {
  float err = normalize_angle(command_target_angle - current_theta_imu);
  if (abs(err) > angle_threshold) {
    integral_rot = constrain(integral_rot + err, integral_min, integral_max);
    float deriv = err - prev_error_rot;
    prev_error_rot = err;
    float out = (Kp_rot * err) + (Ki_rot * integral_rot) + (Kd_rot * deriv);
    int corr = constrain((int)out, -rotation_speed, rotation_speed);
    if (corr > 0 && corr < 96) corr = 96;
    else if (corr < 0 && corr > -96) corr = -96;
    setMotor(corr, -corr);
  } else {
    setMotor(0, 0);
    command_active = false;
    nav_state = NAV_IDLE;
  }
}

void execute_manuver(float current_theta_imu) {
  float delta = current_theta_imu - manuver_start_angle;
  if (delta < -180) delta += 360;
  if (delta > 180) delta -= 360;
  float err = manuver_target_delta - abs(delta);
  if (abs(err) > angle_threshold) {
    integral_rot = constrain(integral_rot + err, integral_min, integral_max);
    float deriv = err - prev_error_rot;
    prev_error_rot = err;
    float out = (Kp_rot * err) + (Ki_rot * integral_rot) + (Kd_rot * deriv);
    int ls = constrain((int)out, min_speed + 5, max_speed + 5);
    if (kanan_direction) {
      setMotor(-ls, 0); // Rotate around right wheel
    } else {
      setMotor(0, -ls); // Rotate around left wheel
    }
  } else {
    setMotor(0, 0);
    command_active = false;
    nav_state = NAV_IDLE;
  }
}

void execute_linefol() {
  read_all_line_sensors();
  float pos = calculatePosition(true);
  error_lf = 2500 - pos;
  lastPosition_lf = pos;
  integral_lf = constrain(integral_lf + error_lf, -10000, 10000);
  float deriv = error_lf - lastError_lf;
  lastError_lf = error_lf;
  float out = (Kp_lf * error_lf) + (Ki_lf * integral_lf) + (Kd_lf * deriv);

  int minSpd = 92;
  int ls, rs;
  if (out < 0) {
    rs = minSpd + abs(out);
    ls = minSpd;
  } else {
    rs = minSpd;
    ls = minSpd + abs(out);
  }
  ls = constrain(ls, minSpd, 180);
  rs = constrain(rs, minSpd, 180);
  setMotor(rs, ls);
}

// ==================== TASKS ====================
void taskOdometry(void *parameter) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t freq = pdMS_TO_TICKS(10);
  for (;;) {
    if (xSemaphoreTake(odomMutex, portMAX_DELAY) == pdTRUE) {
      update_imu();
      update_odom();
      xSemaphoreGive(odomMutex);
    }
    vTaskDelayUntil(&last, freq);
  }
}

void taskNavigation(void *parameter) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t freq = pdMS_TO_TICKS(15);

  for (;;) {
    float lx, ly, ltheta_imu;
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      lx = x; ly = y; ltheta_imu = rad2deg(theta_imu);
      xSemaphoreGive(odomMutex);
    } else {
      vTaskDelayUntil(&last, freq);
      continue;
    }

    // Eksekusi perintah aktif
    if (xSemaphoreTake(navMutex, portMAX_DELAY) == pdTRUE) {
      if (command_active) {
        switch (current_command) {
          case CMD_STRAIGHT: execute_straight(lx, ly, ltheta_imu); break;
          case CMD_ROTATE:   execute_rotate(ltheta_imu); break;
          case CMD_MANUVER:  execute_manuver(ltheta_imu); break;
          case CMD_LINEFOL:  execute_linefol(); break;
          case CMD_IDLE:     setMotor(0, 0); break;
          default: break;
        }
      }
      xSemaphoreGive(navMutex);
    }

    // Eksekusi misi otomatis (jika tidak ada perintah aktif)
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      bool running = (robot_state == ROBOT_RUNNING);
      xSemaphoreGive(stateMutex);

      if (running && current_mission != MISSION_NONE && !command_active) {
        if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          bool mission_done = false;
          switch (current_mission) {
            case MISSION_KANAN:
              switch (mission_step) {
                case 0: start_straight(15, 90); break;
                case 1: start_rotate(180); break;
                case 2: start_straight(44, 180); break;
                case 3: start_rotate(90); break;
                case 4: start_straight(160, 90); break;
                case 5: start_manuver(true); break;
                case 6: start_straight(160, -90); break;
                case 7: start_manuver(true); break;
                case 8: mission_done = true; break;
              }
              break;
            case MISSION_KIRI:
              switch (mission_step) {
                case 0: start_straight(15, 90); break;
                case 1: start_rotate(0); break;
                case 2: start_straight(44, 0); break;
                case 3: start_rotate(90); break;
                case 4: start_straight(160, 90); break;
                case 5: start_manuver(false); break;
                case 6: start_straight(160, -90); break;
                case 7: start_manuver(false); break;
                case 8: mission_done = true; break;
              }
              break;
          }

          if (mission_done) {
            current_mission = MISSION_NONE;
            mission_step = 0;
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
              robot_state = ROBOT_STOPPED;
              xSemaphoreGive(stateMutex);
            }
            Serial.println("Mission completed!");
          } else if (!command_active) {
            mission_step++;
          }
          xSemaphoreGive(navMutex);
        }
      }
    }

    vTaskDelayUntil(&last, freq);
  }
}

void taskMicroRos(void *parameter) {
  if (!setup_microros()) {
    Serial.println("MicroROS setup failed!");
    vTaskDelete(NULL);
    return;
  }
  Serial.println("MicroROS ready.");

  for (;;) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    delay(10);

    
  }
}

// ==================== MICRO-ROS CALLBACK ====================
void subscription_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  String cmd = String(msg->data.data);
  cmd.toUpperCase(); cmd.trim();
  Serial.println("Command: " + cmd);

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (cmd == "START-KANAN") {
      current_mission = MISSION_KANAN;
      mission_step = 0;
      robot_state = ROBOT_RUNNING;
      finish = false;
      pub_msg.data.data = (char *)"Started right mission";
    } else if (cmd == "START-KIRI") {
      current_mission = MISSION_KIRI;
      mission_step = 0;
      robot_state = ROBOT_RUNNING;
      finish = false;
      pub_msg.data.data = (char *)"Started left mission";
    } else if (cmd == "STOP") {
      robot_state = ROBOT_STOPPED;
      current_mission = MISSION_NONE;
      mission_step = 0;
      stop_motors();
      pub_msg.data.data = (char *)"Robot stopped";
    } else if (cmd == "CALIB") {
      calib_imu();
      stop_motors();
      pub_msg.data.data = (char *)"IMU calibrated";
    } else if (cmd == "CALIB_LINE") {
      auto_calib(4);
      line_calibrated = true;
      pub_msg.data.data = (char *)"Line sensors calibrated";
    } else if (cmd == "LINEFOL") {
      if (!line_calibrated) {
        pub_msg.data.data = (char *)"Line sensors not calibrated";
      } else {
        if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          current_command = CMD_LINEFOL;
          command_active = true;
          nav_state = NAV_LINEFOLLOW;
          integral_lf = lastError_lf = 0;
          lastPosition_lf = 2500;
          xSemaphoreGive(navMutex);
        }
        pub_msg.data.data = (char *)"Line follower started";
      }
    } else {
      pub_msg.data.data = (char *)"Unknown command";
    }
    pub_msg.data.size = strlen(pub_msg.data.data);
    rcl_publish(&publisher, &pub_msg, NULL);
    xSemaphoreGive(stateMutex);
  }
}

bool setup_microros() {
  set_microros_wifi_transports((char *)ssid, (char *)password, (char *)agent_ip, agent_port);
  delay(2000);

  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "esp32_robot_node", "", &support) != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "robot_command") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "robot_status") != RCL_RET_OK) return false;
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  sub_msg.data.data = (char *)malloc(100);
  sub_msg.data.capacity = 100;
  pub_msg.data.data = (char *)malloc(100);
  pub_msg.data.capacity = 100;

  return true;
}