#include <Wire.h>
#include <MPU6050.h>
#include <EEPROM.h>

// === PIN CONFIGURATIONS ===
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

int bias = 6;

int s0 = 25;
int s1 = 33;
int s2 = 32;
int s3 = 26;
int SIG_pin = 34;

// === ROBOT PHYSICAL PARAMETERS ===
float ppr = 11.0;
float gearbox_R = 50.0;
float gearbox_L = 50.0;
float L = 28.5;
float diameter = 6.9;
float wheel_k = (PI * diameter);

float ttc_R = wheel_k / (gearbox_R * ppr);
float ttc_L = wheel_k / (gearbox_L * ppr);

// === MOTION CONTROL PARAMETERS ===
float distance_threshold = 0.5;  // cm
float angle_threshold = 0.5;     // degrees
float rotation_speed = 160;
float max_speed = 220;
float min_speed = 115;
float rot_min_speed = 98;

// PID for rotation
float Kp_rot = 2;
float Ki_rot = 0.0;
float Kd_rot = 3.5;
float integral_rot = 0.0;
float prev_error_rot = 0.0;

// PID for linear motion
float Kp_linear = 2.0;
float Ki_linear = 0.000;
float Kd_linear = 0.02;
float integral_linear = 0.0;
float prev_error_linear = 0.0;

// PID for heading correction
float Kp_angular = 4.6;
float Ki_angular = 0.00;
float Kd_angular = 0.8;
float integral_angular = 0.0;
float prev_error_angular = 0.0;

// PID for line following (used in approach)
float Kp_linefol = 0.042;
float Ki_linefol = 0.0;
float Kd_linefol = 0.004;

float integral_max = 2000.0;
float integral_min = -2000.0;

// === IMU PARAMETERS ===
MPU6050 mpu;

const int filter_window_size = 3;
const int calibration_samples = 700;
int filter_index = 0;
bool filter_full = false;
unsigned long last_time;
float gyroZ_calibration = 0;
float gyroZ_avg = 0;
float gyroZ_values[filter_window_size];

// === STATE VARIABLES ===
bool calib_imu_requested = false;
bool calib_line_requested = false;

enum RobotState {
  ROBOT_STOPPED,
  ROBOT_RUNNING
};
RobotState robot_state = ROBOT_STOPPED;

enum NavigationState {
  ROTATING,
  MOVING_STRAIGHT,
  IDLE,
  MANUVER,
};
NavigationState nav_state = IDLE;

enum CommandType {
  CMD_NONE,
  CMD_STRAIGHT,
  CMD_ROTATE,
  CMD_MANUVER,
  CMD_IDLE,
  CMD_APPROACH
};
CommandType current_command = CMD_NONE;

// === POSITION TRACKING ===
volatile long val_R = 0;
volatile long val_L = 0;
long val_R_prev = 0;
long val_L_prev = 0;
float x = 0.0;


float y = 0.0;
float theta = PI / 2;
float theta_imu = PI / 2;
float theta_fuse = PI / 2;

bool command_active = false;
bool finish = false;
float command_target_distance = 0;
float command_target_angle = 0;
float initial_heading = 0;
float start_x = 0;
float start_y = 0;
String command = "";

// === LINE FOLLOWER ===
#define NUM_LINE_SENSORS 12
int line_threshold[NUM_LINE_SENSORS];
int line_values[NUM_LINE_SENSORS];
bool line_calibrated = false;

// === TASK MANAGEMENT ===
SemaphoreHandle_t odomMutex;
SemaphoreHandle_t navMutex;
SemaphoreHandle_t stateMutex;

TaskHandle_t taskOdomHandle = NULL;
TaskHandle_t taskSerialHandle = NULL;
TaskHandle_t taskNavHandle = NULL;

float manuver_start_angle = 0;
float manuver_target_delta = 180.0;
bool kanan = false;
bool linefol = false;

volatile int current_pwm_right = 0;
volatile int current_pwm_left = 0;

// === APPROACH STATE (replaced time-based with distance-based) ===
float approach_target_distance = 0.0;
bool approach_use_front_sensor = true;
float approach_start_x = 0.0;
float approach_start_y = 0.0;
float approach_error = 0;
float approach_last_error = 0;
float approach_integral = 0;

// === FUNCTION DECLARATIONS ===
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
void manuver(bool ccw_dir);
void straight(float distance, float heading);
void rotate(float degrees);
void idle();
bool is_command_done();
void execute_straight(float current_x, float current_y, float current_theta_enc);
void execute_rotate(float current_theta_imu);
void exec_manuver(float current_theta_imu, bool ccw);
float normalize_angle(float angle);
float calculate_distance(float x1, float y1, float x2, float y2);
float rad2deg(float i);
void pick_depan();
void pick_blkg();
void put_depan();
void put_blkg();
void approach(float distance_cm, bool use_front_sensor);  // now takes distance in cm
void execute_approach();
void decide_mission();

// === MISC ===
bool command_in_progress = false;  // for serial task
bool mission_kanan = false;

void setup() {
  init_mux();
  decide_mission();

  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  attachInterrupt(encA1, Read_R, RISING);

  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(encA2, Read_L, RISING);

  EEPROM.begin(512);
  Serial.begin(115200);
  init_motor();
  load_line_thresholds();


  delay(4000);
  calib_imu();

  odomMutex = xSemaphoreCreateMutex();
  navMutex = xSemaphoreCreateMutex();
  stateMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskOdometry, "OdometryTask", 4096, NULL, 2, &taskOdomHandle, 1);
  xTaskCreatePinnedToCore(taskNavigation, "NavigationTask", 4096, NULL, 2, &taskNavHandle, 0);

  vTaskDelay(pdMS_TO_TICKS(1000));
}

void loop() {
  static bool misi_dimulai = false;
  if (calib_imu_requested) {
    calib_imu_requested = false;
    idle();
    vTaskSuspend(taskOdomHandle);
    theta = PI / 2;
    theta_imu = PI / 2;
    calib_imu();
    vTaskResume(taskOdomHandle);
  }

  if (calib_line_requested) {
    calib_line_requested = false;
    idle();
    vTaskSuspend(taskOdomHandle);
    auto_calib(500);
    val_R = 0;
    val_L = 0;
    val_R_prev = 0;
    val_L_prev = 0;
    x = 0.0;
    y = 0.0;
    vTaskResume(taskOdomHandle);
    misi_dimulai = true;
  }

  if (!misi_dimulai && mission_kanan == true) {
    misi_dimulai = true;


    // straight(155, 90);
    // rotate(180);
    // straight(37, 180);
    // rotate(90);
    move_to(-37, 155, 90);

    approach(30, true);
    pick_depan();
    approach(30, false);
    rotate(0);
    straight(-27, 0);
    rotate(270);
    approach(30, false);  // mundur 30 cm
    pick_blkg();
    rotate(360 - bias);
    straight(25, 360 - bias);
    rotate(270 - bias);
    approach(28, true);  // mundur 30 cm

    bias++;
    straight(120, 272 - bias);
    approach(30, true);
    vTaskDelay(pdMS_TO_TICKS(700));
    put_depan();

    approach(30, false);  // mundur 30 cm
    rotate(360 - bias);
    straight(-39, 360 - bias);
    rotate(90 - bias);
    approach(30, false);  // mundur 25 cm
    vTaskDelay(pdMS_TO_TICKS(1000));
    put_blkg();
    // put_blkg();
    approach(30, true);
    bias += 5;

    //mulih
    straight(155, 90 - bias);
    // approach(35, true);
    rotate(360 - bias);
    straight(86, 360 - bias);
    rotate(270);
  } else if (!misi_dimulai && mission_kanan == false) {
    misi_dimulai = true;

    // straight(155, 90);
    // rotate(0);
    // straight(37, 0);
    // rotate(90);
    move_to(42, 155, 88);


    approach(30, true);
    pick_depan();
    approach(30, false);
    rotate(180);
    straight(-27, 180);
    rotate(270);
    approach(30, false);  // mundur 30 cm
    pick_blkg();
    rotate(180 - bias);
    straight(25, 180 - bias);
    rotate(270 - bias);
    approach(28, true);  // mundur 30 cm

    bias++;
    straight(120, 270 - bias);
    approach(30, true);
    vTaskDelay(pdMS_TO_TICKS(700));
    put_depan();

    approach(30, false);  // mundur 30 cm
    rotate(180 - bias);
    straight(-41, 180 - bias);
    rotate(90 - bias);
    approach(30, false);  // mundur 25 cm
    vTaskDelay(pdMS_TO_TICKS(1000));
    put_blkg();
    // put_blkg();
    approach(30, true);
    bias += 5;

    //mulih
    straight(155, 90 - bias);
    // approach(35, true);
    rotate(180 - bias);
    straight(86, 180 - bias);
    rotate(270 - bias);
  }

  delay(100);
}

// === HELPER FUNCTIONS ===

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
  Wire.begin(21, 22, 100000);
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  while (!mpu.testConnection()) {
    delay(500);
  }

  for (int i = 0; i < calibration_samples; i++) {
    gyroZ_calibration += mpu.getRotationZ();
    delay(3);
  }
  gyroZ_calibration /= calibration_samples;

  for (int i = 0; i < filter_window_size; i++) {
    gyroZ_values[i] = 0;
  }

  last_time = millis();
  theta_imu = PI / 2;
  theta = PI / 2;
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
  current_pwm_right = spdKanan;
  current_pwm_left = spdKiri;

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

// === LINE FOLLOWER ===

void init_mux() {
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
}

int readMux(int channel) {
  int sel[14][4] = {
    { 0, 0, 0, 0 }, { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 1, 1, 0, 0 }, { 0, 0, 1, 0 }, { 1, 0, 1, 0 }, { 0, 1, 1, 0 }, { 1, 1, 1, 0 }, { 0, 0, 0, 1 }, { 1, 0, 0, 1 }, { 0, 1, 0, 1 }, { 1, 1, 0, 1 }, { 0, 0, 1, 1 }, { 1, 0, 1, 1 }
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
    delayMicroseconds(100);
  }
}

bool is_on_line(int i) {
  return line_values[i] < line_threshold[i];
}

float calculatePosition(bool depan) {
  float weightedSum = 0;
  int sum = 0;
  int start = depan ? 0 : 6;  // 0-5 = depan, 6-11 = belakang
  for (int i = 0; i < 6; i++) {
    if (is_on_line(start + i)) {
      weightedSum += i * 1000;
      sum++;
    }
  }
  return (sum == 0) ? 2500 : weightedSum / sum;
}

void calibrate_line_sensors(int samples) {
  int minv[12], maxv[12];
  for (int i = 0; i < 12; i++) {
    minv[i] = 4095;
    maxv[i] = 0;
  }
  for (int s = 0; s < samples; s++) {
    read_all_line_sensors();
    for (int i = 0; i < 12; i++) {
      if (line_values[i] < minv[i]) minv[i] = line_values[i];
      if (line_values[i] > maxv[i]) maxv[i] = line_values[i];
    }
    delay(10);
  }
  for (int i = 0; i < 12; i++) {
    line_threshold[i] = (minv[i] + maxv[i]) / 2;
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
  line_calibrated = false;
  setMotor(110, -110);
  while (!line_calibrated) {
    calibrate_line_sensors(step);
    line_calibrated = true;
  }
  setMotor(0, 0);


}

// === TASKS ===

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
            exec_manuver(local_theta_imu, kanan);
            break;
          case CMD_APPROACH:
            execute_approach();
            break;
          default: break;
        }
      }
      xSemaphoreGive(navMutex);
    }

    vTaskDelayUntil(&xLastTime, xFrequency);
  }
}

void send(String command) {
  // vTaskDelay(pdMS_TO_TICKS(500));
  // while(Serial.available()>0){
  //   Serial.read();
  // }
  Serial.println(command);
  Serial.flush();
  vTaskDelay(100 / portTICK_PERIOD_MS);

  while (Serial.available() > 0) {
    Serial.read();
  }
}

void taskSerialPrint(void *parameter) {
  for (;;) {
    send(command);
    vTaskSuspend(taskSerialHandle);
  }
}

// === COMMAND FUNCTIONS ===

void straight(float distance, float heading) {
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
      start_x = x;
      start_y = y;
      initial_heading = heading;
      xSemaphoreGive(odomMutex);
    }
    command_target_distance = distance;
    current_command = CMD_STRAIGHT;
    command_active = true;
    nav_state = MOVING_STRAIGHT;
    integral_linear = integral_angular = 0;
    prev_error_linear = prev_error_angular = 0;
    xSemaphoreGive(navMutex);
  }

  while (!is_command_done()) {
  }
  vTaskDelay(500 / portTICK_PERIOD_MS);
}

void rotate(float degrees) {
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    command_target_angle = degrees;
    while (command_target_angle > 180) command_target_angle -= 360;
    while (command_target_angle < -180) command_target_angle += 360;
    current_command = CMD_ROTATE;
    command_active = true;
    nav_state = ROTATING;
    integral_rot = 0;
    prev_error_rot = 0;
    xSemaphoreGive(navMutex);
  }

  while (!is_command_done()) { /* wait */
  }
  vTaskDelay(500 / portTICK_PERIOD_MS);
}

void manuver(bool ccw_dir) {
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
      manuver_start_angle = rad2deg(theta_imu);
      xSemaphoreGive(odomMutex);
    }
    current_command = CMD_MANUVER;
    kanan = ccw_dir;
    command_active = true;
    nav_state = MANUVER;
    integral_rot = 0;
    prev_error_rot = 0;
    xSemaphoreGive(navMutex);
  }

  while (!is_command_done()) {
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
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

// === EXECUTION FUNCTIONS ===

void execute_straight(float current_x, float current_y, float current_theta_enc) {
  float distance_traveled = calculate_distance(start_x, start_y, current_x, current_y);
  float remaining_distance = abs(command_target_distance) - distance_traveled;
  int direction = (command_target_distance > 0) ? 1 : -1;

  if (remaining_distance < distance_threshold) {
    setMotor(0, 0);
    command_active = false;
    nav_state = IDLE;
    integral_linear = integral_angular = 0;
    prev_error_linear = prev_error_angular = 0;
    return;
  }

  integral_linear += remaining_distance;
  integral_linear = constrain(integral_linear, integral_min, integral_max);
  float derivative_linear = (remaining_distance - prev_error_linear);
  prev_error_linear = remaining_distance;
  float linear_speed = (Kp_linear * remaining_distance) + (Ki_linear * integral_linear) + (Kd_linear * derivative_linear);
  linear_speed = constrain(linear_speed, min_speed, max_speed) * direction;

  float heading_error = normalize_angle(initial_heading - current_theta_enc);
  integral_angular += heading_error;
  integral_angular = constrain(integral_angular, integral_min, integral_max);
  float derivative_angular = (heading_error - prev_error_angular);
  prev_error_angular = heading_error;
  float angular_correction = (Kp_angular * heading_error) + (Ki_angular * integral_angular) + (Kd_angular * derivative_angular);

  float left_speed = linear_speed - angular_correction;
  float right_speed = linear_speed + angular_correction;
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
    if (rotation_correction > 0 && rotation_correction < rot_min_speed) rotation_correction = rot_min_speed;
    else if (rotation_correction < 0 && rotation_correction > -rot_min_speed) rotation_correction = -rot_min_speed;
    setMotor(rotation_correction, -rotation_correction);
  } else {
    setMotor(0, 0);
    command_active = false;
    nav_state = IDLE;
    integral_rot = 0;
    prev_error_rot = 0;
  }
}

void exec_manuver(float current_theta_imu, bool ccw) {
  float delta = current_theta_imu - manuver_start_angle;
  if (delta < -180) delta += 360;
  if (delta > 180) delta -= 360;
  float angle_error = manuver_target_delta - abs(delta);
  if (abs(angle_error) > angle_threshold) {
    integral_rot += angle_error;
    integral_rot = constrain(integral_rot, integral_min, integral_max);
    float derivative_rot = (angle_error - prev_error_rot);
    prev_error_rot = angle_error;
    float pid_output = (Kp_rot * angle_error) + (Ki_rot * integral_rot) + (Kd_rot * derivative_rot);
    int left_speed = constrain((int)pid_output, min_speed, max_speed);
    if (ccw) {
      setMotor(-left_speed, 0);
    } else {
      setMotor(0, -left_speed);
    }
  } else {
    setMotor(0, 0);
    command_active = false;
    nav_state = IDLE;
    integral_rot = 0;
    prev_error_rot = 0;
  }
}


void put_blkg() {
  Serial.println("2");  //PUTB
  Serial.flush();
  vTaskDelay(500 / portTICK_PERIOD_MS);  // Wait for completion
}

void put_depan() {
  Serial.println("1");  //PUTF
  Serial.flush();
  vTaskDelay(500 / portTICK_PERIOD_MS);  // Wait for completion
}

void pick_depan() {
  Serial.println("3");  //PICKF
  Serial.flush();
  vTaskDelay(500 / portTICK_PERIOD_MS);  // Wait for completion
}

void pick_blkg() {
  Serial.println("4");  //PICKB
  Serial.flush();
  vTaskDelay(500 / portTICK_PERIOD_MS);  // Wait for completion
}


void approach(float distance_cm, bool use_front_sensor) {
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
      approach_start_x = x;
      approach_start_y = y;
      xSemaphoreGive(odomMutex);
    }
    approach_target_distance = abs(distance_cm);
    approach_use_front_sensor = use_front_sensor;
    current_command = CMD_APPROACH;
    command_active = true;
    nav_state = MOVING_STRAIGHT;

    approach_error = 0;
    approach_last_error = 0;
    approach_integral = 0;

    xSemaphoreGive(navMutex);
  }

  while (!is_command_done()) {
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  vTaskDelay(500 / portTICK_PERIOD_MS);
}

void execute_approach() {
  float local_x, local_y;
  if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    local_x = x;
    local_y = y;
    xSemaphoreGive(odomMutex);
  } else {
    return;
  }

  float traveled = calculate_distance(approach_start_x, approach_start_y, local_x, local_y);
  float remaining = approach_target_distance - traveled;

  if (remaining <= distance_threshold) {
    setMotor(0, 0);
    command_active = false;
    nav_state = IDLE;
    return;
  }

  read_all_line_sensors();
  float position = calculatePosition(approach_use_front_sensor);
  if (approach_use_front_sensor) {
    approach_error = 2500 - position;
  } else {
    approach_error = position - 2500;
  }

  approach_integral += approach_error;
  approach_integral = constrain(approach_integral, -10000, 10000);
  float derivative = approach_error - approach_last_error;
  approach_last_error = approach_error;

  float output = (Kp_linefol * approach_error) + (Ki_linefol * approach_integral) + (Kd_linefol * derivative);

  int minSpeed = 109;
  int maxSpeed = 150;
  int leftSpeed, rightSpeed;

  

  if (output > 0) {
    rightSpeed = minSpeed + abs(output);
    leftSpeed = minSpeed;
  } else if (output < 0) {
    rightSpeed = minSpeed;
    leftSpeed = minSpeed + abs(output);
  } else {
    leftSpeed = rightSpeed = minSpeed;
  }


  int direction = approach_use_front_sensor ? 1 : -1;

  // if(direction == 1){
  //   leftSpeed-=10;
  //   rightSpeed-=10;
  // }

  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);
  setMotor(rightSpeed * direction, leftSpeed * direction);
}


void decide_mission() {


  read_all_line_sensors();
  float sum = 0;
  for (int i = 0; i < 12; i++) {
    sum += line_values[i];
  }
  sum = sum / 12;

  if (sum >= 2020 && sum<3000) mission_kanan = false;
  else if (sum < 2000) mission_kanan = true;
  // else if(sum>=3500) calib_line_requested = true;
}

void move_to(float target_x, float target_y, float target_theta) {
  float current_x, current_y, current_theta_enc;

  if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
    current_x = x;
    current_y = y;
    current_theta_enc = rad2deg(theta_imu);  // atau theta_imu
    xSemaphoreGive(odomMutex);
  } else {
    // Jika gagal ambil mutex
    return;
  }

  if (abs(target_x - current_x) < 0.1 && abs(target_y - current_y) < 0.1) {
    rotate(target_theta);
    return;
  }

  float dx = target_x - current_x;
  float dy = target_y - current_y;
  float target_angle_rad = atan2(dy, dx);
  float target_angle_deg = rad2deg(target_angle_rad);

  target_angle_deg = normalize_angle(target_angle_deg);

  rotate(target_angle_deg);

  float distance = calculate_distance(current_x, current_y, target_x, target_y);
  straight(distance, target_angle_deg);

  rotate(target_theta);
}
