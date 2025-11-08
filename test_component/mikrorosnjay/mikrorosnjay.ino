// SENSOR GARIS
#define NUM_LINE_SENSORS 12
int line_threshold[NUM_LINE_SENSORS];
int line_values[NUM_LINE_SENSORS];
bool line_calibrated = false;

int s0 = 25, s1 = 33, s2 = 32, s3 = 26, SIG_pin = 34;
// Function declarations
// void setMotor(int spdKanan, int spdKiri);
void init_mux();
int readMux(int ch);
void read_all_line_sensors();
void calibrate_line_sensors(int samples = 2000);
void load_line_thresholds();
void save_line_thresholds();
void print_thresholds();
void linefol(bool depan);
float calculatePosition(bool depan);
void auto_calib(int step);


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
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

// WiFi credentials
const char *ssid = "hay";
const char *password = "jujundial7";

// micro-ROS agent IP (your PC's IP address) - as STRING
const char *agent_ip = "10.241.196.69";  // Change to your PC's IP
const int agent_port = 8888;

// micro-ROS objects
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
std_msgs__msg__String sub_msg;
std_msgs__msg__String pub_msg;

// Robot state
enum RobotState {
  ROBOT_STOPPED,
  ROBOT_RUNNING
};
RobotState robot_state = ROBOT_STOPPED;

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
float gearbox_L = 49.0;
float L = 30.0;
float diameter = 7.0;
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
SemaphoreHandle_t stateMutex;

TaskHandle_t taskOdomHandle = NULL;
TaskHandle_t taskSerialHandle = NULL;
TaskHandle_t taskNavHandle = NULL;
TaskHandle_t taskMicroRosHandle = NULL;

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
float initial_heading = 0;
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

float manuver_start_angle = 0;
float manuver_target_delta = 180.0;
bool kanan = false;

// PID parameters
float Kp_rot = 2;
float Ki_rot = 0.0;
float Kd_rot = 3.5;
float integral_rot = 0.0;
float prev_error_rot = 0.0;

float Kp_linear = 1.5;
float Ki_linear = 0.000;
float Kd_linear = 0.02;
float integral_linear = 0.0;
float prev_error_linear = 0.0;

float Kp_angular = 4.4;
float Ki_angular = 0.0;
float Kd_angular = 1;
float integral_angular = 0.0;
float prev_error_angular = 0.0;

// Speed constraints
float distance_threshold = 1.0;
float angle_threshold = 0.5;
float rotation_speed = 150;
float max_speed = 220;
float min_speed = 125;

float integral_max = 2000.0;
float integral_min = -2000.0;

volatile int current_pwm_right = 0;
volatile int current_pwm_left = 0;

// int serial_cmd = 9;

// Function declarations
void IRAM_ATTR Read_R();
void IRAM_ATTR Read_L();
void update_odom();
void update_imu();
void calib_imu();
void taskOdometry(void *parameter);
void taskSerialPrint(void *parameter);
void taskNavigation(void *parameter);
void taskMicroRos(void *parameter);
void init_motor();
void setMotor(int spdKanan, int spdKiri);

void sendcmd(String cmd);
void waitForLift();

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

// micro-ROS functions
void subscription_callback(const void *msgin);
bool setup_microros();
void destroy_microros();
bool ccw = 1;

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

  // Connect to WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Wait before calibration
  uint16_t start_time = millis();
  while (millis() - start_time < 3000) {
    continue;
  }
  calib_imu();

  odomMutex = xSemaphoreCreateMutex();
  navMutex = xSemaphoreCreateMutex();
  stateMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskOdometry, "OdometryTask", 4096, NULL, 2, &taskOdomHandle, 1);
  xTaskCreatePinnedToCore(taskNavigation, "NavigationTask", 4096, NULL, 2, &taskNavHandle, 1);
  // xTaskCreatePinnedToCore(taskSerialPrint, "SerialTask", 2048, NULL, 1, &taskSerialHandle, 0);
  xTaskCreatePinnedToCore(taskMicroRos, "MicroRosTask", 8192, NULL, 1, &taskMicroRosHandle, 0);

  Serial.println("Robot initialized. Waiting for START command...");
}

void loop() {
  if (robot_state == ROBOT_RUNNING && !finish && kanan == 1) {
    straight(15, 90);
    delay(500);
    rotate(180);
    delay(500);
    // rotate(180);
    // delay(500);
    straight(44, 180);  // Move forward 50 cm
    // // manuver();
    delay(500);
    // rotate(135);
    // delay(500);
    // straight(120, -90);
    rotate(90);
    delay(500);
    straight(160, 90);  // Move forward 50 cm
    delay(500);
    manuver(1);
    delay(500);
    straight(160, -90);
    delay(500);
    manuver(1);
    // sendcmd("1");
    // // waitForLift();
    // delay(1000);
    // sendcmd("0");
    // waitForLift();

    finish = true;

    // Notify completion
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100))) {
      robot_state = ROBOT_STOPPED;
      xSemaphoreGive(stateMutex);
    }
    Serial.println("Mission complete! Send START to run again.");
  } else if (robot_state == ROBOT_RUNNING && !finish && kanan == 0) {
    straight(15, 90);
    // delay(500);
    // rotate(45);
    delay(500);
    rotate(0);
    delay(500);
    straight(44, 0);  // Move forward 50 cm
    // // manuver();
    // delay(500);
    // rotate(45);
    delay(500);
    // straight(120, -90);
    rotate(90);
    delay(500);
    straight(160, 90);  // Move forward 50 cm
    delay(500);
    manuver(0);
    straight(160, -90);
    delay(500);
    manuver(0);
    // sendcmd("1");
    // // waitForLift();
    // delay(1000);
    // sendcmd("0");
    // waitForLift();

    finish = true;

    // Notify completion
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100))) {
      robot_state = ROBOT_STOPPED;
      xSemaphoreGive(stateMutex);
    }
    Serial.println("Mission complete! Send START to run again.");
  }
  delay(100);
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
  Wire.begin(21, 22, 100000);
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
            break;
          // case CMD_LINEFOL:
          //   exec_linefol();
          default:
            break;
        }
      }
      xSemaphoreGive(navMutex);
    }

    vTaskDelayUntil(&xLastTime, xFrequency);
  }
}

void sendcmd(String cmd) {
  unsigned long timeout = millis() + 5000;

  Serial.println(cmd);  // Send command once

  while (millis() < timeout) {
    if (Serial.available()) {
      String ack = Serial.readStringUntil('\n');
      ack.trim();
      if (ack == "2") {
        return;  // Success
      }
    }
    delay(10);
  }

  // Timeout occurred
  Serial.println("ERROR: Slave timeout!");
}
// void waitForLift() {
//   unsigned long timeout = millis() + 5000; // 5 sec timeout
//   while (1) {
//     if (Serial.available()) {
//       String ack = Serial.readStringUntil('\n');
//       ack.trim();
//       if ( ack== "2") {
//         return ; // Success
//       }
//     }
//     delay(10);
//   }
//   // Serial.println("Lift command timeout!");
// }

void taskSerialPrint(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50);

  for (;;) {
    // if (serial_cmd == 1) {
    //   Serial.println("1");
    //   while(!Serial.available());
    //   serial_cmd = 9;

    // }
    // else if (serial_cmd == 0){
    //   Serial.println("0");
    //   while(!Serial.available());
    //   serial_cmd=9;
    // }
    // if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    //   Serial.print("Encoder R: ");
    //   Serial.print(val_R);
    //   Serial.print(" | Encoder L: ");
    //   Serial.print(val_L);
    //   Serial.print(" | X: ");
    //   Serial.print(x, 2);
    //   Serial.print(" cm | Y: ");
    //   Serial.print(y, 2);
    //   Serial.print(" cm | Theta: ");
    //   Serial.print(rad2deg(theta), 2);
    //   Serial.print(" deg | Theta IMU: ");
    //   Serial.print(rad2deg(theta_imu), 2);
    //   Serial.print(" deg | PWM_R: ");
    //   Serial.print(current_pwm_right);
    //   Serial.print(" | PWM_L: ");
    //   Serial.print(current_pwm_left);
    //   Serial.print(" | State: ");

    //   if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    //     switch (nav_state) {
    //       case ROTATING: Serial.print("ROTATING"); break;
    //       case MOVING_STRAIGHT: Serial.print("MOVING_STRAIGHT"); break;
    //       case IDLE: Serial.print("IDLE"); break;
    //       case MANUVER: Serial.print("MANUVER"); break;
    //     }
    //     xSemaphoreGive(navMutex);
    //   }

    //   Serial.print(" | Robot: ");
    //   if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5))) {
    //     Serial.print(robot_state == ROBOT_RUNNING ? "RUNNING" : "STOPPED");
    //     xSemaphoreGive(stateMutex);
    //   }
    //   Serial.println();

    //   xSemaphoreGive(odomMutex);
    // }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void taskMicroRos(void *parameter) {
  // Setup micro-ROS
  if (!setup_microros()) {
    Serial.println("Failed to setup micro-ROS!");
    vTaskDelete(NULL);
    return;
  }

  Serial.println("micro-ROS ready!...");

  for (;;) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    delay(10);
  }
}

// ==================== micro-ROS FUNCTIONS ====================

void subscription_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

  String command = String(msg->data.data);
  command.toUpperCase();
  command.trim();

  Serial.print("Received command: ");
  Serial.println(command);

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100))) {
    if (command == "START-KANAN") {
      // init_imu();
      robot_state = ROBOT_RUNNING;
      kanan = true;
      finish = false;  // Reset mission flag
      Serial.println("Robot STARTED KANAN!");

      // Publish status
      pub_msg.data.data = (char *)"Robot started";
      pub_msg.data.size = strlen(pub_msg.data.data);
      rcl_publish(&publisher, &pub_msg, NULL);

    } else if (command == "START-KIRI") {
      robot_state = ROBOT_RUNNING;
      kanan = false;
      finish = false;  // Reset mission flag
      Serial.println("Robot STARTED KIRI!");

      // Publish status
      pub_msg.data.data = (char *)"Robot started";
      pub_msg.data.size = strlen(pub_msg.data.data);
      rcl_publish(&publisher, &pub_msg, NULL);

    } else if (command == "STOP") {
      robot_state = ROBOT_STOPPED;
      idle();  // Stop all motors
      Serial.println("Robot STOPPED!");

      // Publish status
      pub_msg.data.data = (char *)"Robot stopped";
      pub_msg.data.size = strlen(pub_msg.data.data);
      rcl_publish(&publisher, &pub_msg, NULL);

    } else if (command == "CALIB") {
      calib_imu();
      robot_state = ROBOT_STOPPED;
      idle();
      Serial.println("Robot calibrated!");

      // Publish status
      pub_msg.data.data = (char *)"Robot calibrated";
      pub_msg.data.size = strlen(pub_msg.data.data);
      rcl_publish(&publisher, &pub_msg, NULL);
    } else {
      Serial.println("Unknown command. Use START or STOP");
    }
    xSemaphoreGive(stateMutex);
  }
}

bool setup_microros() {
  // Set WiFi transport with IP as string
  Serial.print("Connecting to micro-ROS agent at ");
  Serial.print(agent_ip);
  Serial.print(":");
  Serial.println(agent_port);

  set_microros_wifi_transports((char *)ssid, (char *)password, (char *)agent_ip, agent_port);

  delay(2000);  // Wait for connection

  allocator = rcl_get_default_allocator();
  Serial.println("Allocator created");

  // Create init_options
  Serial.println("Initializing support...");
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("Failed to initialize support");
    return false;
  }
  Serial.println("Support initialized");

  // Create node
  Serial.println("Creating node...");
  if (rclc_node_init_default(&node, "esp32_robot_node", "", &support) != RCL_RET_OK) {
    Serial.println("Failed to create node");
    return false;
  }
  Serial.println("Node created");
  Serial.println("Node created");

  // Create subscriber
  Serial.println("Creating subscriber...");
  if (rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "robot_command")
      != RCL_RET_OK) {
    Serial.println("Failed to create subscriber");
    return false;
  }
  Serial.println("Subscriber created");

  // Create publisher
  Serial.println("Creating publisher...");
  if (rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "robot_status")
      != RCL_RET_OK) {
    Serial.println("Failed to create publisher");
    return false;
  }
  Serial.println("Publisher created");

  // Create executor
  Serial.println("Creating executor...");
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    Serial.println("Failed to create executor");
    return false;
  }
  Serial.println("Executor created");

  // Add subscription to executor
  Serial.println("Adding subscription to executor...");
  if (rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("Failed to add subscription");
    return false;
  }
  Serial.println("Subscription added");

  // Allocate memory for messages
  sub_msg.data.data = (char *)malloc(100 * sizeof(char));
  sub_msg.data.size = 0;
  sub_msg.data.capacity = 100;

  pub_msg.data.data = (char *)malloc(100 * sizeof(char));
  pub_msg.data.size = 0;
  pub_msg.data.capacity = 100;

  return true;
}

// ==================== COMMAND FUNCTIONS ====================

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
    delay(100);
  }
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

  while (!is_command_done()) {
    delay(100);
  }
}

void manuver(bool ccw_dir) {
  if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    if (xSemaphoreTake(odomMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
      manuver_start_angle = rad2deg(theta_imu);
      xSemaphoreGive(odomMutex);
    }

    current_command = CMD_MANUVER;
    ccw = ccw_dir;
    command_active = true;
    nav_state = MANUVER;

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
  linear_speed = constrain(linear_speed, min_speed + 5, max_speed + 5) * direction;

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

    if (rotation_correction > 0 && rotation_correction < 96) rotation_correction = 96;
    else if (rotation_correction < 0 && rotation_correction > -96) rotation_correction = -96;

    setMotor(rotation_correction, -rotation_correction);
  } else {
    setMotor(0, 0);
    command_active = false;
    nav_state = IDLE;
    integral_rot = 0;
    prev_error_rot = 0;
  }
}

void exec_manuver(float current_theta_imu) {
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
    int left_speed = constrain((int)pid_output, min_speed + 5, max_speed + 5);
    if (ccw == 1) {

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


// void print_thresholds() {
//   Serial.println("Current thresholds:");
//   for (int i = 0; i < NUM_LINE_SENSORS; i++) {
//     Serial.print("Sensor ");
//     Serial.print(i);
//     Serial.print(": ");
//     Serial.println(line_threshold[i]);
//   }
//   Serial.println();
// }

// // --- Rest of your functions unchanged ---
// void init_mux() {
//   pinMode(s0, OUTPUT);
//   pinMode(s1, OUTPUT);
//   pinMode(s2, OUTPUT);
//   pinMode(s3, OUTPUT);
// }

// int readMux(int channel) {
//   int pins[] = { s0, s1, s2, s3 };
//   int sel[12][4] = {
//     { 0, 0, 0, 0 }, { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 1, 1, 0, 0 }, { 0, 0, 1, 0 }, { 1, 0, 1, 0 }, { 0, 1, 1, 0 }, { 1, 1, 1, 0 }, { 0, 0, 0, 1 }, { 1, 0, 0, 1 }, { 0, 1, 0, 1 }, { 1, 1, 0, 1 }
//   };
//   for (int i = 0; i < 4; i++) {
//     digitalWrite(pins[i], sel[channel][i]);
//   }
//   return analogRead(SIG_pin);
// }

// void read_all_line_sensors() {
//   for (int i = 0; i < NUM_LINE_SENSORS; i++) {
//     line_values[i] = readMux(i);
//     delay(2);
//   }
// }

// bool is_on_line(int i) {
//   return line_values[i] < line_threshold[i];
// }

// // int get_line_position(bool use_front) {
// //   int weightedSum = 0;
// //   int sum = 0;
// //   int start = use_front ? 6 : 0;
// //   for (int i = 0; i < 6; i++) {
// //     weightedSum += is_on_line(start + i) * (i * 1000);
// //     sum += is_on_line(start + i);
// //   }
// //   if (sum == 0) return 2500;
// //   return weightedSum / sum;
// // }

// float calculatePosition(bool depan) {

//   // Weighted average untuk menentukan posisi garis
//   float weightedSum = 0;
//   int sum = 0;
//   int start = depan ? 6 : 0;

//   for (int i = 0; i < 6; i++) {
//     weightedSum += is_on_line(start + i) * (i * 1000);
//     sum += is_on_line(start + i);
//   }
//   if (sum == 0) return 2500;

//   return weightedSum / sum;
// }

// void calibrate_line_sensors(int samples) {
//   int minv[12], maxv[12];
//   for (int i = 0; i < 12; i++) {
//     minv[i] = 4095;
//     maxv[i] = 0;
//   }
//   for (int s = 0; s < samples; s++) {
//     read_all_line_sensors();
//     for (int i = 0; i < 12; i++) {
//       if (line_values[i] < minv[i]) minv[i] = line_values[i];
//       if (line_values[i] > maxv[i]) maxv[i] = line_values[i];
//     }
//     delay(10);
//   }
//   for (int i = 0; i < 12; i++) {
//     line_threshold[i] = (minv[i] + maxv[i]) / 2.5;
//   }
//   save_line_thresholds();
//   line_calibrated = true;
// }

// void load_line_thresholds() {
//   for (int i = 0; i < 12; i++) {
//     EEPROM.get(i * sizeof(int), line_threshold[i]);
//   }
// }

// void save_line_thresholds() {
//   for (int i = 0; i < 12; i++) {
//     EEPROM.put(i * sizeof(int), line_threshold[i]);
//   }
//   EEPROM.commit();
// }


// void linefol(bool depan) {
//   float Position = calculatePosition(depan);

//   // float position = (FILTER_WEIGHT * rawPosition) + ((1 - FILTER_WEIGHT) * lastPosition);

//   error = 2500 - Position;
//   lastPosition = Position;

//   // Update integral dengan batasan untuk mencegah integral windup
//   integral = constrain(integral + error, -10000, 10000);

//   derivative = (error - lastError);

//   // Hitung output PID
//   float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

//   lastError = error;

//   int leftSpeed, rightSpeed;
//   int minSpeed = 95;

//   if (output < 0) {
//     rightSpeed = minSpeed + abs(output);
//     leftSpeed = minSpeed;
//   } else {
//     rightSpeed = minSpeed;
//     leftSpeed = minSpeed + abs(output);
//   }

//   leftSpeed = constrain(leftSpeed, minSpeed, 180);
//   rightSpeed = constrain(rightSpeed, minSpeed, 180);

//   // Serial.printf("Kiri: %d   ||   Kanan: %d\n", leftSpeed, rightSpeed);
// }

// void auto_calib(int step) {
//   for (int i = 0; i < step; i++) {
//     calib_start = millis();

//     while (millis() - calib_start <= 1500) {
//       if (millis() - calib_start <= 700) setMotor(100, 100);
//       else if (millis() - calib_start <= 750) setMotor(0, 0);
//       else if (millis() - calib_start <= 1500) setMotor(-100, -100);
//       else setMotor(0, 0);
//       calibrate_line_sensors(1);
//     }
//   }
// }
