#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <EEPROM.h>

// === WiFi & MicroROS ===
const char* ssid = "AIRLANGGA-HOTSPOT";
const char* password = "@irlangg@";
const char* agent_ip = "172.16.207.74";  // Ganti dengan IP laptop Anda
const int agent_port = 8888;

// === Line Sensor ===
#define NUM_LINE_SENSORS 12
int line_threshold[NUM_LINE_SENSORS];
int line_values[NUM_LINE_SENSORS];
bool line_calibrated = false;

int s0 = 25, s1 = 33, s2 = 32, s3 = 26, SIG_pin = 34;

// === Motor ===
#define ENA 18
#define ENB 17
#define IN1 23
#define IN2 4
#define IN3 13
#define IN4 19

// === MicroROS Objects ===
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
std_msgs__msg__String sub_msg;
std_msgs__msg__String pub_msg;

// === Line Follower PID ===
float Kp = 0.02;
float Ki = 0.0;
float Kd = 0.002;
float error = 0, lastError = 0, integral = 0, lastPosition = 2500;

// === State ===
bool linefollower_active = false;

// === Function Declarations ===
void subscription_callback(const void* msgin);
bool setup_microros();

// Motor & Sensor Functions
void setMotor(int spdKanan, int spdKiri);
void init_mux();
int readMux(int ch);
void read_all_line_sensors();
void calibrate_line_sensors(int samples = 2000);
void load_line_thresholds();
void save_line_thresholds();
bool is_on_line(int i);
float calculatePosition(bool depan);
void auto_calib(int step);
void execute_linefol();

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Line sensor mux
  init_mux();
  load_line_thresholds();
  Serial.println("Line thresholds loaded.");

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // MicroROS
  if (!setup_microros()) {
    Serial.println("MicroROS setup failed!");
    while (1) delay(1000);
  }

  Serial.println("LineFollower MicroROS ready. Send: LINEFOL, CALIB_LINE, STOP");
}

void loop() {
  // Jalankan line follower jika aktif
  if (linefollower_active) {
    execute_linefol();
  }

  // Spin MicroROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}

// ==================== MOTOR & SENSOR FUNCTIONS ====================
void setMotor(int spdKanan, int spdKiri) {
  if (spdKanan > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else if (spdKanan < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, constrain(abs(spdKanan), 0, 255));

  if (spdKiri > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (spdKiri < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, constrain(abs(spdKiri), 0, 255));
}

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
  float weightedSum = 0;
  int sum = 0;
  int start = depan ? 6 : 0;
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
      if (millis() - start <= 700) {
        setMotor(100, 100);
      } else if (millis() - start <= 750) {
        setMotor(0, 0);
      } else if (millis() - start <= 1500) {
        setMotor(-100, -100);
      } else {
        setMotor(0, 0);
      }
      calibrate_line_sensors(1);
    }
  }
  line_calibrated = true;
}

void execute_linefol() {
  read_all_line_sensors();
  float position = calculatePosition(0); // sensor depan
  error = position-2500;
  lastPosition = position;

  integral = constrain(integral + error, -10000, 10000);
  float derivative = error - lastError;
  lastError = error;

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  int minSpeed = 120;
  int leftSpeed, rightSpeed;

  if (output < 0) {
    rightSpeed = minSpeed + abs(output);
    leftSpeed = minSpeed;
  } else {
    rightSpeed = minSpeed;
    leftSpeed = minSpeed + abs(output);
  }

  leftSpeed = constrain(leftSpeed, minSpeed, 230);
  rightSpeed = constrain(rightSpeed, minSpeed, 200);

  setMotor(rightSpeed, leftSpeed);
}

// ==================== MICRO-ROS ====================
void subscription_callback(const void* msgin) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
  String command = String(msg->data.data);
  command.trim();
  command.toUpperCase();

  Serial.print("Received: "); Serial.println(command);

  if (command.startsWith("PID")) {
    // Format: "PID Kp Ki Kd"
    // Example: "PID 0.017 0.0 0.005"
    float newKp = Kp, newKi = Ki, newKd = Kd;
    bool success = false;

    // Split string manually (Arduino String has no robust split)
    int first = command.indexOf(' ');
    int second = command.indexOf(' ', first + 1);
    int third = command.indexOf(' ', second + 1);

    if (first != -1 && second != -1 && third != -1) {
      String kpStr = command.substring(first + 1, second);
      String kiStr = command.substring(second + 1, third);
      String kdStr = command.substring(third + 1);

      newKp = kpStr.toFloat();
      newKi = kiStr.toFloat();
      newKd = kdStr.toFloat();

      // Validate (non-negative, reasonable range)
      if (newKp >= 0 && newKp <= 1.0 &&
          newKi >= 0 && newKi <= 0.1 &&
          newKd >= 0 && newKd <= 1.0) {
        Kp = newKp;
        Ki = newKi;
        Kd = newKd;
        success = true;
      }
    }

    if (success) {
      char buffer[100];
      snprintf(buffer, sizeof(buffer), "PID updated: Kp=%.4f Ki=%.4f Kd=%.4f", Kp, Ki, Kd);
      pub_msg.data.data = buffer;
    } else {
      pub_msg.data.data = (char*)"Invalid PID format. Use: PID Kp Ki Kd (e.g., PID 0.02 0.0 0.01)";
    }
  }
  else if (command == "LINEFOL") {
    if (!line_calibrated) {
      pub_msg.data.data = (char*)"Line sensors not calibrated! Using EEPROM values.";
    } else {
      pub_msg.data.data = (char*)"Line follower started";
    }
    linefollower_active = true;
    integral = lastError = 0;
    lastPosition = 2500;
  } 
  else if (command == "CALIB") {
    pub_msg.data.data = (char*)"Calibrating...";
    rcl_publish(&publisher, &pub_msg, NULL);
    calibrate_line_sensors(1000);
    pub_msg.data.data = (char*)"Calibration done";
  } 
  else if (command == "STOP") {
    linefollower_active = false;
    setMotor(0, 0);
    pub_msg.data.data = (char*)"Stopped";
  } 
  else {
    pub_msg.data.data = (char*)"Unknown: use LINEFOL, CALIB, STOP, or PID Kp Ki Kd";
  }

  pub_msg.data.size = strlen(pub_msg.data.data);
  rcl_publish(&publisher, &pub_msg, NULL);
}

bool setup_microros() {
  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)agent_ip, agent_port);
  delay(2000);

  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "linefollower_node", "", &support) != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "line_command") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "line_status") != RCL_RET_OK) return false;
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  sub_msg.data.data = (char*)malloc(50);
  sub_msg.data.capacity = 50;
  pub_msg.data.data = (char*)malloc(50);
  pub_msg.data.capacity = 50;

  return true;
}