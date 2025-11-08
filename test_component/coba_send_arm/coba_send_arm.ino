#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

// === WiFi & MicroROS ===
const char* ssid = "hay";
const char* password = "jujundial7";
const char* agent_ip = "10.241.196.69"; 
const int agent_port = 8888;

// === MicroROS Objects ===
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
std_msgs__msg__String sub_msg;
std_msgs__msg__String pub_msg;

// === Serial to ESP32 #2 ===
#define SERIAL_BAUD 115200

// === State ===
bool command_in_progress = false;

// === Function Declarations ===
void subscription_callback(const void* msgin);
bool setup_microros();

void setup() {
  Serial.begin(SERIAL_BAUD); // Serial0 ke ESP32 #2
  // Serial.println("ESP32 #1: Serial to slave ready");

  // Connect to WiFi
  WiFi.begin(ssid, password);
  // Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Serial.println("\nWiFi connected");

  // Setup micro-ROS
  if (!setup_microros()) {
    // Serial.println("Failed to setup micro-ROS!");
    while (1) delay(1000);
  }

  // Serial.println("ESP32 #1 ready. Send PICK or PUT via micro-ROS.");
}

void loop() {
  // Cek respons dari ESP32 #2
  if (command_in_progress && Serial.available()) {
    String response = Serial.readStringUntil('\n');
    response.trim();

    if (response == "2") {
      // Kirim konfirmasi ke laptop via micro-ROS
      pub_msg.data.data = (char*)"Action completed";
      pub_msg.data.size = strlen(pub_msg.data.data);
      rcl_publish(&publisher, &pub_msg, NULL);
      // Serial.println("Action confirmed by slave");
      command_in_progress = false;
    }
  }

  // Spin micro-ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}

// === MicroROS Callback ===
void subscription_callback(const void* msgin) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
  String command = String(msg->data.data);
  command.trim();
  command.toUpperCase();

  Serial.print("Received command: "); Serial.println(command);

  if (command == "PICK1") {
    // Serial.println("Forwarding PICK to ESP32 #2...");
    Serial.println("11"); // Kirim "1" ke ESP32 #2
    command_in_progress = true;
  } 
  else if (command == "PICK2") {
    // Serial.println("Forwarding PUT to ESP32 #2...");
    Serial.println("10"); // Kirim "0" ke ESP32 #2
    command_in_progress = true;
  } 
  else if (command == "PUT1") {
    // Serial.println("Forwarding PUT to ESP32 #2...");
    Serial.println("01"); // Kirim "0" ke ESP32 #2
    command_in_progress = true;
  } 
  else if (command == "PUT2") {
    // Serial.println("Forwarding PUT to ESP32 #2...");
    Serial.println("00"); // Kirim "0" ke ESP32 #2
    command_in_progress = true;
  } 
  else {
    // Kirim error ke laptop
    pub_msg.data.data = (char*)"Unknown command. Use PICK or PUT";
    pub_msg.data.size = strlen(pub_msg.data.data);
    rcl_publish(&publisher, &pub_msg, NULL);
  }
}

// === Setup MicroROS ===
bool setup_microros() {
  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)agent_ip, agent_port);
  delay(2000);

  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "esp32_master_node", "", &support) != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "robot_arm_command") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "robot_arm_status") != RCL_RET_OK) return false;
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  sub_msg.data.data = (char*)malloc(50);
  sub_msg.data.capacity = 50;
  pub_msg.data.data = (char*)malloc(50);
  pub_msg.data.capacity = 50;

  return true;
}