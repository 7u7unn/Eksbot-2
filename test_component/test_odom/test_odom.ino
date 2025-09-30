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
float diameter = 6.5;
float wheel_k = (PI * diameter);  // cm

// Position variables
float x = 0.0;    // cm
float y = 0.0;    // cm
float theta = 0;  // rad

// Ratio tics_to_cm (ttc)
float ttc_R = wheel_k / (gearbox_R * ppr);
float ttc_L = wheel_k / (gearbox_L * ppr);


SemaphoreHandle_t odomMutex;
TaskHandle_t taskOdomHandle = NULL;
TaskHandle_t taskSerialHandle = NULL;

// Function declarations - ADDED missing taskIMU declaration
void IRAM_ATTR Read_R();
void IRAM_ATTR Read_L();
void update_odom();
void taskOdometry(void *parameter);
void taskSerialPrint(void *parameter);

void setup() {
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  attachInterrupt(encA1, Read_R, RISING);

  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(encA2, Read_L, RISING);

  Serial.begin(115200);

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

  // Task for serial printing (lower priority)
  xTaskCreatePinnedToCore(
    taskSerialPrint,   // Task function
    "SerialTask",      // Name
    4096,              // Stack size
    NULL,              // Parameters
    0,                 // Priority (lower)
    &taskSerialHandle  // Task handle
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
  x += dAvg * cos(theta);
  y += dAvg * sin(theta);
}

void taskOdometry(void *parameter) {
  TickType_t xLastTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20) //50Hz 

  for (;;) {
    if (xSemaphoreTake(odomMutex, portMAX_DELAY)) {
      update_odom();
      xSemaphoreGive(odomMutex);
    }

    vTaskDelayUntil(&LastTime, xFrequency);
  }
}

void taskSerialPrint(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(200);  // 10Hz (100ms)

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
      Serial.print(" deg");
      xSemaphoreGive(odomMutex);


      // Wait for next cycle
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }