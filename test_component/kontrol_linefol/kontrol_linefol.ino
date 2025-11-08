#include <EEPROM.h>

#define ENA 18
#define ENB 17
#define IN1 23
#define IN2 4
#define IN3 13
#define IN4 19

#define NUM_LINE_SENSORS 12
int line_threshold[NUM_LINE_SENSORS];
int line_values[NUM_LINE_SENSORS];
bool line_calibrated = false;

int s0 = 25, s1 = 33, s2 = 32, s3 = 26, SIG_pin = 34;

// Function declarations
void setMotor(int spdKanan, int spdKiri);
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


//PARAMETER PID
float Kp = 0.03;   // Dikurangi sedikit agar tidak terlalu agresif
float Ki = 0.000;  // Ditambahkan untuk mengurangi steady-state error
float Kd = 0.0;    // Ditingkatkan untuk antisipasi perubahan error
int biastae = 0;
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float lastPosition = 2500;

unsigned long calib_start = 0;


void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  init_mux();

  Serial.println("\nLine Sensor Setup");
  Serial.println("Send 'c' to calibrate, 'l' to load from EEPROM:");

  while (!Serial.available()) {
    delay(100); // Wait for input
  }

  char cmd = Serial.read();
  Serial.flush(); // Clear any extra input

  if (cmd == 'c' || cmd == 'C') {
    Serial.println("Calibrating line sensors... (place over full white & black surfaces)");
    delay(2000); // Give user time to get ready
    calibrate_line_sensors(500);
    // auto_calib(4);
    Serial.println("✅ Calibration complete.");
  } else if (cmd == 'l' || cmd == 'L') {
    load_line_thresholds();
    Serial.println("✅ Loaded thresholds from EEPROM.");
  } else {
    Serial.println("!! Invalid command. Defaulting to EEPROM load. !!");
    load_line_thresholds();
  }

  print_thresholds();
}

void loop() {
  read_all_line_sensors();
  // linefol(1);


  // Print semua sensor dalam format 1 atau 0
  Serial.print("Sensors: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(is_on_line(i) ? "1" : "0");
    if (i < NUM_LINE_SENSORS - 1) Serial.print(" ");
  }
  Serial.println(calculatePosition(0));
  Serial.println();

  // Optional: tetap print posisi line jika diperlukan
  // int frontPos = get_line_position(true);   // Front: sensors 6–11
  // int rearPos  = get_line_position(false);  // Rear: sensors 0–5

  // Serial.print("Front: ");
  // Serial.print(frontPos);
  // Serial.print(" | Rear: ");
  // Serial.println(rearPos);

  delay(100);
}

// --- Helper to print thresholds ---
void print_thresholds() {
  Serial.println("Current thresholds:");
  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(line_threshold[i]);
  }
  Serial.println();
}

// --- Rest of your functions unchanged ---
void init_mux() {
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
}

int readMux(int channel) {
  int pins[] = { s0, s1, s2, s3 };
  int sel[12][4] = {
    { 0, 0, 0, 0 }, { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 1, 1, 0, 0 },
    { 0, 0, 1, 0 }, { 1, 0, 1, 0 }, { 0, 1, 1, 0 }, { 1, 1, 1, 0 },
    { 0, 0, 0, 1 }, { 1, 0, 0, 1 }, { 0, 1, 0, 1 }, { 1, 1, 0, 1 }
  };
  for (int i = 0; i < 4; i++) {
    digitalWrite(pins[i], sel[channel][i]);
  }
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

// int get_line_position(bool use_front) {
//   int weightedSum = 0;
//   int sum = 0;
//   int start = use_front ? 6 : 0;
//   for (int i = 0; i < 6; i++) {
//     weightedSum += is_on_line(start + i) * (i * 1000);
//     sum += is_on_line(start + i);
//   }
//   if (sum == 0) return 2500;
//   return weightedSum / sum;
// }

float calculatePosition(bool depan) {

  // Weighted average untuk menentukan posisi garis
  float weightedSum = 0;
  int sum = 0;
  int start = depan ? 6 : 0;

  for (int i = 0; i < 6; i++) { 
    weightedSum += is_on_line(start + i) * (i * 1000);
    sum += is_on_line(start + i);
  }
  if (sum == 0) return 2500;

  return weightedSum / sum;
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


void linefol(bool depan) {
  float Position = calculatePosition(depan);

  // float position = (FILTER_WEIGHT * rawPosition) + ((1 - FILTER_WEIGHT) * lastPosition);

  error = 2500 - Position;
  lastPosition = Position;

  // Update integral dengan batasan untuk mencegah integral windup
  integral = constrain(integral + error, -10000, 10000);

  derivative = (error - lastError);

  // Hitung output PID
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  lastError = error;

  int leftSpeed, rightSpeed;
  int minSpeed = 120;
  int maxSpeed = 230;

  if (output < 0) {
    rightSpeed = minSpeed + abs(output);
    leftSpeed = minSpeed ;  
  } else {
    rightSpeed = minSpeed;  
    leftSpeed = minSpeed + abs(output);
  }

  leftSpeed = constrain(leftSpeed, minSpeed, 180);
  rightSpeed = constrain(rightSpeed, minSpeed, 180);
  setMotor(rightSpeed,leftSpeed);

  Serial.printf("Kiri: %d   ||   Kanan: %d\n", leftSpeed, rightSpeed);
}

void auto_calib(int step) {
  for (int i = 0; i < step; i++) {
    calib_start = millis();

    while (millis() - calib_start <= 1500) {
      if (millis() - calib_start <= 700) setMotor(100, 100);
      else if (millis() - calib_start <= 750) setMotor(0,0);
      else if (millis() - calib_start <= 1500) setMotor(-100, -100);
      else setMotor(0,0);
      calibrate_line_sensors(1);
    }
  }
}


void setMotor(int spdKanan, int spdKiri) {
  // current_pwm_right = spdKanan;
  // current_pwm_left = spdKiri;

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
