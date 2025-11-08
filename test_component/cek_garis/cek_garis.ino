#include <EEPROM.h>

#define NUM_LINE_SENSORS 12
int line_threshold[NUM_LINE_SENSORS];
int line_values[NUM_LINE_SENSORS];
bool line_calibrated = false;

int s0 = 25, s1 = 33, s2 = 32, s3 = 26, SIG_pin = 34;
int minv[12];
int maxv[12];


// Function declarations
void init_mux();
int readMux(int ch);
void read_all_line_sensors();
void calibrate_line_sensors(int samples = 2000);
void load_line_thresholds();
void save_line_thresholds();
void print_thresholds();

void setup() {
  for (int i = 0; i < 12; i++) {
    minv[i] = 4095;
    maxv[i] = 0;
  }
  Serial.begin(115200);
  EEPROM.begin(512);
  init_mux();

  Serial.println("\nLine Sensor Setup");
  Serial.println("Send 'c' to calibrate, 'l' to load from EEPROM:");

  while (!Serial.available()) {
    delay(100);  // Wait for input
  }

  char cmd = Serial.read();
  Serial.flush();  // Clear any extra input

  if (cmd == 'c' || cmd == 'C') {
    Serial.println("Calibrating line sensors... (place over full white & black surfaces)");
    delay(2000);  // Give user time to get ready
    calibrate_line_sensors(500);
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
  // read_all_line_sensors();

  // // Print semua sensor dalam format 1 atau 0
  // Serial.print("Sensors: ");
  // for (int i = 0; i < NUM_LINE_SENSORS; i++) {
  //   Serial.print(is_on_line(i) ? "1" : "0");
  //   if (i < NUM_LINE_SENSORS - 1) Serial.print(" ");
  // }
  // Serial.println();

  // Optional: tetap print posisi line jika diperlukan
  // int frontPos = get_line_position(true);   // Front: sensors 6–11
  // int rearPos  = get_line_position(false);  // Rear: sensors 0–5

  // Serial.print("Front: ");
  // Serial.print(frontPos);
  // Serial.print(" | Rear: ");
  // Serial.println(rearPos);

  // delay(100);
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
  Serial.println("Current min:");
  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(minv[i]);
  }
  Serial.println("Current max:");
  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(maxv[i]);
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
    { 0, 0, 0, 0 }, { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 1, 1, 0, 0 }, { 0, 0, 1, 0 }, { 1, 0, 1, 0 }, { 0, 1, 1, 0 }, { 1, 1, 1, 0 }, { 0, 0, 0, 1 }, { 1, 0, 0, 1 }, { 0, 1, 0, 1 }, { 1, 1, 0, 1 }
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

int get_line_position(bool use_front) {
  int weightedSum = 0;
  int sum = 0;
  int start = use_front ? 6 : 0;
  for (int i = 0; i < 6; i++) {
    weightedSum += is_on_line(start + i) * (i * 1000);
    sum += is_on_line(start + i);
  }
  if (sum == 0) return 2500;
  return weightedSum / sum;
}

void calibrate_line_sensors(int samples) {
  // int minv[12], maxv[12];
  // for (int i = 0; i < 12; i++) {
  //   minv[i] = 4095;
  //   maxv[i] = 0;
  // }
  for (int s = 0; s < samples; s++) {
    read_all_line_sensors();
    for (int i = 0; i < 12; i++) {
      if (line_values[i] < minv[i]) minv[i] = line_values[i];
      if (line_values[i] > maxv[i]) maxv[i] = line_values[i];
    }
    delay(10);
  }
  for (int i = 0; i < 12; i++) {
    line_threshold[i] = (minv[i] + maxv[i]) / 1.8;
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