
//IMU Deklarasi
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Variables for calibration and filtering
const int filter_window_size = 10;
const int calibration_samples = 1000;
int filter_index = 0;
bool filter_full = false;
unsigned long last_time;
float theta = 90.0;
// float theta = 90.0;

// float accelZ_calibration = 0, accelZ_avg = 0, accelZ_values[filter_window_size];
float gyroZ_calibration = 0, gyroZ_avg = 0, gyroZ_values[filter_window_size];

void init_mpu() {
  Wire.begin(21,22,10000);
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);//bisa ganti ke 500, 1000, 2000
    // mpu.setSleepEnabled(false);

  while (1){

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
  gyroZ_avg=0;
  int count = filter_full ? filter_window_size : filter_index;
  for (int i = 0; i < count; i++) {
    gyroZ_avg += gyroZ_values[i];
  }
  gyroZ_avg /= count;

  theta -= gyroZ_avg * dt;
  if(theta < 0) theta += 360;
  if(theta>=360) theta-=360;
  float theta_rad = theta*(PI/180);
  // Serial.println(gyroZ_raw);
  Serial.print("Deg: ");
  Serial.print(theta);
  Serial.print(" ");
  Serial.print("Rad: ");
  Serial.println(theta_rad);
}


void setup() {
  Serial.begin(115200);

  init_mpu();
}

void loop() {


  update_imu();
  delay(10);

}
