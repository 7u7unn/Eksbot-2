#include <Wire.h>
const int MPU_ADDR = 0x68 //i2c address

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

void setup() {
   Serial.begin(115200);
   Wire.begin(21, 22, 100000); // sda, scl, clock speed
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x6B);  // PWR_MGMT_1 register
   Wire.write(0);     // set to zero (wakes up the MPU−6050)
   Wire.endTransmission(true);
   Serial.println("Setup complete");
}