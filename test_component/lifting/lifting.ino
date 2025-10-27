#define IN1 19
#define IN2 18
#define IN3 13
#define IN4 14
#define ENA 21
#define ENB 27

bool start = 1;

void setup() {
  // put your setup code here, to run once:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // digitalWrite(IN1, LOW);
  // digitalWrite(in1, LOW);
  // digitalWrite(in1, LOW);
  // digitalWrite(in1, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (start) {

    setMotor(180, 0);
    delay(1000);
    setMotor(-170, 0);
    delay(300);
    setMotor(0,0);
    start = 0;
  }
}



void setMotor(int spdKanan, int spdKiri) {
  // Apply constraints with deadzone compensation
  // if (spdKiri != 0) {
  //   if (spdKiri > 0) {
  //     spdKiri = constrain(spdKiri, min_speed, max_speed);
  //   } else {
  //     spdKiri = constrain(spdKiri, -max_speed, -min_speed);
  //   }
  // }

  // if (spdKanan != 0) {
  //   if (spdKanan > 0) {
  //     spdKanan = constrain(spdKanan, min_speed, max_speed);
  //   } else {
  //     spdKanan = constrain(spdKanan, -max_speed, -min_speed);
  //   }
  // }

  // // Store PWM values for monitoring
  // current_pwm_right = spdKanan;
  // current_pwm_left = spdKiri;

  // Motor kanan
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

  // Motor kiri
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
