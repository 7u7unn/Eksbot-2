#define ENA 18
#define ENB 17
#define IN1 23
#define IN2 4
#define IN3 13
#define IN4 19

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(115200);
}

void setMotor(int spdKanan, int spdKiri) {
  // Motor kanan
  if (spdKanan > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (spdKanan < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else { // berhenti
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
  } else { // berhenti
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, constrain(abs(spdKiri), 0, 255));
}

void loop() {
  // contoh penggunaan
  setMotor(200, 200);  // maju lurus
  delay(2000);

  setMotor(-200, -200); // mundur lurus
  delay(2000);

  setMotor(200, -200); // belok kanan
  delay(2000);

  setMotor(-200, 200); // belok kiri
  delay(2000);

  setMotor(0, 0); // berhenti
  delay(2000);
}
