#define IN1 19
#define IN2 18
#define IN3 13
#define IN4 14
#define ENA 21
#define ENB 27
#define atas 26
#define bawah 25

int lim_atas = 1;
int lim_bawah = 1;

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(atas, INPUT_PULLUP);
  pinMode(bawah, INPUT_PULLUP);
}

void loop() {
  // lim_bawah = digitalRead(bawah);
  // lim_atas = digitalRead(atas);
  // Serial.printf("atas: %d || bawah %d", lim_atas, lim_bawah);
  // Serial.println();
  // delay(10);


  pick();
  delay(3000);
  put();
  delay(3000);
}

void setMotor(int spdKanan, int spdKiri) {
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

void pick() {
  lim_atas = digitalRead(atas);
  while (lim_atas == 1) {  // HIGH = not pressed
    setMotor(180, 0);
    delay(50);
    lim_atas = digitalRead(atas);
  }
  setMotor(0, 0);
}

void put() {
  lim_bawah = digitalRead(bawah);
  while (lim_bawah == 1) {
    setMotor(-170, 0);
    delay(10);
    lim_bawah = digitalRead(bawah);
  }
  setMotor(0, 0);
}
