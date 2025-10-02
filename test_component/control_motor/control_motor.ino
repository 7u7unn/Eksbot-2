#define ENA 18
#define ENB 17
#define IN1 23
#define IN2 4
#define IN3 13
#define IN4 19

const int start_pwm = 95;

void setup() {
init_motor();

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

  for(int i =0; i<=255; i++){
    setMotor(i,i);
    Serial.println(i);
    delay(40);
  }
  for(int i =255; i>=0; i--){
    setMotor(-i,-i);
    Serial.println(-i);
    delay(40);
  }

  delay(2000);
}


void init_motor(){
pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}
