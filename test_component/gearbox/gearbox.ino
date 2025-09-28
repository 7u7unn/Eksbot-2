#define encA1 35  //interrupt
#define encB1 36

#define encA2 14  //interrupt
#define encB2 16
int val_R = 0;
int val_L = 0;



float ppr = 11;
float gb = 0;


// int statusA, statusB = 0;

void IRAM_ATTR Read_R();
void IRAM_ATTR Read_L();
void Calc_gearbox(int RL);

bool start = false;
int opt;

void setup() {
  // put your setup code here, to run once:
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  attachInterrupt(encA1, Read_R, RISING);

  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(encA2, Read_L, RISING);
  Serial.begin(115200);
}


void loop() {
  // put your main code here, to run repeatedly:
  if (start == 0) {
    Serial.println("Calculate Gearbox: 1=Right, 0=Left, 2=Both");
    while (1) {
      if (Serial.available()>0) {

        int opt = Serial.parseInt();
        break;
      }
    }
    for (int i = 0; i < 3; i++) {
      Serial.print("Start to measure in ");
      Serial.println(3 - i);
      delay(1000);
    }
    val_R = 0;
    val_L = 0;
    start = true;
  }
  if (start == true) {
    if (opt == 1) {
      Serial.print("Right: ");
      Serial.println(val_R);
    } else if (opt == 0) {
      Serial.print("Left: ");
      Serial.println(val_L);
    } else if (opt == 2) {
      Serial.print("Right: ");
      Serial.print(val_R);
      Serial.print(" | Left: ");
      Serial.println(val_L);
    }

    if (Serial.available()>0) {
      String input = Serial.readString();
      input.trim();
      if (input.equals("ok")) {
        // Calculate gearbox
        if (opt == 2) {
          Serial.print("Right gearbox ratio: ");
          Serial.println(Calc_gearbox(1));
          Serial.print("Left gearbox ratio: ");
          Serial.println(Calc_gearbox(0));
        } else {
          float result = Calc_gearbox(opt);
          Serial.print("Gearbox ratio: ");
          Serial.println(result);
        }
        start = false;  // Reset for next measurement
      }
    }
    delay(100);
  }
}

void IRAM_ATTR Read_R() {
  if (digitalRead(encB1) == LOW) val_R += 1;
  else val_R -= 1;
}
void IRAM_ATTR Read_L() {
  if (digitalRead(encB2) == LOW) val_L += 1;
  else val_L -= 1;
}

float Calc_gearbox(int RL) {
  if (RL == 1) {
    gb = val_R / ppr;
    return gb;
  } else if (RL == 0) {
    gb = val_L / ppr;
    return gb;
  } else {
    return -1;
  }
}
