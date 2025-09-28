#define encA1 2//interrupt
#define encB1 3

#define encA2 2//interrupt
#define encB2 3
int val_R = 0;

int statusA, statusB = 0;

void IRAM_ATTR ISR_R();

void setup() {
  // put your setup code here, to run once:
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  attachInterrupt(encA,ISR_R,RISING);
  Serial.begin(19200);
  
}


void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(val_R);

}

void IRAM_ATTR ISR_R(){
  if(digitalRead(encB)==LOW) val_R+=1;
  else val_R-=1;
}
