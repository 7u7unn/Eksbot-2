#define encA 2//interrupt
#define encB 3
int val_R = 0;

int statusA, statusB = 0;

void ISR_R();

void setup() {
  // put your setup code here, to run once:
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  attachInterrupt(encA,ISR_R,RISING);
  Serial.begin(19200);
  
}
///1.77 untuk motor freak 1/19
// 1/11 untuk motor oke

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(val_R);

}

void ISR_R(){
  if(digitalRead(encB)==LOW) val_R+=1;
  else val_R-=1;
}
