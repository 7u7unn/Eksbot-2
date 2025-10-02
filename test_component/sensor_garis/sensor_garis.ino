/* 
Modified on Nov 28, 2020
Modified by MehranMaleki from Arduino Examples
Home
*/


//Mux control pins
int s0 = 25;
int s1 = 33;
int s2 = 32;
int s3 = 26; //26

//Mux in "SIG" pin
int SIG_pin = 34; //


void setup() {
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  Serial.begin(115200);
}


void loop() {

  //Loop through and read all 16 values
  for (int i = 0; i < 12; i++) {
    if (i < 6) {

      Serial.print("s1 ");
      Serial.print(i+1);
    } else {
      Serial.print("s2 ");
      Serial.print(i - 5);
    }
    Serial.print(": ");
    Serial.print(readMux(i));
    Serial.print(" ");


    delay(15);

    // delay(20);
  }
  Serial.println();
  // delay(10);
}


float readMux(int channel) {
  int controlPin[] = { s0, s1, s2, s3 };

  int muxChannel[16][4] = {
    { 0, 0, 0, 0 },  //channel 0
    { 1, 0, 0, 0 },  //channel 1
    { 0, 1, 0, 0 },  //channel 2
    { 1, 1, 0, 0 },  //channel 3
    { 0, 0, 1, 0 },  //channel 4
    { 1, 0, 1, 0 },  //channel 5
    { 0, 1, 1, 0 },  //channel 6
    { 1, 1, 1, 0 },  //channel 7
    { 0, 0, 0, 1 },  //channel 8
    { 1, 0, 0, 1 },  //channel 9
    { 0, 1, 0, 1 },  //channel 10
    { 1, 1, 0, 1 },  //channel 11
    { 0, 0, 1, 1 },  //channel 12
    { 1, 0, 1, 1 },  //channel 13
    { 0, 1, 1, 1 },  //channel 14
    { 1, 1, 1, 1 }   //channel 15
  };

  //loop through the 4 sig
  for (int i = 0; i < 4; i++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //read the value at the SIG pin
  int val = analogRead(SIG_pin);

  //return the value
  // float voltage = (val * 3.3) / 4096.0;
  return val;
}
