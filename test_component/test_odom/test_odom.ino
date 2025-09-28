#define encA1 35  //interrupt
#define encB1 36
#define encA2 14  //interrupt
#define encB2 16

volatile long val_R = 0;
volatile long val_L = 0;
long val_R_prev = 0;
long val_L_prev = 0;

//robot parameters
float ppr = 11.0;
float gearbox_R = 0.0;
float gearbox_L = 0.0;
float base_width = 0.0;
float wheel_k = (2*PI*base_width); //cm

//position val_R
float x = 0.0; //cm
float y = 0.0; //cm
float theta = 0.0; //rad


// ratio tics_to_cm (ttc) 
//cm = tics * ttc

float ttc_R = wheel_k/(gearbox_R*ppr);
float ttc_L = wheel_k/(gearbox_L*ppr);

void IRAM_ATTR Read_R();
void IRAM_ATTR Read_L();
void update_odom();

void setup(){
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  attachInterrupt(encA1, Read_R, RISING);

  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(encA2, Read_L, RISING);
  Serial.begin(115200);

}
void loop(){

}


void IRAM_ATTR Read_R() {
  if (digitalRead(encB1) == LOW) val_R += 1;
  else val_R -= 1;
}
void IRAM_ATTR Read_L() {
  if (digitalRead(encB2) == LOW) val_L += 1;
  else val_L -= 1;
}

void update_odom(){
    long d_right_val = val_R-val_R_prev;
    long d_left_val = val_L-val_L_prev;

    val_R_prev = val_R;
    val_L_prev = val_L;

    // Convert ticks (val) to dist
    float dRight = d_right_val * ttc_R;
    float dLeft = d_left_val * ttc_L;

    // Linear and angular displacement
    float dCenter = (dLeft + dRight) / 2.0;
    float dTheta = (dRight - dLeft) / base_width;

    // Update position
    theta += dTheta;
    if (theta > PI) theta -= 2 * PI;
    if (theta < -PI) theta += 2 * PI;
    x += dCenter * cos(theta);
    y += dCenter * sin(theta);


}







