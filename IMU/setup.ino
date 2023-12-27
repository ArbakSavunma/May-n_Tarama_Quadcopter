#include<Servo.h>
Servo ESC1, ESC2, ESC3, ESC4;
Servo servo1, servo2, servo3, servo4;
Servo dif;
Servo ontek1;
Servo ontek2;
Servo arkatek;
int receiver_pins[] = {A0, A1, A2, A3, A4, A5};
long int receiver_values[] = {0, 0, 0, 0, 0, 0};
int res_min = 891;
int res_max = 1977;
int servo_val1= 0;
int servo_val2= 90;
long int throttle=0, prevyaw=0, prevpitch=0, prevroll=0, en1=0, en2=0; 

void setup(){
  initializeIMU();
  Serial.begin(115200);
  servo1.attach(2,1000,2000);
  servo2.attach(3,1000,2000);
  servo3.attach(4,1000,2000);
  servo4.attach(5,1000,2000);
  ESC1.attach(6,1000,2000);
  ESC2.attach(7,1000,2000);
  ESC3.attach(8,1000,2000);
  ESC4.attach(9,1000,2000);
  dif.attach(10,1000,2000);
  ontek1.attach(11,1000,2000);
  ontek2.attach(12,1000,2000);
  arkatek.attach(13,1000,2000);
  Serial.println("esc kalibre basladı...");
  ESC1.write(180);
  ESC2.write(180);
  ESC3.write(180);
  ESC4.write(180);
  delay(2000);
  ESC1.write(0);
  ESC2.write(0);
  ESC3.write(0);
  ESC4.write(0);
  Serial.println("esc kalibre ortada1");
  delay(2000);
  
  Serial.println("esc kalibre bitti");

}

void loop() {
  struct Orientation o = getIMUOrientation();
  for(int i=0; i<6; i++) {
    receiver_values[i]=pulseIn (receiver_pins[i], HIGH, 25000);
    Serial.print("CH");
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(receiver_values[i]);
    Serial.print(",\t");
  }
  Serial.println(" ");
  
  if( receiver_values[5]>1500) {
    
      servo1.write(servo_val1);
      servo2.write(servo_val1);
      servo3.write(servo_val1);
      servo4.write(servo_val1);
    for(int i=0; i<6; i++) {
      receiver_values[i]=map(receiver_values[i],970,1881,0,180);
      
    }
    
    if(receiver_values[2]<0) receiver_values[2]=0;
    if(receiver_values[2]>170) receiver_values[2]=180;
    throttle=receiver_values[2];
    prevyaw=map(receiver_values[0],-30,150,0,180);
    prevpitch=map(receiver_values[1],-3,196,0,180);
    prevroll=map(receiver_values[3],-21,179,0,180);
    for(int i=0; i<6; i++) {
      Serial.print("CH");
      Serial.print(i);
      Serial.print(" : ");
      Serial.print(receiver_values[i]);
      Serial.print(",\t");
    }
    Serial.println(" ");
    Serial.print(prevyaw);
    Serial.print(" , ");
    Serial.print(prevpitch);
    Serial.print(" , ");
    Serial.print(prevroll);
    Serial.println(" ");
//*************************************************************
    ESC1.write(receiver_values[2]);
    ESC2.write(receiver_values[2]);
    ESC3.write(receiver_values[2]);
    ESC4.write(receiver_values[2]);

  }
  
  Serial.print("Yaw:");
  Serial.print(o.Yaw);
  Serial.print(",");

  Serial.print("Pitch:");
  Serial.print(o.Pitch);
  Serial.print(",");

  Serial.print("Roll:");
  Serial.print(o.Roll);
  Serial.println();

}
