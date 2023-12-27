float analogIn[]={A0,A1,A2,A3,A6};
float analogVal[]={0,0,0,0,0};
int leds[]= {2,3,4,5};
int state=0;
float sagTemp=0, solTemp=0, onTemp=0, arkaTemp=0, ustTemp=0, bataryCharge=0;

void thermalDetect() {
  for(int i=0; i<5;i++) {
    analogVal[i]= analogRead(analogIn[i]);
    analogVal[i]=analogVal[i] * 4.88 / 10;
  }
  sagTemp=analogVal[0]; onTemp=analogVal[1]; solTemp=analogVal[2]; arkaTemp=analogVal[3]; ustTemp=analogVal[4];
}

void cargelevel() {
  if(bataryCharge<896.148) state=1;
  else if(bataryCharge>=896.148 && bataryCharge<927.861) state=2;  
  else if(bataryCharge>=927.861 && bataryCharge<965.574) state=3;
  else if(bataryCharge>=965.574 && bataryCharge<1003) state=4;
  else state=5;
  switch(state) {
    case 1:
      digitalWrite(2,LOW);
      digitalWrite(3,LOW);
      digitalWrite(4,LOW);
      digitalWrite(5,LOW);
      break;
    case 2:
      digitalWrite(2,HIGH);
      digitalWrite(3,LOW);
      digitalWrite(4,LOW);
      digitalWrite(5,LOW);
      break;
    case 3:
      digitalWrite(2,HIGH);
      digitalWrite(3,HIGH);
      digitalWrite(4,LOW);
      digitalWrite(5,LOW);
      break;
    case 4:
      digitalWrite(2,HIGH);
      digitalWrite(3,HIGH);
      digitalWrite(4,HIGH);
      digitalWrite(5,LOW);
      break;
    case 5:
      digitalWrite(2,HIGH);
      digitalWrite(3,HIGH);
      digitalWrite(4,HIGH);
      digitalWrite(5,HIGH);
      break;
    
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(8,OUTPUT);
  pinMode(7,OUTPUT);
  for(int i=0;i<4;i++){
    pinMode(leds[i],OUTPUT);
  }
  digitalWrite(8,LOW);
  digitalWrite(7,LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
  bataryCharge= analogRead(A7);
  cargelevel();
  if(bataryCharge<910.17)
    digitalWrite(7,HIGH);
  else 
    digitalWrite(7,LOW);
  thermalDetect();
  if(sagTemp>60 || solTemp>60 || onTemp>60 || arkaTemp>60) {
    if(ustTemp>60) {
        digitalWrite(8,HIGH);
    }
    
  }
  else 
    digitalWrite(8,LOW);

  
}
