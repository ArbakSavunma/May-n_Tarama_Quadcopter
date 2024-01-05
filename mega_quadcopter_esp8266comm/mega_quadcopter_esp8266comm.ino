/*ibrahim mut
 * simple emulation for robotic device
 * 
 * this device(2560) will be emulate any part some robot.
 * 
 *  The inner work of nodeMCU was like below
 *  1. 8266 will wait for a 16 bit int message from hivemq
 *  2. 8266 will parse the message and determine the command
 *  3. 8266 will transmitt received data to this mega2560
 *  3. thus 2560 will blink led as much as that received int size
 *  
 *  expected message from serial will be 16 bit to blink leds
 *  and will be parsed as|     4     +     8       +     4             bits respectively
 *  which assumed as     |led part   + blink count + total blink time(seconds)
 */
 #include <SoftwareSerial.h>

//*** nodeMCU pin connections for serial commnctn(to be able to send message)************************
/*#define RX 5//mavi
#define TX 6//yeşil
SoftwareSerial espSerial(RX, TX);
*/
//***emulating robot parts**************************************
#define LED_GREEN 11  //pin11
#define LED_RED 10  //pin10
//***emulating robot parts**************************************

int DEGER=0;//to store serial data comes through serial line
/*
 * blinks the 'led' 'n' times in 'dt' time
 */
void blinkInfo(int led, int n, int dt){

  int i;
  for(i=0; i<n; i++){
    digitalWrite(led, HIGH);
    delay(dt/(2*n));
    digitalWrite(led, LOW);
    delay(dt/(2*n));
  }
}

void setup() {
  // put your setup code here, to run once:
  
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  blinkInfo(LED_GREEN, 2, 1000);
  blinkInfo(LED_RED, 2, 1000);

  Serial.begin(9600);//esp will send by this baud rate
  
  delay(300);

}

void loop() {
  // put your main code here, to run repeatedly:

  //wait for a 16 bit(int16_t) message from nodeMCU
  if(Serial.available()>=2){//wait for at least 2 bytes
    //parse incoming message
    DEGER=DEGER | Serial.read()<<8;
    DEGER=DEGER | Serial.read(); 

    Serial.print("received:");
    Serial.println(DEGER, HEX);

    //assume first 4 bits as
    //indicator to which part of robot to execute
    //we can advertise 2^4 parts for this emulation code
    int ROBO_PART=0;
    if ( ((DEGER>>12)&0xff) == 0 ){
      ROBO_PART=LED_GREEN;

    }
    else{
      ROBO_PART=LED_RED;
    }


    //assume remaining 12 bits as
    //command to execute of that robo part
    DEGER=DEGER & 0x0fff;//bitwise last 12 bits
    
    int DELAY=DEGER &0x0f;//last 4 bits as blink time
    DEGER=DEGER >>4;//first 8 bits as blink count
    Serial.print("[INFO]: pin "+String(ROBO_PART)+ " will be signalled "+ String(DEGER) + " times for "+String(DELAY)+" seconds" );
    blinkInfo(ROBO_PART, DEGER, DELAY*1000);

    DEGER=0;
  }

}