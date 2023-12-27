#include <SoftwareSerial.h>    
SoftwareSerial esp(31, 30);   //Pin 6 and 7 act as RX and TX. Connect them to TX and RX of ESP8266   
   
#define DEBUG true
int number;
String mySSID = "subocugu";  // Wi-Fi SSID
String myPWD = "deu12345"; // Wi-Fi Password
String myAPI = "P0A9VSQFXBILDJTQ";   // WRITE API Key
String myHOST = "api.thingspeak.com";
String myPORT = "80";
String myFIELD = "field1"; 

void setup()
{
  Serial.begin(115200);
  esp.begin(115200);
  Send_AT_Cmd("AT+RST", 1000, DEBUG,0);                      
  Send_AT_Cmd("AT+CWMODE=1", 1000, DEBUG,0);                 
  Send_AT_Cmd("AT+CWJAP=\""+ mySSID +"\",\""+ myPWD +"\"", 1000, DEBUG,0);   
  delay(1000); 
}

  void loop()
  {
    number = random(100); // Send a random number between 1 and 100
    String sendData = "GET /update?api_key="+ myAPI +"&"+ myFIELD +"="+String(number);
    Send_AT_Cmd("AT+CIPMUX=1", 500, DEBUG,1);       //Allow multiple connections
    Send_AT_Cmd("AT+CIPSTART=0,\"TCP\",\""+ myHOST +"\","+ myPORT, 500, DEBUG,2);
    Send_AT_Cmd("AT+CIPSEND=0," +String(sendData.length()+4),500,DEBUG,3);  
    esp.find(">"); 
    esp.println(sendData);
    Serial.print("Value to be sent: ");
    Serial.println(number);
    Send_AT_Cmd("AT+CIPCLOSE=0",500,DEBUG,4);
    Serial.println(Send_AT_Cmd("AT+CIPCLOSE=0",500,DEBUG,5));
    Serial.println("Done!");
    Serial.println("");

  }

  String Send_AT_Cmd(String command, const int timeout, boolean debug, int flag)
{
  Serial.print(command);
  Serial.println("     ");
  Serial.println(flag);
  String response = "";
  esp.println(command);
  if(esp.find("OK"))
    Serial.println("buraya girdi da");
  long int time = millis();
  while ( (time + timeout) > millis())
  {
    
    while (esp.available())
    {
      char c = esp.read();
     
      response += c;
       Serial.println(int(c));
    }
  }
  if (debug)
  {
    Serial.print(response);
  }
  return response;
}