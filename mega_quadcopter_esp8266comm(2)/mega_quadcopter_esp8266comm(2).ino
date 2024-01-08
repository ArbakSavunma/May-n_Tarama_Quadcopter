/*ibrahim mut
 *  arduino json parser example
 *  for mega 2560
 *  
 *  1. wait for text message from nodeMCU (listens Serial wire)
 *  2. parse incoming text as JSON object
 *  3. perform desired operations by these values
 *  
 */

#include <ArduinoJson.h>

//***EXPECTED DATA FROM MQTT SERVER************
#define MODE "MODE"
#define THROTTLE "THROTTLE"
#define ROLL "ROLL"
#define PITCH "PITCH"
#define YAW "YAW"
 
void setup() {
  // Initialize serial port
  Serial.begin(9600);
  while (!Serial)
    continue;

  
  //nodeMCu üzerinden okunan veri için bekleme süresini
  //arttır(varsayılan=1000)
  Serial.setTimeout(2000);
}

/*
 * Parse given text as JSON and return that
 * JSON object
 * Thus data can be accessed through mapping
 */
JsonDocument parseToJsonDoc(String json){
  // Allocate the JSON document
  JsonDocument doc;
  
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, json);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
  }

  return doc;  
}

void loop() {
  // not used in this example

  if(Serial.available()){

    //wait 2seconds for a string which have escape char '\n'
    //NOTE: if '\n' could not find in 2 seconds, parser will 
    //give error since text will not be complete
    String json=Serial.readStringUntil("\n");

    Serial.println("ayrıştırılacak: "+json);
    JsonDocument doc=parseToJsonDoc(json);

    //write parsed json data to Serial monitor
    int mode = doc[MODE];
    int throttle = doc[THROTTLE];
    int roll = doc[ROLL];
    int pitch = doc[PITCH];
    int yaw= doc[YAW];
  
    Serial.println("mega2560 seri protokolden alınan metni json yaptı");
  
    Serial.print(MODE":");
    Serial.println(mode);
    
    Serial.println(THROTTLE":"+String(throttle));
    
    Serial.print(ROLL":");
    Serial.println(roll);
  
    Serial.print(PITCH":");
    Serial.println(pitch);
  
    Serial.print(YAW":");
    Serial.println(yaw);
  
  }
}
​

