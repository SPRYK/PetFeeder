#include <MicroGear.h>
#include <ESP8266WiFi.h>
#include "DHT.h"

const char* ssid     = "Your ssid";
const char* password = "Your WiFi password";

#define APPID   "PetFeederEmbedded"
#define KEY     "5q0HQ67dWjZqNjB"
#define SECRET  "Y9naSXOKUkXT5QCI4UnyR1Fbo"

#define ALIAS   "NodeMCU1"
#define TargetWeb "HTML_web"

//#define D4 2   // TXD1
//#define DHTPIN D4     // what digital pin we're connected to
//#define DHTTYPE DHT11   // DHT 11

//DHT dht(DHTPIN, DHTTYPE);

WiFiClient client;
MicroGear microgear(client);

void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) 
{
  Serial.print("Incoming message --> ");
  Serial.print(topic);
  Serial.print(" : ");
  int check;
  char strState[msglen];
  for (int i = 0; i < msglen; i++) 
  { 
    if ((char)msg[i] == '/') {
      check = i;  
    }
    strState[i] = (char)msg[i];
    Serial.print((char)msg[i]);
  }
  Serial.println();
  
  String stateStr = String(strState).substring(0, msglen); 
  
  if(stateStr == "ON") 
  {
    digitalWrite(ledPin, LOW);
    microgear.chat(TargetWeb, "ON");
  } 
  else if (stateStr == "OFF") 
  {
    digitalWrite(ledPin, HIGH);
    microgear.chat(TargetWeb, "OFF");
  }
  else {
    String newAmount = stateStr.substring(0,check);
    String newEmail = stateStr.substring(check+1,msglen)
    // TODO Set new parameter
  
  }
}


void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) 
{
    Serial.println("Connected to NETPIE...");
    microgear.setAlias(ALIAS);
}

void setup() 
{
     /* Event listener */
    microgear.on(MESSAGE,onMsghandler);
    microgear.on(CONNECTED,onConnected);

    //dht.begin();
    Serial.begin(115200);
    Serial.println("Starting...");

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
       delay(250);
       Serial.print(".");
    }

    Serial.println("WiFi connected");  
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    microgear.init(KEY,SECRET,ALIAS);
    microgear.connect(APPID);
}

void loop() 
{
    if (microgear.connected())
    {
       microgear.loop();
       Serial.println("connected");

       //float Humidity = dht.readHumidity();
       //float Temp = dht.readTemperature();  // Read temperature as Celsius (the default)
       //String data = "/" + String(Humidity) + "/" + String(Temp);
       //char msg[128];
       //data.toCharArray(msg,data.length());
      // Serial.println(msg);    

      //TODO get input to sent by this format
      
      String data = "/" +String(currentUltra)
                      + "/" + String(minUltra) "/" + String(status);
       microgear.chat(TargetWeb , msg);
    }
   else 
   {
    Serial.println("connection lost, reconnect...");
    microgear.connect(APPID);
   }
    delay(1500);
}