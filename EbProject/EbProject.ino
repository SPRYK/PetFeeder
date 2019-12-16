#include <dummy.h>

#include <MicroGear.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>

#include <WiFiUdp.h>
unsigned int localPort = 2390; 
WiFiUDP udp;

// Gmail
#include "Gsender.h"
#pragma region Globals
#pragma endregion Globals

const char* ssid     = "bump";
const char* password = "11111111";

#define APPID   "PetFeederEmbedded"
#define KEY     "5q0HQ67dWjZqNjB"
#define SECRET  "Y9naSXOKUkXT5QCI4UnyR1Fbo"

#define ALIAS   "NodeMCU"
#define TargetWeb "HTML_web"
#define TargetFreeboard "Freeboard"

#define D6 12 
#define D7 13

SoftwareSerial chat(D6,D7); //RX,TX
String data;

//#define D4 2   // TXD1
//#define DHTPIN D4     // what digital pin we're connected to
//#define DHTTYPE DHT11   // DHT 11

//DHT dht(DHTPIN, DHTTYPE);

WiFiClient client;
MicroGear microgear(client);

String currentAmount = "83";
String minAmount = "60";
String status = "OFF";

void sendMail() {
  Gsender *gsender = Gsender::Instance();    // Getting pointer to class instance
  String subject = "PetFeeder needs refill.";
  if(gsender->Subject(subject)->Send("getdummy001@gmail.com", "Need refill")) {
      Serial.println("Message send.");
  } else {
      Serial.print("Error sending message: ");
      Serial.println(gsender->getError());
  }
}

void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) 
{
  Serial.print("Incoming message --> ");
  Serial.print(topic);
  Serial.print(" : ");
  
  char strState[msglen];
  for (int i = 0; i < msglen; i++) 
  { 
    strState[i] = (char)msg[i];
    Serial.print((char)msg[i]);
  }
  Serial.println();

  
  String stateStr = String(strState).substring(0, msglen);
   
  //-----------------receive from server and send to STM32-----------------
  if(stateStr == "ON") 
  {
    /*
    //digitalWrite(ledPin, LOW);
    //microgear.chat(TargetWeb, "ON");
    status = "ON";
    microgear.chat(TargetFreeboard, "ON,");
    Serial.println("TURN ON");
    */
    chat.print("1");
    chat.println();
  } 
  else if (stateStr == "OFF") 
  {
    /*
    //digitalWrite(ledPin, HIGH);
    //microgear.chat(TargetWeb, "OFF");
    status = "OFF";
    microgear.chat(TargetFreeboard, "OFF,");
    Serial.println("TURN OFF"); 
    */
    chat.print("0");
    chat.println();
  }
  else if (stateStr.substring(0,1) == "M") {
      minAmount = stateStr.substring(1,msglen);
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
    microgear.subscribe("/gearname/NodeMCU/data/$/command");
}

void loop() 
{
    //digitalRead(3);
    if (microgear.connected())
    {
       //------------receive from stm32 and send to SERVER---------------------
       microgear.loop();
       Serial.println("connected");

       //float Humidity = dht.readHumidity();
       //float Temp = dht.readTemperature();  // Read temperature as Celsius (the default)
       //String data = "/" + String(Humidity) + "/" + String(Temp);
       //char msg[128];
       //data.toCharArray(msg,data.length());
      // Serial.println(msg);    

      //TODO get input to sent by this format
      while(chat.available()){
      char recieved = chat.read();
      data+= recieved; 
      }
      if(data != ""){
        Serial.println(data);
        data=""; // data be like "ON,50" or "OFF,ERROR"
      }
 
      //String data = String(currentAmount) + "," + String(minAmount) + "," + String(status);
      //char msg[128];
      //data.toCharArray(msg,data.length());
      // microgear.chat(TargetWeb , msg);
      
      microgear.chat(TargetFreeboard , data);
      microgear.publish("/gearname/NodeMCU/minAmount", String(minAmount));
      Serial.println(data); 
      
      
    }
   else 
   {
    Serial.println("connection lost, reconnect...");
    microgear.connect(APPID);
   }
    delay(1500);
}