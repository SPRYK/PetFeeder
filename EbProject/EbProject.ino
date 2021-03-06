#include <dummy.h>

#include <MicroGear.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <TridentTD_LineNotify.h>
#include <WiFiUdp.h>
unsigned int localPort = 2390; 
WiFiUDP udp;

// Gmail
String email = "team9762@gmail.com"; // exist but don't know password need to change
#include "Gsender.h"
#pragma region Globals
#pragma endregion Globals

// Wifi name and password
const char* ssid     = "AndroidAP";
const char* password = "jwbs8766";

#define APPID   "PetFeederEmbedded"
#define KEY     "5q0HQ67dWjZqNjB"
#define SECRET  "Y9naSXOKUkXT5QCI4UnyR1Fbo"

#define ALIAS   "NodeMCU"
#define TargetFreeboard "HTML_web"


#define D6 12 
#define D7 13

SoftwareSerial chat(D6,D7); //RX,TX
String data;
String minAmount = "40"; // can be change
String currentAmount = "20";
String status = "ON";
String out;
String lineToken = "xdDmFDNwkKGXamqzRoaMuxAHbMSITgfaDLKdLUy5Xcg";
int state = 0;
WiFiClient client;
MicroGear microgear(client);

void sendMail() {
  Gsender *gsender = Gsender::Instance();    // Getting pointer to class instance
  String subject = "PetFeeder needs refill.";
  if(gsender->Subject(subject)->Send(email, "Need refill")) {
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

  int count = 0;
  String stateStr = String(strState).substring(0, msglen);
   
  //-----------------receive from server and send to STM32-----------------
  if(stateStr == "ON") 
  {
    status = "ON";
    Serial.println("TURN ON");
    Serial.println(status);
    count++;
    chat.print("1");
    chat.println();
    delay(1000);
  } 
  else if (stateStr == "OFF") 
  {
    status = "OFF";
    Serial.println("TURN OFF");
    count++;
    Serial.println(status); 
    chat.print("0");
    chat.println();
    delay(1000);
  }
  else if (stateStr.substring(0,1) == "M") {
      minAmount = stateStr.substring(2);
      Serial.println("Set minimum amount to " + minAmount);
      count++; 
      delay(1000);
    }
  Serial.println(count);   
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

    Serial.begin(115200);
    chat.begin(115200);
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
    LINE.setToken(lineToken);
    if (microgear.connected())
    {
       //------------receive from stm32 and send to SERVER---------------------
       Serial.println("connected");
       microgear.loop();
      //TODO get input to sent by this format
      while(chat.available()){
      char recieved = chat.read();
      data+= recieved; 
      }
      if(data != ""){ // data be like "ON,50" or "OFF,ERROR"
        Serial.println(data);
        
        // split string may be wrong
        for (int i=0; i<data.length(); i++) {
          if (data.substring(i,i+1) == ",") {
            status = data.substring(0,i);
            currentAmount = data.substring(i+1);
            if (currentAmount == "ERROR") {
              currentAmount = "Out of range";
              }
            break;
            }
          }
        
        data="";  
      }
 
      out = currentAmount + "," + minAmount + "," + status;

      // send email
      if (currentAmount.toInt() < minAmount.toInt() && state == 0) {
        LINE.notify("Current Amount is " + (String)currentAmount);
        state = 1; 
        }
      if (currentAmount.toInt() > minAmount.toInt() && state == 1) {
        state = 0; 
      }
      microgear.chat(TargetFreeboard , out);
      microgear.publish("/gearname/NodeMCU/minAmount", String(minAmount));
      Serial.println(out); 
      
      
    }
   else 
   {
    Serial.println("connection lost, reconnect...");
    microgear.connect(APPID);
   }
    delay(1500);
}
