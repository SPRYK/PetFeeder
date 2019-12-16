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

SoftwareSerial chat(D6,D7); //RX,TX
String data;

//#define D4 2   // TXD1
//#define DHTPIN D4     // what digital pin we're connected to
//#define DHTTYPE DHT11   // DHT 11

//DHT dht(DHTPIN, DHTTYPE);

WiFiClient client;
MicroGear microgear(client);

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
  int check;
  char strState[msglen];
  for (int i = 0; i < msglen; i++) 
  { 
    if ((char)msg[i] == ',') {
      check = i;  
    }
    strState[i] = (char)msg[i];
    Serial.print((char)msg[i]);
  }
  Serial.println();

  
  String currentAmount = String(strState).substring(0, check);
  String stateStr = String(strState).substring(check+1, msglen);
   
  
  if(stateStr == "ON") 
  {
    //digitalWrite(ledPin, LOW);
    //microgear.chat(TargetWeb, "ON");
    microgear.chat(TargetFreeboard, "ON,");
    Serial.println("ON");
  } 
  else if (stateStr == "OFF") 
  {
    //digitalWrite(ledPin, HIGH);
    //microgear.chat(TargetWeb, "OFF");
    microgear.chat(TargetFreeboard, "OFF,");
    Serial.println("OFF"); 
  }
  else {
    String newAmount = stateStr.substring(0,check);
    String newEmail = stateStr.substring(check+1,msglen);
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

     chat.begin(115200);

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

    Serial.println("Starting UDP");
    udp.begin(localPort);
    Serial.print("Local port: ");
    Serial.println(udp.localPort());
}

void loop() 
{
  /*
    digitalRead(3);
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
      
      //String msg = String(currentAmount)
       //               + "," + String(minAmount) "," + String(status);
      // microgear.chat(TargetWeb , msg);
      String msg = "83,60,OFF";
      microgear.chat(TargetFreeboard , msg);
      Serial.println("sent data"); 
    }
   else 
   {
    Serial.println("connection lost, reconnect...");
    microgear.connect(APPID);
   }
    delay(1500);
    */
  while(chat.available()){
    char recieved = chat.read();
    data+= recieved; 
    //Serial.print('1');
  }
  if(data != ""){
    Serial.println(data);
    //split
    char s1[10],s2[10];
    bool isSecond;
    int j;
    /*
    for(int i=0;i<data.size();i++)
    {
      if(data[i]==',')
      {
        isSecond=1;
        j=i;
        continue;
      }
      if(!isSecond) s1[i]=data[i];
      else s2[i-j-1]=data[i];
    }
    s1[j]='\0';
    s2[data.size()-j]='\0';
    */
    data = "";
  }
  //Serial.println(chat.available());
  chat.print("a");
  chat.println();
  delay(500);
}
