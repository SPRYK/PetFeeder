#include <MicroGear.h>
#include <ESP8266WiFi.h>

const char* ssid     = "Seth";
const char* password = "17299004";

#define APPID   "PetFeederEmbedded"
#define KEY     "5q0HQ67dWjZqNjB"
#define SECRET  "Y9naSXOKUkXT5QCI4UnyR1Fbo"

#define ALIAS   "NodeMCU"
#define TargetWeb "HTML_web"
#define TargetFreeboard "Freeboard"


//#define D4 2   // TXD1
//#define DHTPIN D4     // what digital pin we're connected to
//#define DHTTYPE DHT11   // DHT 11

//DHT dht(DHTPIN, DHTTYPE);

WiFiClient client;
MicroGear microgear(client);

String currentAmount = "83";
String minAmount = "60";
String status = "OFF";
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
   
  
  if(stateStr == "ON") 
  {
    //digitalWrite(ledPin, LOW);
    //microgear.chat(TargetWeb, "ON");
    status = "ON";
    microgear.chat(TargetFreeboard, "ON,");
    Serial.println("TURN ON");
  } 
  else if (stateStr == "OFF") 
  {
    //digitalWrite(ledPin, HIGH);
    //microgear.chat(TargetWeb, "OFF");
    status = "OFF";
    microgear.chat(TargetFreeboard, "OFF,");
    Serial.println("TURN OFF"); 
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
       microgear.loop();
       Serial.println("connected");

       //float Humidity = dht.readHumidity();
       //float Temp = dht.readTemperature();  // Read temperature as Celsius (the default)
       //String data = "/" + String(Humidity) + "/" + String(Temp);
       //char msg[128];
       //data.toCharArray(msg,data.length());
      // Serial.println(msg);    

      //TODO get input to sent by this format
 
      String data = String(currentAmount) + "," + String(minAmount) + "," + String(status);
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
