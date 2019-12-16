//new stuff

motor = microgear.init(8X2HnCSz4nngZjA,j4on6kHArCOoU5uRA63uGRj78);
timer = microgear.init(PQyComEVbDECyzx,ucMISXHjcScJyoL05P4vh8BdK);
ultrasonicSensor = microgear.init(ltqfFwALCa727US,YyLnVvBajTB0DtGoh6SDRDgJo);

microgear.connect(PetFeederEmbedded)

microgear.on(event,callback)




//basic stuff
int hour;
int minute;

long breakfast = 6;
long lunch = 12;
long dinner = 18;
bool b_fed;
bool l_fed;
bool d_fed;

int ENA = 1;
int IN1 = 8;
int IN2 = 9;

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

char ssid[] = "*************";  //  your network SSID (name)
char pass[] = "********";       // your network password


unsigned int localPort = 2390;      // local port to listen for UDP packets

/* Don't hardwire the IP address or we won't get the benefits of the pool.
 *  Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
//IPAddress timeServerIP; // time.nist.gov NTP server address
IPAddress timeServerIP(124, 109, 2 ,169); // time.nist.gov NTP server
const char* ntpServerName = "3.th.pool.ntp.org";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP

WiFiUDP udp;

//motion
int motionPin = 3;

//ultrasonic
#include <Ultrasonic.h>

#define TRIG_PIN 4
#define ECHO_PIN 5

Ultrasonic hcsr04(TRIG_PIN, ECHO_PIN);

int distance;
int feed_to;
int min_food_amount = 0;
int amount_feed = 3; //


//DHT11
#include <dht.h>

dht DHT;

#define DHT11_PIN 6

int startupTemp;
int startupHumid;

// Gmail
#include "Gsender.h"
#pragma region Globals
#pragma endregion Globals

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  // We start by connecting to a WiFi network
  connectWifi();

  pinMode(motionPin, INPUT);
  /*pinMode(buzzerPin , OUTPUT);
  pinMode(ledPin, OUTPUT);*/

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());

  int chk = DHT.read11(DHT11_PIN);
  startupTemp = DHT.temperature;
  startupHumid = DHT.humidity;
  delay(1000);
}

void loop()
{
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP); 

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  
  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no packet yet");
  }
  else {
    Serial.print("packet received, length=");
    Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    // now convert NTP time into everyday time:
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;


    // print the hour, minute and second:
    hour = (epoch  % 86400L) / 3600;
    minute = (epoch  % 3600) / 60;
  }
  // wait ten seconds before asking for the time again
  delay(10000);
  
  if (hour == breakfast && b_fed != true) { // breakfast
    feed();
    b_fed = true;
  } else if (hour == lunch && l_fed != true) { // lunch
    feed();
    l_fed = true;
  } else if (hour == dinner && d_fed != true) { // dinner
    feed();
    d_fed = true;
  } else if (hour == 0){ // New day
    b_fed = false;
    l_fed = false;
    d_fed = false;
  }

  //refill alarm
  distance = hcsr04.Ranging(CM);
  if (distance > min_food_amount) {
    //blinking();
    sendMail();
  }
  
  int chk = DHT.read11(DHT11_PIN);
  if (DHT.temperature > startupTemp + 9 || DHT.temperature < startupTemp - 9 || DHT.humidity > startupHumid + 15 || DHT.humidity > startupHumid - 15) {
    //blinking();
    Gsender *gsender = Gsender::Instance();    // Getting pointer to class instance
    String subject = "irregular temp or humid";
    if(gsender->Subject(subject)->Send("getdummy001@gmail.com", "plz check")) {
        Serial.println("Message send.");
    } else {
        Serial.print("Error sending message: ");
        Serial.println(gsender->getError());
    }
  }
  
}

unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  udp.beginPacket(address, 123);
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void connectWifi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void feed() {
  while (digitalRead(motionPin == LOW)){
    //digitalWrite(buzzerPin, HIGH);
  }
  //digitalWrite(buzzerPin, LOW);
  distance = hcsr04.Ranging(CM);
  feed_to = distance + amount_feed;
  while (distance <= feed_to) {
    digitalWrite(ENA, HIGH);  // set speed to 200 out of possible range 0~255
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    distance = hcsr04.Ranging(CM);
  }
  digitalWrite(ENA, HIGH);  // set speed to 200 out of possible range 0~255
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void sendMail() {
  Gsender *gsender = Gsender::Instance();    // Getting pointer to class instance
  String subject = "SureFeed needs refill.";
  if(gsender->Subject(subject)->Send("getdummy001@gmail.com", "Need refill")) {
      Serial.println("Message send.");
  } else {
      Serial.print("Error sending message: ");
      Serial.println(gsender->getError());
  }
}

/*void blinking() {
  digitalWrite(ledPin, HIGH);
  delay(2000);
  digitalWrite(ledPin, LOW);
}*/
