/*
  ArduinoMqttClient - WiFi Advanced Callback

  This example connects to a MQTT broker and subscribes to a single topic,
  it also publishes a message to another topic every 10 seconds.
  When a message is received it prints the message to the serial monitor,
  it uses the callback functionality of the library.

  It also demonstrates how to set the will message, get/set QoS,
  duplicate and retain values of messages.

  The circuit:
  - Arduino MKR 1000, MKR 1010 or Uno WiFi Rev.2 board

  This example code is in the public domain.
*/

// **** use this compiler directive for ESP8266 boards
// **** otherwise comment it out

// #define ESP8266

#include <ArduinoMqttClient.h>

#ifdef ESP8266
  #include <ESP8266WiFi.h> // for NodeMCU and ESP8266 ethernet modules
#else
  #include <WiFiNINA.h> // for MKR1010 boards
  //#include <WiFi101.h>  // for MKR 1000 boards
#endif

#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

byte mac[6];  // the MAC address of your WiFi Module
IPAddress ip; // the IP address of this board
IPAddress subnetMask; // subnet mask
IPAddress gatewayIP; // IP address of the gateway
String hostname; // default hostname of this module

#ifdef ESP8266
  const char *myHostname = "NodeMCU";  // set the mqtt hostname of this device
#else
  const char *myHostname = "MKR1010";
#endif

// To connect with SSL/TLS:
// 1) Change WiFiClient to WiFiSSLClient.
// 2) Change port value from 1883 to 8883.
// 3) Change broker value to a server with a known SSL/TLS root certificate
//    flashed in the WiFi module.

#ifdef ESP8266
  // initial variables for pin IO on my NodeMCU protoboard
  const int LED = 12; // built in board LED (use LED8 pin for now)
  const int LED1 = 14; // GPIO 14
  const int LED2 = 13; // GPIO 13
  const int LED3 = 5; // GPIO 5
  const int LED4 = 15; // GPIO 15
  const int LED5 = 10; // GPIO 10
  const int LED6 = 4; // GPIO 4
  const int LED7 = 16; // GPIO 16
  const int LED8 = 12; // GPIO 12
#else
  // initial variables for pin IO on my MKR1010 protoboard
  const int LED = 6; // built in board LED
  const int LED1 = 0;
  const int LED2 = 1;
  const int LED3 = 2;
#endif

const int d1 = 2000; // first flash long on time
const int d2 = 1000; // off time
const int d3 = 500;  // second flash short on time

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] =  "192.168.1.250";   //  broker host is NR4O-pi1 - otiginal assignment ="test.mosquitto.org"
int        port     = 1883;
const char willTopic[] = "BarricAid/will";
const char inTopic[]   = "BarricAid/test";
const char outTopic[]  = "BarricAid/out";
char msg[128] = "";  //message buffer
boolean newMsg = false;

const long interval = 10000;
unsigned long previousMillis = 0;
boolean glob_flash = false;
unsigned long cMillis = 0;
unsigned long p1Millis = 0;
unsigned long p2Millis = 0;
unsigned long wifiConnectCount = 0;
unsigned long mqttConnectCount = 0;
unsigned long mqttCumUpTime = 0;
unsigned long mqttAvgUpTimeSec = 0;
unsigned long mqttStartTime = 0;
unsigned long mqttUpTime = 0;
unsigned long msgRcvCount = 0;
int ledState = HIGH; // start with LED off
int led3State = HIGH; // start with LED off
boolean statusFlag = true;  // flag to start stop sending status messages

int count = 0;

void setup() {
  // initialize digital pins as an output.

#ifdef ESP8266
  // difine 8 LEDs on NodeMCU protoboard
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED7, OUTPUT);
  pinMode(LED8, OUTPUT);
#else
  // define 3 LEDs on MKR1010 protoboard
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
#endif

#ifdef ESP8266
  digitalWrite(LED1, HIGH);    // turn all the LEDs off to start HIGH = LED OFF
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
  digitalWrite(LED4, HIGH);
  digitalWrite(LED5, HIGH);
  digitalWrite(LED6, HIGH);
  digitalWrite(LED7, HIGH);
  digitalWrite(LED8, HIGH);
#else
  digitalWrite(LED1, HIGH);    // turn all the LEDs off to start HIGH = LED OFF
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
#endif

  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // attempt to connect to WiFi network
  connectToWiFi();
/*
  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();
*/

  // You can provide a unique client ID, if not set the library uses Arduin-millis()
  // Each client must have a unique client ID

  mqttClient.setId(myHostname);

  // You can provide a username and password for authentication
  // mqttClient.setUsernamePassword("username", "password");

  // set a will message, used by the broker when the connection dies unexpectantly
  // you must know the size of the message before hand, and it must be set before connecting

  String willPayload = "oh no! - NodeMCU stopped responding";
  bool willRetain = true;
  int willQos = 1;

  mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
  mqttClient.print(willPayload);
  mqttClient.endWill();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }
  mqttConnectCount = 1;  // connection counter
  mqttStartTime = millis();
  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  // set the message receive callback
  mqttClient.onMessage(onMqttMessage);

  Serial.print("Subscribing to topic: ");
  Serial.println(inTopic);
  Serial.println();

  // subscribe to a topic
  // the second paramter set's the QoS of the subscription,
  // the the library supports subscribing at QoS 0, 1, or 2
  int subscribeQos = 1;

  mqttClient.subscribe(inTopic, subscribeQos);

  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(inTopic);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(inTopic);
  Serial.println();
}

void loop() {

  // call poll() regularly to allow the library to receive MQTT messages and
  // send MQTT keep alives which avoids being disconnected by the broker
  int mqttClientStatus = mqttClient.connected();
  if (!mqttClientStatus) {
    Serial.print("MQTT NOT CONNECTED!...");
    Serial.print("Status: ");
    Serial.print(mqttClientStatus);
    Serial.print("...Error Code: ");
    Serial.println(mqttClient.connectError());
	  mqttConnectCount += 1;  // increment the connection count
    mqttUpTime = millis() - mqttStartTime;
	  mqttCumUpTime += mqttUpTime;
    mqttAvgUpTimeSec = mqttCumUpTime / (mqttConnectCount - 1) / 1000;
    Serial.print("MQTT Connection Uptime: ");
    Serial.print(mqttUpTime/1000);
    Serial.print(" Sec");
    Serial.print("  Avg Up Time: ");
    Serial.print(mqttAvgUpTimeSec);
    Serial.println(" seconds");

    int clientStatus = wifiClient.connected();
    if (!clientStatus) {
	  wifiConnectCount += 1; // increment WiFi connection counter
	  Serial.print("wifiClient Disconnected...");
    Serial.println("Value: ");
	  Serial.print(clientStatus);
	  Serial.print("...WiFi Status: ");

  	  switch (WiFi.status()) {
  		  case WL_CONNECTION_LOST :
  			Serial.print("CONNECTION_LOST");
        Serial.println("...attempting WiFi reconnect");
        connectToWiFi();
  			break;
  		  case WL_DISCONNECTED :
  			Serial.print("DISONNECTED");
        Serial.println("...attempting WiFi reconnect");
        connectToWiFi();
  			break;
  		  case WL_CONNECTED :
  			Serial.print("CONNECTED");
  			break;
  		  case WL_NO_SHIELD :
  			Serial.print("NO_SHIELD");
  			break;
  		  case WL_NO_SSID_AVAIL :
  			Serial.print("NO_SSID_AVAIL");
  			break;
  		  case WL_CONNECT_FAILED :
  			Serial.print("CONNECT_FAILED");
        Serial.println("...attempting WiFi reconnect");
        connectToWiFi();
  			break;
  		  default :
  			// no matches to the above - must be one
  			//   WL_AP_CONNECTED, WL_AP_LISTENING, WL_IDLE_STATUS, WL_SCAN_COMPLETED
  			Serial.println("One of 4 other status returns");
  	  }
    }
    Serial.println();
    if (MQTT_connect() == true){  // attempt to reconnect to MQTT broker
      mqttStartTime = millis();  // reset the start time if connected
    }
  }
  mqttClient.poll();
  if (mqttConnectCount == 1){ // set the Avg Up Time to the current up time on first connection
    mqttAvgUpTimeSec = (millis() - mqttStartTime) / 1000;
  }

  if (newMsg == true) {
    // check message for a command
    newMsg = false;
    if (strcmp(msg,"LED 1 ON") == 0) {
        digitalWrite(LED1, LOW);    // turn the LED ON by making the voltage LOW
      }
      else if (strcmp(msg, "LED 1 OFF") == 0) {
        digitalWrite(LED1, HIGH);    // turn the LED ON by making the voltage LOW
      }
      else if (strcmp(msg, "LED 2 ON") == 0) {
        digitalWrite(LED2, LOW);    // turn the LED ON by making the voltage LOW
      }
      else if (strcmp(msg, "LED 2 OFF") == 0) {
        digitalWrite(LED2, HIGH);    // turn the LED ON by making the voltage LOW
      }
      else if (strcmp(msg, "LED 3 ON") == 0) {
		statusFlag = false;
        digitalWrite(LED3, LOW);    // turn the LED ON by making the voltage LOW
      }
      else if (strcmp(msg, "LED 3 OFF") == 0) {
		statusFlag = true;
        digitalWrite(LED3, HIGH);    // turn the LED ON by making the voltage LOW
      }
      else {
        // invalid command - flash LED 3 for 5 seconds
        Serial.println("invalid message");
        flash3("Start");
      }
  }
  // avoid having delays in loop, we'll use the strategy from BlinkWithoutDelay
  // see: File -> Examples -> 02.Digital -> BlinkWithoutDelay for more info
  unsigned long currentMillis = millis();

  flash3("");    // flash LED 3 as needed

  if ((currentMillis - previousMillis >= interval) && statusFlag) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    String payload;

    payload += "BarricAid Status: ";
    payload += " ";
    payload += count;
	  payload += " Receive Count: ";
	  payload += msgRcvCount;
    payload += "\nRestart Count: ";
    payload += mqttConnectCount - 1;
    payload += "  Avg Up Time: ";
    payload += days_hrs_mins_secs(mqttAvgUpTimeSec);
	  payload += "\nCurrent Uptime: ";  // add current uptime to message payload
	  payload += days_hrs_mins_secs((millis() - mqttStartTime)/1000); // add current uptime to message payload


    Serial.print("Sending message to topic: ");
    Serial.println(outTopic);
    Serial.println(payload);

    // send message, the Print interface can be used to set the message contents
    // in this case we know the size ahead of time, so the message payload can be streamed

    bool retained = false;
    int qos = 1;
    bool dup = false;

    mqttClient.beginMessage(outTopic, payload.length(), retained, qos, dup);
    mqttClient.print(payload);
    mqttClient.endMessage();

    Serial.println();

    count++;

    // toggle the onboard LED on each published message
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(LED, ledState);
  }
}

void onMqttMessage(int messageSize) {
  // we received a message, print out the topic and contents
  // and make it available in the buffer called 'msg'
  Serial.print("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', duplicate = ");
  Serial.print(mqttClient.messageDup() ? "true" : "false");
  Serial.print(", QoS = ");
  Serial.print(mqttClient.messageQoS());
  Serial.print(", retained = ");
  Serial.print(mqttClient.messageRetain() ? "true" : "false");
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  int i = 0;  // character array interator

  // use the stream interface to read the message data
  while (mqttClient.available()) {
    msg[i] = mqttClient.read();  // read each byte into buffer
    i = i + 1;
  }
  msg[i] = '\0';

  newMsg = true;
  msgRcvCount += 1;  // increment the received message counter
  Serial.println(msg); // print the message received

  Serial.println();
}

void flash3(String s){
  // used to flash LED3 quickly for 5 seconds when an invalid message is recieved.

  const long i = 200;   // 300 mSec timer
  const long j = 5000;  // 5 second timer

  if (s == "Start") {
    // initialize the timer count variables if called with "Start"
    Serial.println("Start LED3 flash");
    glob_flash = true;
    cMillis = millis();
    p1Millis = cMillis;
    p2Millis = cMillis;
    led3State = LOW; //turn on the LED to start
  }
  cMillis = millis();
  // if glob_flash is true - fast flash at i mSec intervals for j mSec of time
  if (glob_flash && (cMillis - p1Millis >= i)) {
    p1Millis = cMillis;
    // Serial.println(cMillis, p1Millis);
    if (led3State == LOW) {
      led3State = HIGH;
    } else {
      led3State = LOW;
    }
  if (glob_flash && (cMillis - p2Millis >= j)) {
    Serial.println("Stop LED3 flash");
    Serial.println();
    led3State = HIGH; // turn the LED off
    glob_flash = false; // stop the fast flashing
  }
  // set the LED with the leState of the variable
  digitalWrite(LED3, led3State);
  }
}

void connectToWiFi(){

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  WiFi.macAddress(mac);

  char str[3] = "";
  char buf[30];

  // Generate a Hostname for this device based on myHostname plus the last byte of the MAC address
  strcpy(buf, myHostname);
  strcat(buf, ":");
  byte array[1] = {mac[0]};
  array_to_string(array, 1, str);  // convert the last byte of MAC address to a string
  strcat(buf, str);
  Serial.println(buf);

  // attempt to set Hostname of this device
  // WiFi.setHostname(buf); // not valid for ESP8266WiFi

  #ifdef ESP8266
  WiFi.mode(WIFI_STA);  // needed for ESP8266WiFi
  #endif

  // initialize the WiFi library network settings
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    // failed, retry
    Serial.print("*");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();
  // print the firmware version of the WiFi module
  // Serial.print("Firmware Version: ");
  // Serial.println(WiFi.firmwareVersion());  // not valid for ESP8266WiFi

  // print the MAC address beginning with byte 5 to byte 0
  Serial.print("MAC: ");

#ifdef ESP8266
// swap order of bytes for ESP8266 modules
  for (unsigned int i = 0; i < 6; i++) {
#else
// use this ordering for all others like MKR1010
  for (unsigned int i = 5; i > 0; i--) {
#endif

    Serial.print(mac[i],HEX);
    Serial.print(":");
  }
  Serial.println(mac[0],HEX);
  // print the SSID you are connected to
  Serial.print("CONNECTED TO: ");
  Serial.println(WiFi.SSID());
  // print the signal strength of the WiFi
  Serial.print("Signal Strength (RSSI): ");
  Serial.println(WiFi.RSSI());

#ifdef ESP8266
  // hostname function only available on ESP8266WiFi
  hostname = WiFi.hostname();  // get the current Hostname
  Serial.print("Hostname: ");
  Serial.println(hostname);
#endif

  // print this device's ip address
  ip = WiFi.localIP();
  Serial.print("IP: ");
  Serial.println(ip);
  subnetMask = WiFi.subnetMask(); // get current subnet mask
  Serial.print("Subnet Mask: ");
  Serial.print(subnetMask);
  gatewayIP = WiFi.gatewayIP();  // get current gateway IP
  Serial.print("  Gateway: ");
  Serial.println(gatewayIP);
}

void array_to_string(byte array[], unsigned int len, char buffer[]){
  // converts a byte array to a hex character string that can be printed
  // args are:
  //      byte array to be converted
  //      length of array
  //      destination char buffer

    for (unsigned int i = 0; i < len; i++){
        byte nib1 = (array[i] >> 4) & 0x0F;  // mask the first nibble of data in element i of array
        byte nib2 = (array[i] >> 0) & 0x0F;  // mask the second nibble of data in element i of array
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;  // convert the nibble to the ASCII character '0' to 'F'
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

boolean  MQTT_connect() {
  // Function to connect and reconnect as necessary to the MQTT server.
  // Should be called in the loop function and it will take care ff connecting.

  int8_t ret;

  // Stop if already connected.
  if (mqttClient.connected()) {
    Serial.println("MQTT already connected");
    return true;
  }

  Serial.print("Connecting to MQTT broker... ");

  while(!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    Serial.println("Retrying MQTT connection in 5 seconds...");
    // mqttClient.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");

  // set the message receive callback
  mqttClient.onMessage(onMqttMessage);

  Serial.print("Subscribing to topic: ");
  Serial.println(inTopic);
  Serial.println();

  // subscribe to a topic
  // the second paramter set's the QoS of the subscription,
  // the the library supports subscribing at QoS 0, 1, or 2
  int subscribeQos = 1;

  mqttClient.subscribe(inTopic, subscribeQos);

  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(inTopic);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(inTopic);
  Serial.println();

  return true;
}

String days_hrs_mins_secs(unsigned int s){
  // return a string with Days: Hrs:  Min:  Sec: calculated from the unsigned long s seconds argument
  unsigned int secs = 0;
  unsigned int t_mins = 0;
  unsigned int mins = 0;
  unsigned int t_hrs = 0;
  unsigned int hrs = 0;
  unsigned int days = 0;
  String r = "";

  if (s < 1) {
      // no complete seconds have elapsed
      r = "0 Days 0 Hrs 0 Mins 0 Secs";
      return r;
  }

  secs = s % 60;    // remaining # of seconds

  if (s >= 60) {
    t_mins = (s - secs) / 60; // total minutes
    mins = t_mins % 60;  // remainder of minutes
  }
  if (t_mins >= 60) {
      t_hrs = (t_mins - mins) / 60; // total hours
      hrs = t_hrs % 24; // remainder of hours in days
  }
  if (t_hrs >= 24) {
      days = (t_hrs - hrs) / 24; // number of days
  }
  r += days;
  r += " Days ";
  r += hrs;
  r += " Hrs ";
  r += mins;
  r += " Mins ";
  r += secs;
  r += " Secs";
  return r;
}
