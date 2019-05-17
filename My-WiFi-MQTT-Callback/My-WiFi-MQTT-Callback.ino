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

#include <ArduinoMqttClient.h>
#include <WiFiNINA.h> // for MKR1000 change to: #include <WiFi101.h>

#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

// To connect with SSL/TLS:
// 1) Change WiFiClient to WiFiSSLClient.
// 2) Change port value from 1883 to 8883.
// 3) Change broker value to a server with a known SSL/TLS root certificate 
//    flashed in the WiFi module.

// initial variables for pin IO
const int LED = 6; // built in board LED
const int LED1 = 0;
const int LED2 = 1;
const int LED3 = 2;
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
int ledState = HIGH; // start with LED off
int led3State = HIGH; // start with LED off

int count = 0;

void setup() {
  // initialize digital pins as an output.
  
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  digitalWrite(LED1, HIGH);    // turn all the LEDs off to start HIGH = LED OFF
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

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

  // You can provide a unique client ID, if not set the library uses Arduin-millis()
  // Each client must have a unique client ID
  mqttClient.setId("MKR1010");

  // You can provide a username and password for authentication
  // mqttClient.setUsernamePassword("username", "password");

  // set a will message, used by the broker when the connection dies unexpectantly
  // you must know the size of the message before hand, and it must be set before connecting
  String willPayload = "oh no! - MKR1010 stopped resonding";
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

  mqttClient.poll();

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
        digitalWrite(LED3, LOW);    // turn the LED ON by making the voltage LOW
      }
      else if (strcmp(msg, "LED 3 OFF") == 0) {
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
  
  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    String payload;

    payload += "BarricAid Status: ";
    payload += " ";
    payload += count;

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
  
  Serial.println(msg); // print the message received
  
  Serial.println();
}

void flash3(String s){

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
    Serial.println("flash short: ");
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
