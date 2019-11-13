// Arduino RFID-Wiegand Reader adapted for use with ESP32
//  Plus MQTT messaging
// by J. Daniel Ozeran 13 October 2018
//  - 4 December 2018:   Debugging with Ideaworks MQTT server to ignore keypad presses
//                       and cardIDs < 10000 (less than 5 valid digits).
//  - 17 September 2019: Rebuild to deal with router failure and take advantage of "FresnoIdeaworks" access point
//                       and to debug for update to ESP32 core leading to AsyncTCP libary update.  Still need to debug
//                       to confirm that update to ArduinoJSON library version 6 provides correct data.
//
//    Multiple libraries were found for "WiFi.h"
//      Used: C:\Users\Dan\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.3\libraries\WiFi
//      Not used: C:\Program Files (x86)\Arduino\libraries\WiFi
//    Using library WiFi at version 1.0 in folder: C:\Users\Dan\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.3\libraries\WiFi
//    Using library async-mqtt-client-master at version 0.8.2 in folder: C:\Users\Dan\Documents\Arduino\libraries\async-mqtt-client-master
//    Using library AsyncTCP-master at version 1.1.0 in folder: C:\Users\Dan\Documents\Arduino\libraries\AsyncTCP-master
//    Using library ArduinoJson at version 6.12.0 in folder: C:\Users\Dan\Documents\Arduino\libraries\ArduinoJson
//
// From Corey Harding  RFID   24 November 2015
// https://legacysecuritygroup.com/projects/recent/9-rfid/21-arduino-hid-reader#
//
//
#include <WiFi.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
}
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

// Change the credentials, so the ESP32 connects to the proper router
#define WIFI_SSID "FresnoIdeaworks"
#define WIFI_PASSWORD "8gp28juu#6bc"

// Change the MQTT_HOST variable to the Raspberry Pi IP address,
// so it connects to your Mosquitto MQTT broker
#define MQTT_HOST IPAddress(192,168,1,125)
#define MQTT_PORT 1883

// Create objects to handle MQTT client
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

//  Create objects to handle JSON elements for MQTT messages
const int capacity = JSON_OBJECT_SIZE(4);
DynamicJsonDocument outgoing(1024);
//JsonObject outgoing = doc.to<JsonObject>();
String JSONString;      // Variable to hold the card data


#define MAX_BITS 100                 // max number of bits 
#define WEIGAND_WAIT_TIME  100000      // time to wait for another weigand pulse.  
unsigned char databits[MAX_BITS];    // stores all of the data bits note that the 'bits' are stored as chars
volatile unsigned int bitCount = 0;
unsigned char flagDone;              // goes low when data is currently being captured
unsigned int weigand_counter;        // countdown until we assume there are no more bits
volatile unsigned long facilityCode = 0;      // decoded facility code
volatile unsigned long cardCode = 0;          // decoded card code

// Breaking up card value into 2 chunks to create 10 char HEX value
volatile unsigned long bitHolder1 = 0;
volatile unsigned long bitHolder2 = 0;
volatile unsigned long cardChunk1 = 0;
volatile unsigned long cardChunk2 = 0;

uint8_t pinD0 = 36;
uint8_t pinIntD0 = 36;
uint8_t pinD1 = 39;
unsigned int wiegandbits = 26;
unsigned int packetGap = 15;
uint8_t pinIntD1 = 39;
uint8_t redLED = 25;
uint8_t greenLED = 26;
uint8_t doorRelay = 16;
int rfid = 0;

// sitter variable to allow for toggling shop status
bool sitter = false;

int m, i; //  extra integer variables for loops, etc.
char c[255] = "";      // long enough to hold complete integer string
char cardID[10] = "";

///////////////////////////////////////////////////////
//  Door response Routines
///////////////////////////////////////////////////////
void doorOpen() {
  digitalWrite(doorRelay, HIGH);
  delay(5000);
  digitalWrite(doorRelay, LOW);
}

void doorNoOpen() {
  for (int i = 0; i <= 10; i++) {
    digitalWrite(redLED, LOW);
    delay(100);
    digitalWrite(redLED, HIGH);
    delay(100);
  }
}


void sitterOpen() {
  // Have to allow sitter ~5 seconds to push keypad, then open door anyway
  Serial.print("Sitter = "); Serial.println(sitter);
  digitalWrite(doorRelay, HIGH);
  //sitter == true;
  for (int i = 0; i <= 5; i++) {
    digitalWrite(redLED, LOW);
    delay(250);
    digitalWrite(greenLED, LOW);
    delay(250);
    digitalWrite(redLED, HIGH);
    delay(250);
    digitalWrite(greenLED, HIGH);
    delay(250);
  }
  digitalWrite(doorRelay, LOW);
  sitter = false;
  Serial.print("Sitter = "); Serial.println(sitter);
  //  doorOpen();
}

//////////////////////////////////////////////////////////////////////////
// Interrupt Service Routines
//////////////////////////////////////////////////////////////////////////
// interrupt that happens when INT0 goes low (0 bit)
void ISR_INT0()
{
  // Code to fix "Task watchdog got triggered" error
  TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed = 1;
  TIMERG0.wdt_wprotect = 0;
  // End code fix
  bitCount++;
  flagDone = 0;

  if (bitCount < 23) {
    bitHolder1 = bitHolder1 << 1;
  }
  else {
    bitHolder2 = bitHolder2 << 1;
  }

  weigand_counter = WEIGAND_WAIT_TIME;

}

// interrupt that happens when INT1 goes low (1 bit)
void ISR_INT1()
{
  // Code to fix "Task watchdog got triggered" error
  TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed = 1;
  TIMERG0.wdt_wprotect = 0;
  // End code fix
  databits[bitCount] = 1;
  bitCount++;
  flagDone = 0;

  if (bitCount < 23) {
    bitHolder1 = bitHolder1 << 1;
    bitHolder1 |= 1;
  }
  else {
    bitHolder2 = bitHolder2 << 1;
    bitHolder2 |= 1;
  }

  weigand_counter = WEIGAND_WAIT_TIME;
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

// Add more topics that want your ESP32 to be subscribed to
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  // Subscribe the ESP32 to the 'ideaworks/fresno/frontdoor/reply' topic
  uint16_t packetIdSub = mqttClient.subscribe("ideaworks/fresno/frontdoor/reply", 0);
  Serial.print("Subscribing at QoS 0, packetId: ");
  Serial.println(packetIdSub);
  // Subscribe the ESP32 to the 'ideaworks/fresno/frontdoor/request' topic
  packetIdSub = mqttClient.subscribe("ideaworks/fresno/frontdoor/request", 0);
  Serial.print("Subscribing at QoS 0, packetId: ");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

// Modify this function to handle receipt of messages in specific topics
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;
  for (int i = 0; i < len; i++) {
    //Serial.print((char)payload[i]);  //Don't need to re-echo the MQTT message string to the serial device.
    messageTemp += (char)payload[i];
  }
  // Check if the MQTT message was received on topic ideaworks/fresno/frontdoor/request
  //  Parse the payload into a JSON object for handling
  if (strcmp(topic, "ideaworks/fresno/frontdoor/request") == 0) {
    //    JsonObject& obj = jb.parseObject(messageTemp);
    auto error = deserializeJson(outgoing, messageTemp);
    if (!error) {//(obj.success()) {
      //  parse object succeeded
      //      Serial.print("Printing from JSON Object: "); \
      //      obj.prettyPrintTo(Serial);
      //      Serial.println();
    } else {
      Serial.print("JSON Message failed");
      Serial.println(error.c_str());
      //  parse object failed
    }
    Serial.print("Received rfid: ");
    Serial.println(messageTemp);
  }
  // Check if the MQTT message was received on topic ideaworks/fresno/frontdoor/reply
  else if (strcmp(topic, "ideaworks/fresno/frontdoor/reply") == 0) {
    //  Received reply from server for prior request.
    Serial.print("Received server reply for topic: ");
    Serial.println(topic); Serial.print("Message: ");
    Serial.println(messageTemp);
    if (messageTemp == "OPEN") { //  If message is "OPEN" then activate the door latch for ~5 seconds
      doorOpen();
    }
    //  If message is "CLOSED" then flash the red LED for an error message
    else if (messageTemp == "CLOSED") {
      doorNoOpen();
    }
    //  If message is "SHOPSITTER" then flash both LEDs and allow the sitter to
    //  enter code on keypad.  After 5 seconds for keypad input, open door.
    else if (messageTemp == "SHOPSITTER") {
      sitter = true; // Serial.print("Sitter = "); Serial.println(sitter);
      sitterOpen();
    }
  }
}



void SendRequestMQTT(int rfid) {
  // Publish an MQTT message on topic ideaworks/fresno/frontdoor/request with payload = card number
  //  The next 2 lines are necessary to convert the string back
  //  into a character array since that's what the MQTT library
  //  is expecting.
  //  readCard();
  m = sprintf(c, "%05d", rfid);   // build integer string using C integer formatters  (m is length of string built, and not used
  // in this code since they are all 5 digits long)
  //  If there's a card at the reader, generate a publish message with the cardID.
  memcpy(cardID, c, sizeof(c));
  //  outgoing is the JSON construct
  if (rfid < 10000) {
    return;
  }
  outgoing["RFID"] = cardID;
  outgoing["ShopAccessID"] = 1;
  if (sitter == true) {           // 'sitter' will only be true if this is the second time a sitter's card is read within 5 seconds.
    sitter = false;
    outgoing["Other"] = "TOGGLESTATUS";
  }
  else {
    outgoing["Other"] = "Other";
  }
  serializeJson(outgoing, JSONString);
  Serial.print("Sending MQTT RFID posting with rfid = ");
  Serial.print(cardID);
  Serial.print(". Integer version: ");
  Serial.println(rfid);
  Serial.print("JSON String: ");
  Serial.println(JSONString);
  char charBuf[JSONString.length() + 1];
  JSONString.toCharArray(charBuf, JSONString.length() + 1);
  uint16_t packetIdPub2 = mqttClient.publish("ideaworks/fresno/frontdoor/request", 2, false, charBuf);
  Serial.print("Publishing on topic ideaworks/fresno/frontdoor/request at QoS 2, packetId: ");
  Serial.println(packetIdPub2);
  JSONString = "";
}


///////////////////////////////////////////////////////
// SETUP function
void setup()
{
  pinMode(redLED, OUTPUT);    // LED
  pinMode(greenLED, OUTPUT);  // LED Didn't get rid of this even though only one LED.
  pinMode(doorRelay, OUTPUT); // Relay connection
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, HIGH);
  digitalWrite(doorRelay, LOW);
  pinMode(pinD0, INPUT);     // DATA0 (INT0)
  pinMode(pinD1, INPUT);     // DATA1 (INT1)

  Serial.begin(115200);
  //  Set up timers and callbacks for MQTT client
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  Serial.println("RFID Reader Started");


  // binds the ISR functions to the falling edge of INTO and INT1
  attachInterrupt(digitalPinToInterrupt(pinIntD0), ISR_INT0, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinIntD1), ISR_INT1, FALLING);

  weigand_counter = WEIGAND_WAIT_TIME;
  connectToWifi();
}


///////////////////////////////////////////////////////
// LOOP function
void loop()
{
  // This waits to make sure that there have been no more data pulses before processing data
  if (!flagDone) {
    if (--weigand_counter == 0)
      flagDone = 1;
  }

  // if we have bits and we the weigand counter went out
  if (bitCount > 0 && flagDone) {
    unsigned char i;
    getCardValues();
    getCardNumAndSiteCode();
    SendRequestMQTT(cardCode);
    printBits();

    // cleanup and get ready for the next card
    bitCount = 0; facilityCode = 0; cardCode = 0;
    bitHolder1 = 0; bitHolder2 = 0;
    cardChunk1 = 0; cardChunk2 = 0;

    for (i = 0; i < MAX_BITS; i++)
    {
      databits[i] = 0;
    }
  }
}

///////////////////////////////////////////////////////
// PRINTBITS function
void printBits()
{
  // I really hope you can figure out what this function does
  Serial.print(bitCount);
  Serial.print(" bit card. ");
  Serial.print("FC = ");
  Serial.print(facilityCode);
  Serial.print(", CC = ");
  Serial.print(cardCode);
  Serial.print(", 44bit HEX = ");
  Serial.print(cardChunk1, HEX);
  Serial.println(cardChunk2, HEX);
  for (int i = 0; i < bitCount; i++) {
    Serial.print(databits[i]);
  };
  Serial.println();
  //  digitalWrite(redLED,LOW);
  //  delay(250);
  //  digitalWrite(greenLED,LOW);
  //  delay(250);
  //  digitalWrite(redLED,HIGH);
  //  delay(250);
  //  digitalWrite(greenLED,HIGH);

}


///////////////////////////////////////////////////////
// SETUP function
void getCardNumAndSiteCode()
{
  unsigned char i;

  // we will decode the bits differently depending on how many bits we have
  // see www.pagemac.com/azure/data_formats.php for more info
  // also specifically: www.brivo.com/app/static_data/js/calculate.js
  switch (bitCount) {


    ///////////////////////////////////////
    // standard 26 bit format
    // facility code = bits 2 to 9
    case 26:
      for (i = 1; i < 9; i++)
      {
        facilityCode <<= 1;
        facilityCode |= databits[i];
      }

      // card code = bits 10 to 23
      for (i = 9; i < 25; i++)
      {
        cardCode <<= 1;
        cardCode |= databits[i];
      }
      break;

    ///////////////////////////////////////
    // 33 bit HID Generic
    case 33:
      for (i = 1; i < 8; i++)
      {
        facilityCode <<= 1;
        facilityCode |= databits[i];
      }

      // card code
      for (i = 8; i < 32; i++)
      {
        cardCode <<= 1;
        cardCode |= databits[i];
      }
      break;

    ///////////////////////////////////////
    // 34 bit HID Generic
    case 34:
      for (i = 1; i < 17; i++)
      {
        facilityCode <<= 1;
        facilityCode |= databits[i];
      }

      // card code
      for (i = 17; i < 33; i++)
      {
        cardCode <<= 1;
        cardCode |= databits[i];
      }
      break;

    ///////////////////////////////////////
    // 35 bit HID Corporate 1000 format
    // facility code = bits 2 to 14
    case 35:
      for (i = 2; i < 14; i++)
      {
        facilityCode <<= 1;
        facilityCode |= databits[i];
      }

      // card code = bits 15 to 34
      for (i = 14; i < 34; i++)
      {
        cardCode <<= 1;
        cardCode |= databits[i];
      }
      break;

  }
  return;

}


//////////////////////////////////////
// Function to append the card value (bitHolder1 and bitHolder2) to the necessary array then tranlate that to
// the two chunks for the card value that will be output
void getCardValues() {
  switch (bitCount) {
    case 26:
      // Example of full card value
      // |>   preamble   <| |>   Actual card value   <|
      // 000000100000000001 11 111000100000100100111000
      // |> write to chunk1 <| |>  write to chunk2   <|

      for (int i = 19; i >= 0; i--) {
        if (i == 13 || i == 2) {
          bitWrite(cardChunk1, i, 1); // Write preamble 1's to the 13th and 2nd bits
        }
        else if (i > 2) {
          bitWrite(cardChunk1, i, 0); // Write preamble 0's to all other bits above 1
        }
        else {
          bitWrite(cardChunk1, i, bitRead(bitHolder1, i + 20)); // Write remaining bits to cardChunk1 from bitHolder1
        }
        if (i < 20) {
          bitWrite(cardChunk2, i + 4, bitRead(bitHolder1, i)); // Write the remaining bits of bitHolder1 to cardChunk2
        }
        if (i < 4) {
          bitWrite(cardChunk2, i, bitRead(bitHolder2, i)); // Write the remaining bit of cardChunk2 with bitHolder2 bits
        }
      }
      break;

    case 27:
      for (int i = 19; i >= 0; i--) {
        if (i == 13 || i == 3) {
          bitWrite(cardChunk1, i, 1);
        }
        else if (i > 3) {
          bitWrite(cardChunk1, i, 0);
        }
        else {
          bitWrite(cardChunk1, i, bitRead(bitHolder1, i + 19));
        }
        if (i < 19) {
          bitWrite(cardChunk2, i + 5, bitRead(bitHolder1, i));
        }
        if (i < 5) {
          bitWrite(cardChunk2, i, bitRead(bitHolder2, i));
        }
      }
      break;

    case 28:
      for (int i = 19; i >= 0; i--) {
        if (i == 13 || i == 4) {
          bitWrite(cardChunk1, i, 1);
        }
        else if (i > 4) {
          bitWrite(cardChunk1, i, 0);
        }
        else {
          bitWrite(cardChunk1, i, bitRead(bitHolder1, i + 18));
        }
        if (i < 18) {
          bitWrite(cardChunk2, i + 6, bitRead(bitHolder1, i));
        }
        if (i < 6) {
          bitWrite(cardChunk2, i, bitRead(bitHolder2, i));
        }
      }
      break;

    case 29:
      for (int i = 19; i >= 0; i--) {
        if (i == 13 || i == 5) {
          bitWrite(cardChunk1, i, 1);
        }
        else if (i > 5) {
          bitWrite(cardChunk1, i, 0);
        }
        else {
          bitWrite(cardChunk1, i, bitRead(bitHolder1, i + 17));
        }
        if (i < 17) {
          bitWrite(cardChunk2, i + 7, bitRead(bitHolder1, i));
        }
        if (i < 7) {
          bitWrite(cardChunk2, i, bitRead(bitHolder2, i));
        }
      }
      break;

    case 30:
      for (int i = 19; i >= 0; i--) {
        if (i == 13 || i == 6) {
          bitWrite(cardChunk1, i, 1);
        }
        else if (i > 6) {
          bitWrite(cardChunk1, i, 0);
        }
        else {
          bitWrite(cardChunk1, i, bitRead(bitHolder1, i + 16));
        }
        if (i < 16) {
          bitWrite(cardChunk2, i + 8, bitRead(bitHolder1, i));
        }
        if (i < 8) {
          bitWrite(cardChunk2, i, bitRead(bitHolder2, i));
        }
      }
      break;

    case 31:
      for (int i = 19; i >= 0; i--) {
        if (i == 13 || i == 7) {
          bitWrite(cardChunk1, i, 1);
        }
        else if (i > 7) {
          bitWrite(cardChunk1, i, 0);
        }
        else {
          bitWrite(cardChunk1, i, bitRead(bitHolder1, i + 15));
        }
        if (i < 15) {
          bitWrite(cardChunk2, i + 9, bitRead(bitHolder1, i));
        }
        if (i < 9) {
          bitWrite(cardChunk2, i, bitRead(bitHolder2, i));
        }
      }
      break;

    case 32:
      for (int i = 19; i >= 0; i--) {
        if (i == 13 || i == 8) {
          bitWrite(cardChunk1, i, 1);
        }
        else if (i > 8) {
          bitWrite(cardChunk1, i, 0);
        }
        else {
          bitWrite(cardChunk1, i, bitRead(bitHolder1, i + 14));
        }
        if (i < 14) {
          bitWrite(cardChunk2, i + 10, bitRead(bitHolder1, i));
        }
        if (i < 10) {
          bitWrite(cardChunk2, i, bitRead(bitHolder2, i));
        }
      }
      break;

    case 33:
      for (int i = 19; i >= 0; i--) {
        if (i == 13 || i == 9) {
          bitWrite(cardChunk1, i, 1);
        }
        else if (i > 9) {
          bitWrite(cardChunk1, i, 0);
        }
        else {
          bitWrite(cardChunk1, i, bitRead(bitHolder1, i + 13));
        }
        if (i < 13) {
          bitWrite(cardChunk2, i + 11, bitRead(bitHolder1, i));
        }
        if (i < 11) {
          bitWrite(cardChunk2, i, bitRead(bitHolder2, i));
        }
      }
      break;

    case 34:
      for (int i = 19; i >= 0; i--) {
        if (i == 13 || i == 10) {
          bitWrite(cardChunk1, i, 1);
        }
        else if (i > 10) {
          bitWrite(cardChunk1, i, 0);
        }
        else {
          bitWrite(cardChunk1, i, bitRead(bitHolder1, i + 12));
        }
        if (i < 12) {
          bitWrite(cardChunk2, i + 12, bitRead(bitHolder1, i));
        }
        if (i < 12) {
          bitWrite(cardChunk2, i, bitRead(bitHolder2, i));
        }
      }
      break;

    case 35:
      for (int i = 19; i >= 0; i--) {
        if (i == 13 || i == 11) {
          bitWrite(cardChunk1, i, 1);
        }
        else if (i > 11) {
          bitWrite(cardChunk1, i, 0);
        }
        else {
          bitWrite(cardChunk1, i, bitRead(bitHolder1, i + 11));
        }
        if (i < 11) {
          bitWrite(cardChunk2, i + 13, bitRead(bitHolder1, i));
        }
        if (i < 13) {
          bitWrite(cardChunk2, i, bitRead(bitHolder2, i));
        }
      }
      break;

    case 36:
      for (int i = 19; i >= 0; i--) {
        if (i == 13 || i == 12) {
          bitWrite(cardChunk1, i, 1);
        }
        else if (i > 12) {
          bitWrite(cardChunk1, i, 0);
        }
        else {
          bitWrite(cardChunk1, i, bitRead(bitHolder1, i + 10));
        }
        if (i < 10) {
          bitWrite(cardChunk2, i + 14, bitRead(bitHolder1, i));
        }
        if (i < 14) {
          bitWrite(cardChunk2, i, bitRead(bitHolder2, i));
        }
      }
      break;

    case 37:
      for (int i = 19; i >= 0; i--) {
        if (i == 13) {
          bitWrite(cardChunk1, i, 0);
        }
        else {
          bitWrite(cardChunk1, i, bitRead(bitHolder1, i + 9));
        }
        if (i < 9) {
          bitWrite(cardChunk2, i + 15, bitRead(bitHolder1, i));
        }
        if (i < 15) {
          bitWrite(cardChunk2, i, bitRead(bitHolder2, i));
        }
      }
      break;
  }
  return;
}
