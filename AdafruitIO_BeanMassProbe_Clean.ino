/***************************************************
  Bean Mass Probe for GeneCafe Coffee Roaster

  Code here is based on the Adafruit MQTT Library ESP8266 Example (mqtt_esp8266)
  
  MIT License
  
  Copyright (c) 2016 Evan Graham
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

****************************************************/


// IMPORTANT NOTE: You must modify the Adafruit_MAX31855 library for this 
// code to work correctly.  See http://roasthacker.com/?page_id=65 for details


#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"


/********************** Configuration *****************************/
// WiFi
#define WLAN_SSID       "YOUR SSID"
#define WLAN_PASS       "YOUR WLAN PASSWORD"

//MQTT
#define SERVERNAME      "io.adafruit.com"
#define SERVERPORT      1883
#define USERNAME        "YOUR ADAFRUIT IO USERNAME"
#define PASSWORD        "YOUR ADAFRUIT IO KEY"

//Pins
#define DO             2      // Orange wire
#define CS             4      // Green Wire
#define CLK            5      // Yellow Wire

//Other
#define READRATE       500                //# of milliseconds between readings
#define PUBLISHRATE    2000               //# of milliseconds between publishing data
#define DEBUG                             //uncomment for debugging

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

const char MQTT_SERVER[] PROGMEM    = SERVERNAME;
const char MQTT_CLIENTID[] PROGMEM  = __TIME__ USERNAME; //__TIME__ is the time code was complied; ensures unique client ID
const char MQTT_USERNAME[] PROGMEM  = USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = PASSWORD;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);

/****************************** Feeds ***************************************/

// Setup an MQTT feeds  
const char MAXTEMP_FEED[] PROGMEM = USERNAME "/feeds/MaxT";
Adafruit_MQTT_Publish maxTemp = Adafruit_MQTT_Publish(&mqtt, MAXTEMP_FEED);

const char CURRENTTEMP_FEED[] PROGMEM = USERNAME "/feeds/CurT";
Adafruit_MQTT_Publish currentTemp = Adafruit_MQTT_Publish(&mqtt, CURRENTTEMP_FEED);

const char MINTEMP_FEED[] PROGMEM = USERNAME "/feeds/MinT";
Adafruit_MQTT_Publish minTemp = Adafruit_MQTT_Publish(&mqtt, MINTEMP_FEED);

Adafruit_MAX31855 thermocouple(CLK, CS, DO);


int TmaxLoopCount=0;                                              // # of measurements since Tmax was set
int TminLoopCount=0;                                              // # of measurements since Tmin was set

double Tmin;                                                      // best estimate of minumum temperature (this is bean mass temp)
double Tmax;                                                      // best estimate of maximum temperature (this is environmental temp)
double Tnow;                                                      // current thermocouple reading (t=0)
double Tlast;                                                     // last thermocouple reading (t=-1)
double T_dif_now;                                                 // temp difference between most recent readings(t = 0 and = -1)
double T_dif_last;                                                // temp difference between previous two readings (t = -1 and t = -2)

unsigned long currentTime = millis();
unsigned long previousReadTime = 0;
unsigned long previousPublishTime = 0;

void setup() {
  // if debug enabled, initialize serial
  #ifdef DEBUG  
    Serial.begin(115200);
    Serial.println("Bean Mass Probe via MQTT");
  #endif
  
  // Connect to WiFi access point.
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #ifdef DEBUG    
      Serial.print(F("."));
    #endif
  }
  // print WiFi connection diagnostics
  #ifdef DEBUG
    Serial.print(F("Connecting to "));
    Serial.println(WLAN_SSID);
    Serial.println();
    Serial.println(F("WiFi connected"));
    Serial.println(F("IP address: "));
    Serial.println(WiFi.localIP());
  #endif
  // connect to MQTT Server
  connect();
  
  // initialize temperature variables
  Tmin = thermocouple.readFarenheit();
  Tlast = Tnow = Tmax = Tmin;
  T_dif_now = 0;
  T_dif_last = 0;
}

void loop() {
  // ping MQTT Broker a few times to make sure we remain connected
  if (!client.connected()) {
    #ifdef DEBUG
      Serial.println(F("\nWiFi Connection Lost"));
    #endif
  }
  if(! mqtt.ping(3)) {
    // reconnect to MQTT Broker
    if(! mqtt.connected()){
      connect();
      #ifdef DEBUG
        Serial.println(F("\nMQTT Connection Restarted"));
      #endif
    }
  }
  
   
  currentTime = millis();
  if(currentTime - previousReadTime > READRATE){
    previousReadTime = currentTime;
    //READ DATA
    #ifdef DEBUG
      Serial.print(F("."));
    #endif 
    Tnow = thermocouple.readFarenheit();
    TminLoopCount ++;
    TmaxLoopCount ++;
    T_dif_now = Tnow - Tlast; 
  
    if (T_dif_now >= 0.0 && T_dif_last < 0.0 && TminLoopCount > 1){    // this is a local minimum
      Tmin = Tlast;                                                    // best estimate of bean mass temp
      TminLoopCount = 0;                                               // reset loop counter
    }
     
    if (T_dif_now <= 0.0 && T_dif_last > 0.0 && TmaxLoopCount > 1){    // this is a local maximum
      Tmax = Tlast;                                                    // best estimate of environmental temp
      TmaxLoopCount = 0;                                               // reset loop counter
    }   
     
    Tlast = Tnow;
    T_dif_last = T_dif_now;
    
  }
 
  if(currentTime - previousPublishTime > PUBLISHRATE){
    previousPublishTime = currentTime;
    //PUBLISH DATA
    #ifdef DEBUG
      Serial.print(F("*"));
    #endif 
    maxTemp.publish(Tmax);
    currentTemp.publish(Tnow);
    minTemp.publish(Tmin);
//    mqtt.disconnect();  DON'T USE THIS WITH ADAFRUIT IO
  }
}


// connect to MQTT Broker
void connect() {
  Serial.print(F("Connecting to MQTT Broker... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if(ret >= 0)
      mqtt.disconnect();
    #ifdef DEBUG
      Serial.println(F("Retrying connection..."));
    #endif
    delay(5000);
  }
  #ifdef DEBUG
    Serial.println(F("MQTT Broker Connected!"));
  #endif
}
