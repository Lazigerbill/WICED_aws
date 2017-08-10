/*

 ####### THIS IS A WORKING VERSION FOR WICED TO SET NTP TIME THROUGH WIFI #######
  ####### check battery and wifi status #######
  ####### read temperature from DS18B20 #######
  ####### MQTT to AWS #######

 */

#include <adafruit_feather.h>
#include <adafruit_mqtt.h>
#include "certificates.h"
#include <Time.h> 
#include <TimeLib.h>
#include <OneWire.h>

/**************************************************************************/
/*!
    @brief  Initialization
*/
/**************************************************************************/

// ####### MQTT interval
const long interval = 5000; 
unsigned long previousMillis = 0;

// ####### OnBoard LED
const int onBoardLed = PA15;

// ####### Initiate OneWire
OneWire  ds(PC3);  // on pin PC3 (a 4.7K resistor is necessary)

// ####### AdafruitMQTT
#define MQTT_TX_BUFSIZE   1024
#define MQTT_RX_BUFSIZE   1024
AdafruitMQTT mqtt;

// ####### AWS IoT
#define AWS_IOT_MQTT_HOST              "a2r8j1et2l5ygq.iot.us-east-1.amazonaws.com"
#define AWS_IOT_MQTT_PORT              8883
#define AWS_IOT_MQTT_CLIENT_ID         "test_wiced"
#define AWS_IOT_MY_THING_NAME          "test_wiced"
#define AWS_IOT_MQTT_TOPIC             "/things/test_wiced/test"
#define MQTT_QOS_AT_MOST_ONCE           0
#define MQTT_QOS_AT_LEAST_ONCE          1

const char aws_private_key[] = 
"-----BEGIN RSA PRIVATE KEY-----\n"
"MIIEpAIBAAKCAQEAiroeRC3ffcQmER3YxEBGQQb5abc7EwgVWen34FQdxxiirVuc\n"
"AM4uzFFiw0hOs4xqdOgyRwKGVt/JzWHEkuZG7BzrMhT+4p//Qs8tAPhAj7u93LmH\n"
"mT6cJJac5R0Q6idSaZT/WPTopFAN+l1lOvByiKHiEKu33MhIrg6wyiN+fQnFVDHa\n"
"AQvYAMIMlgZPYfzJBse9NvAQ2c3NxqWroofxADitl3/IzjO1io73P0V8APsPBG8Q\n"
"+A46v2gZarS3Qdx9V/Go0m12sFk0fkr0wLAC7lsiFnFv9vDCxSMpt9YCxPq8Iagx\n"
"SiLHL//VJsLJ+FDAduvvWA/ndCqLeGOQjE4ggwIDAQABAoIBAEMNlOWFernyXif1\n"
"0jmBi7OQqGtFqmMXjCJqqWofnkkGilLarbvh7NLLH76eK5QzcZ1SQkL6YG6LOqJv\n"
"+DC851jm3XPWnCB0D1B+jRUj6SjQKQQQFQ+oKJayyeEgPHXBTd9EqmuX7P7GxwrV\n"
"q8FiSSnrfTZrmbI4iJDA/wilQVUz9pH7kU4Fo8gOdZL/Jy4YwL6HnWPt389pb6LV\n"
"9d71VhPXBK4M04M9p0eH3UycDJC7shCNxTi3SeN4wI9NQkRQVY4at8zKkLlJ4orx\n"
"1bfO0pKwPMCVcbXTC43Nu/GEuetozI3EWu3gxlQxxzxmgNXv5qn/F+tcQ2pIfUgO\n"
"B/7ExXECgYEAwd+4ppQUiYfM1IpEJUhzksIdfs4J4zS5ua486epaiM8l2KMmDocP\n"
"v+muV7Ip46dj6R6Vjuzyrv0DS/CxP6OemzjWz2e6M+F0o9tZKY+YcbX9/CY8xZhl\n"
"mPeiD+hmoEfGLkqe+rmmPL1hku3ibpIf5yem5J84mcyBqIf7zEFOJ9sCgYEAty54\n"
"QMWKYGhv7KhGyBm/JECKXubAV6y+8FDnFQh1Qo3sByw/JI7hFp2tDEsDjQ/PxvpO\n"
"MQmwb5rm6myj12upkkF/cxFQqV9i7glgGhIgjxlCaSmK+3+AksiawwlRaBi6t3B5\n"
"NcVA45iHKoicfoCBrQgSr7ABou1hVfOeiQIW/nkCgYATYF6+F1KuJCpum5sP9tvG\n"
"MSxtff4y/RQK4MUpw8hkn/9yVWv6S7lhPuOz1BSnshUkOXBNJpVis9refiHY8Gtr\n"
"rBScCgvsH35e2g9hPf7Ibp4B03iDbyXIUgeae2m2XzYJbl/RQzjAHVVhL/FPh780\n"
"hRwDAX7QsEXNKDocQfKjvQKBgQC2W2KHSKA63X0eoV7lcQwhsMaPNzQfk+75GfER\n"
"tTvWp6ZidZ/eawaVFx7gcAT29tingyM4Gic004YxtTleCOXknaOdD423LygzQ84R\n"
"5h3XlxPP0PV07Tc36NB4fNw3vvaSCzv/VYEFkiBtLVZtcMynTrQbhiH66knN4Rrf\n"
"1d7CMQKBgQCXjRtd0++W1fPq6HS5PCC7CsafIrHv3Q0E04nznQJe/McmzlvrxZt2\n"
"4bccOv9J7AKwtmlnWrnMjUsESXghKLu5kMgTJEHpC2k25j8+LAEGHvjRhs2MH4Bb\n"
"d2+BeFswM0b9WOfvgKfXHo40KUd6nAobHTll5spqbnyHs1T90lRhmg==\n"
"-----END RSA PRIVATE KEY-----";

// ####### Adafruit UDP Settings
unsigned int localPort = 2390;      // local port to listen for UDP packets
IPAddress timeServer(132, 163, 4, 101); // time.nist.gov NTP server
// 129.6.15.28
// 129.6.15.29
// 129.6.15.30
// 132.163.4.101
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
AdafruitUDP Udp; // A UDP instance to let us send and receive packets over UDP

/**************************************************************************/
/*!
    ######### setup  ############
*/
/**************************************************************************/
void setup(){
  pinMode(PA1, INPUT_ANALOG); 
  pinMode(onBoardLed, OUTPUT);

  Serial.begin(115200);

  // Wait for the Serial Monitor to open
  // while (!Serial){
  //   delay(1);  //Delay required to avoid RTOS task switching problems 
  // };

  // #######  Setup for WICED WIFI
  Feather.printVersions(); // Print all software versions
  Serial.println("Saved AP profile"); // Print AP profile list
  Serial.println("ID SSID                 Sec");
  for(uint8_t i=0; i<WIFI_MAX_PROFILE; i++)
  {
    char * profile_ssid = Feather.profileSSID(i);
    int32_t profile_enc = Feather.profileEncryptionType(i);

    Serial.printf("%02d ", i);
    if ( profile_ssid != NULL ){
      Serial.printf("%-20s ", profile_ssid);
      Feather.printEncryption( profile_enc );
      Serial.println();
    }else{
      Serial.println("Not Available ");
    }
  }
  Serial.println();
  Feather.setDisconnectCallback(wifi_disconnect_callback); // Set disconnection callback
  Serial.println("Attempting to connect with saved profile");
  while ( !connectAP() ){
    delay(500); // delay between each attempt
  };
  Feather.printNetwork();  // Connected: Print network info

  //  ####### Connect to UDP Server
  Serial.println("\nStarting connection to UDP server...");
  Udp.begin(localPort);
  sendNTPpacket(timeServer); // send an NTP packet to a time server
  delay(1000);// wait to see if a reply is available
  if ( Udp.parsePacket() ) {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = " );
    Serial.println(secsSince1900);
    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);
    setTime(epoch);
    Serial.print("Now system time is set at: ");
    digitalClockDisplay();
  } else {
    Serial.println("Failed to get Internet Time!");
  }

// ####### AWS IoT settings
  // Tell the MQTT client to auto print error codes and halt on errors
  mqtt.err_actions(true, true);
  // Set ClientID
  mqtt.clientID(AWS_IOT_MQTT_CLIENT_ID);
  mqtt.setBufferSize(MQTT_TX_BUFSIZE, MQTT_RX_BUFSIZE);
  // Set the disconnect callback handler
  mqtt.setDisconnectCallback(mqtt_disconnect_callback);
  // default RootCA include certificate to verify AWS
  Feather.useDefaultRootCA(true);
  // Setting Indentity with AWS Private Key & Certificate
  mqtt.tlsSetIdentity(aws_private_key, local_cert, LOCAL_CERT_LEN);
  // Connect with SSL/TLS
  Serial.printf("Connecting to " AWS_IOT_MQTT_HOST " port %d ... ", AWS_IOT_MQTT_PORT);
  mqtt.connectSSL(AWS_IOT_MQTT_HOST, AWS_IOT_MQTT_PORT);
  Serial.println("OK");
  // Serial.print("Subscribing to " AWS_IOT_MQTT_TOPIC " ... ");
  // mqtt.subscribe(AWS_IOT_MQTT_TOPIC, MQTT_QOS_AT_MOST_ONCE, subscribed_callback); // Will halted if an error occurs
  // Serial.println("OK");
}
  
/**************************************************************************/
/*!
    ######### The loop function runs over and over again forever   #########
*/
/**************************************************************************/
void loop(){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    unsigned long epoch = now();
    String payload = "{ \"d\": ";
    payload += getDeviceStatus();
    payload += ", \"sensor\": {\"temperature\": ";
    digitalWrite(onBoardLed, HIGH);
    payload += getReadings();
    digitalWrite(onBoardLed, LOW);
    payload += "}, \"timestamp\": ";
    payload += epoch;
    payload += "}}";

    mqtt.publish(AWS_IOT_MQTT_TOPIC, (char*) payload.c_str(), MQTT_QOS_AT_LEAST_ONCE); // Will halted if an error occurs

  // FORMAT: { "d": {"status": {"battery_lvl": "USB Power", "wifi_signal": "-63"}, "sensor": {"temperature" : "XX"}, "timestamp": 1500258681}}
    previousMillis = currentMillis; // save the last time the cycle is run
  }
}

/**************************************************************************/
/*!
    @brief  MQTT subscribe event callback handler
    @param  topic      The topic causing this callback to fire
    @param  message    The new value associated with 'topic'
    @note   'topic' and 'message' are UTF8Strings (byte array), which means
            they are not null-terminated like C-style strings. You can
            access its data and len using .data & .len, although there is
            also a Serial.print override to handle UTF8String data types.
*/
/**************************************************************************/
void subscribed_callback(UTF8String topic, UTF8String message)
{
  Serial.print(message.data);
//  if ( 0 == memcmp(SHADOW_PUBLISH_STATE_OFF, message.data, message.len) )
//  {
//    digitalWrite(ledPin, LOW);
//  }
//  
//  if ( 0 == memcmp(SHADOW_PUBLISH_STATE_ON, message.data, message.len) )
//  {
//    digitalWrite(ledPin, HIGH);
//  }
}

/**************************************************************************/
/*!
    @brief  Wifi methods
*/
/**************************************************************************/
void wifi_disconnect_callback(void)
{
  Serial.println();
  Serial.println("-----------------------------");
  Serial.println("DISCONNECTED FROM WIFI");
  Serial.println("-----------------------------");
  Serial.println();
}

bool connectAP(void)
{
  // Attempt to connect to an AP
  Serial.print("Please wait while connecting ... ");
  
  if ( Feather.connect() )
  {
    Serial.println("Connected!");
  }
  else
  {
    Serial.printf("Failed! %s (%d)", Feather.errstr());
    Serial.println();
  }
  Serial.println();

  return Feather.connected();
}

/**************************************************************************/
/*!
    @brief  clock display methods
*/
/**************************************************************************/
void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}
void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

/**************************************************************************/
/*!
    @brief  get device setting
*/
/**************************************************************************/

String getDeviceStatus(){
  const int analogInPin = PA1; // The pin where the voltage divider is set
  int   vbatADC   = 0;         // The raw ADC value off the voltage div
  float vbatLSB   = 0.80566F;  // mV per LSB to convert raw values to volts
  float vbatFloat = 0.0F;      // The ADC equivalent in millivolts
  // Read the analog in value:
  vbatADC = analogRead(analogInPin);

  // Multiply the ADC by mV per LSB, and then
  // double the output to compensate for the
  // 10K+10K voltage divider
  vbatFloat = ((float)vbatADC * vbatLSB) * 2.0F;
  String vbatString = String(vbatFloat/1000,4) += "V";
  String batStatus;

  if (vbatADC > 2300)
  {
    
    batStatus = "USB";
  } else {
    batStatus = vbatString;
  };
  int rssi = Feather.RSSI();
  String res = "{\"status\": {\"battery_lvl\": \"";
  res += batStatus;
  res += "\", \"wifi_signal\": \"";
  res += rssi;
  res += "\"}";
  return res;
}

/**************************************************************************/
/*!
    @brief  Disconnect handler for MQTT broker connection
*/
/**************************************************************************/

void mqtt_disconnect_callback(void)
{
  Serial.println();
  Serial.println("-----------------------------");
  Serial.println("DISCONNECTED FROM MQTT BROKER");
  Serial.println("-----------------------------");
  Serial.println();
}

// ####### NTP function, send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address){
  Serial.println("1");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  Serial.println("2");
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  Serial.println("3");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Serial.println("4");
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Serial.println("5");
  Udp.endPacket();
  Serial.println("6");
}

// ####### Function for DS18B20
String getReadings(){
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  // byte addr[8] = {0x28, 0xFF, 0x51, 0x16, 0xA1, 0x16, 0x03, 0xD8};
  float celsius, fahrenheit;

  ds.reset_search();

  if (ds.search(addr)) {
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end
    
    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.
    
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad

    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
    }

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
    
    celsius = (float)raw / 16.0;
    fahrenheit = celsius * 1.8 + 32.0;

    String res = "{\"celsius\": ";
    res += celsius;
    res += ", \"fahrenheit\": ";
    res += fahrenheit;
    res += "}";
    return res;
  } 
}  