/***************************************************
  SMS Pet Food Dish
    
  Send an SMS text message when your pet food dish is low!
  This sketch uses and IR sensor and LED to detect if a pet
  food dish is full or empty, and uses Amazon's SNS system to
  send SMS notifications.
  
  See the Adafruit learning system guide for more details
  and usage information:
    http://learn.adafruit.com/sms-pet-food-dish/overview

  Dependencies:
  - Adafruit CC3000 Library 
    https://github.com/adafruit/Adafruit_CC3000_Library
  - RTClib Library
    https://github.com/adafruit/RTClib
  
  License:
 
  This example is copyright (c) 2013 Tony DiCola (tony@tonydicola.com)
  and is released under an open source MIT license.  See details at:
    http://opensource.org/licenses/MIT
  
  This code was adapted from Adafruit CC3000 library example 
  code which has the following license:
  
  Designed specifically to work with the Adafruit WiFi products:
  ----> https://www.adafruit.com/products/1469

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution

  SHA1 hash and signing code adapted from Peter Knight's library
  available at https://github.com/Cathedrow/Cryptosuite  
 ****************************************************/
#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
#include "sha1.h"

// CC3000 configuration
#define     ADAFRUIT_CC3000_IRQ    3    // MUST be an interrupt pin!
#define     ADAFRUIT_CC3000_VBAT   5    // VBAT and CS can be any two pins
#define     ADAFRUIT_CC3000_CS     10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, 
                                         ADAFRUIT_CC3000_IRQ, 
                                         ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIV2);

// Wireless network configuration
#define     WLAN_SSID              "myNetwork"      // cannot be longer than 32 characters!
#define     WLAN_PASS              "myPassword"
#define     WLAN_SECURITY          WLAN_SEC_WPA2  // Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2


// Amazon AWS configuration
#define     AWS_ACCESS_KEY         "your_AWS_access_key"                               // Put your AWS access key here.
#define     AWS_SECRET_ACCESS_KEY  "your_AWS_secret_access_key"                        // Put your AWS secret access key here.
#define     AWS_REGION             "us-east-1"                                         // The region where your SNS topic lives.
                                                                                       // See the table at: http://docs.aws.amazon.com/general/latest/gr/rande.html#sns_region
#define     AWS_HOST               "sns.us-east-1.amazonaws.com"                       // The host URL for the region where your SNS topic lives.
                                                                                       // See the table at: http://docs.aws.amazon.com/general/latest/gr/rande.html#sns_region
#define     SNS_TOPIC_ARN          "url_encoded_SNS_topic_ARN"                         // Amazon resource name (ARN) for the SNS topic to receive notifications.
                                                                                       // Note: This ARN _MUST_ be URL encoded!  See http://meyerweb.com/eric/tools/dencoder/ for an example URL encoder tool.

// SMS pet food dish configuration
#define     IR_LED                 8         // Digital pin that is hooked up to the IR LED.
#define     IR_SENSOR              7         // Digital pin that is hooked up to the IR sensor.
#define     BOWL_CHECK_SECONDS     5         // How long to wait (in seconds) between pet food dish level checks.
#define     PUBLISH_LIMIT_SECONDS  43200     // How long to wait (in seconds) between publish calls.  This will prevent too many messages from being published in a short period.
                                             // A value of 43200 will limit publish calls to twice per day at most.
#define     TIMEOUT_MS             15000     // How long to wait (in milliseconds) for a server connection to respond (for both AWS and NTP calls).

// The configuration below should not be changed.
#define     SHA1_HASH_LENGTH       20

unsigned long lastPolledTime = 0;   // Last value retrieved from time server.
unsigned long sketchTime = 0;       // CPU milliseconds since last time server query.
bool bowlState;                     // The current bowl full/empty status.
unsigned long lastBowlCheck = 0;    // The last time the bowl status was checked.
unsigned long lastSNSPublish = 0;   // The last time an SNS message was published.

void setup(void) {
  // Set up the input and output pins.
  pinMode(IR_LED, OUTPUT);
  pinMode(IR_SENSOR, INPUT);
  
  // Initialize bowl state.
  bowlState = isBowlFull();
  
  // Set up the serial port connection.
  Serial.begin(115200);
  Serial.println(F("Hello, CC3000!\n")); 
  
  // Set up the CC3000, connect to the access point, and get an IP address.
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
  Serial.println(F("Connected!"));
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100);
  }
 
  // Get an initial time value by querying an NTP server.
  unsigned long t = getTime();
  while (t == 0) {
    // Failed to get time, try again in a minute.
    delay(60*1000);
    t = getTime();
  }
  lastPolledTime = t;
  sketchTime = millis(); 
  
  Serial.println(F("Ready..."));
}

void loop(void) {
  // Update the current time.
  unsigned long currentTime = lastPolledTime + (millis() - sketchTime) / 1000;
  
  // Check for bowl changing state from full to empty periodically.
  if (currentTime - lastBowlCheck >= BOWL_CHECK_SECONDS) {
    bool newBowl = isBowlFull();
    // Check if bowl is transitioning to empty (i.e. was full but now is not)
    if (!newBowl && bowlState) {
      // Check if enough time has passed since the last publish.
      if (currentTime - lastSNSPublish >= PUBLISH_LIMIT_SECONDS) {
        // Publish the SNS message.  Make sure the topic and message text is URL encoded!
        snsPublish(SNS_TOPIC_ARN, "Bowl%20is%20LOW", currentTime);
        lastSNSPublish = currentTime;
      }
    }
    bowlState = newBowl;
    lastBowlCheck = currentTime;
  }
  
}

// Return true if the bowl is detected to be full.
boolean isBowlFull() {
  // Pulse the IR LED at 38khz for 1 millisecond
  pulseIR(1000);
  // Check if the IR sensor picked up the pulse (i.e. output wire went to ground).
  if (digitalRead(IR_SENSOR) == LOW) {
    return false; // Sensor can see LED, return not full.
  }
  return true; // Sensor can't see LED, return full.
}

// 38khz IR pulse function from Adafruit tutorial: http://learn.adafruit.com/ir-sensor/overview
void pulseIR(long microsecs) {
  // we'll count down from the number of microseconds we are told to wait
 
  cli();  // this turns off any background interrupts
 
  while (microsecs > 0) {
    // 38 kHz is about 13 microseconds high and 13 microseconds low
   digitalWrite(IR_LED, HIGH);  // this takes about 3 microseconds to happen
   delayMicroseconds(10);         // hang out for 10 microseconds, you can also change this to 9 if its not working
   digitalWrite(IR_LED, LOW);   // this also takes about 3 microseconds
   delayMicroseconds(10);         // hang out for 10 microseconds, you can also change this to 9 if its not working
 
   // so 26 microseconds altogether
   microsecs -= 26;
  }
 
  sei();  // this turns them back on
}

// Publish a message to an SNS topic.
// Note, both the topic and message strings _MUST_ be URL encoded before calling this function!
void snsPublish(const char* topic, const char* message, unsigned long currentTime) {
  // Set dateTime to the URL encoded ISO8601 format string.
  DateTime dt(currentTime);
  char dateTime[25];
  memset(dateTime, 0, 25);
  dateTime8601UrlEncoded(dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second(), dateTime);

  // Generate the signature for the request.
  // For details on the AWS signature process, see: 
  //   http://docs.aws.amazon.com/general/latest/gr/signature-version-2.html
  Sha1.initHmac((uint8_t*)AWS_SECRET_ACCESS_KEY, strlen(AWS_SECRET_ACCESS_KEY));
  Sha1.print(F("POST\n"));
  Sha1.print(AWS_HOST); Sha1.print(F("\n"));
  Sha1.print(F("/\n"));
  Sha1.print(F("AWSAccessKeyId="));
  Sha1.print(AWS_ACCESS_KEY);
  Sha1.print(F("&Action=Publish"));
  Sha1.print(F("&Message="));
  Sha1.print(message);
  Sha1.print(F("&SignatureMethod=HmacSHA1"));
  Sha1.print(F("&SignatureVersion=2"));
  Sha1.print(F("&Timestamp="));
  Sha1.print(dateTime);
  Sha1.print(F("&TopicArn="));
  Sha1.print(topic);
  Sha1.print(F("&Version=2010-03-31"));
  
  // Convert signature to base64
  // Adapted from Adafruit code for SendTweet example.
  uint8_t *in, out, i, j;
  char b64[27];
  memset(b64, 0, sizeof(b64));
  static const char PROGMEM b64chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  for(in = Sha1.resultHmac(), out=0; ; in += 3) { // octets to sextets
    b64[out++] =   in[0] >> 2;
    b64[out++] = ((in[0] & 0x03) << 4) | (in[1] >> 4);
    if(out >= 26) break;
    b64[out++] = ((in[1] & 0x0f) << 2) | (in[2] >> 6);
    b64[out++] =   in[2] & 0x3f;
  }
  b64[out] = (in[1] & 0x0f) << 2;
  // Remap sextets to base64 ASCII chars
  for(i=0; i<=out; i++) b64[i] = pgm_read_byte(&b64chars[(unsigned char)b64[i]]);
  
  // URL encode base64 signature.  Note, this is not a general URL encoding routine!
  char b64Encoded[100];
  memset(b64Encoded, 0, sizeof(b64Encoded));
  for(i=0, j=0; i<=out; i++) {
    uint8_t ch = b64[i];
    if (ch == '+') {
      b64Encoded[j++] = '%';  
      b64Encoded[j++] = '2';  
      b64Encoded[j++] = 'B';  
    }
    else if (ch == '/') {
      b64Encoded[j++] = '%';  
      b64Encoded[j++] = '2';  
      b64Encoded[j++] = 'F'; 
    }
    else {
      b64Encoded[j++] = ch;
    }
  }
  b64Encoded[j++] = '%';
  b64Encoded[j++] = '3';
  b64Encoded[j++] = 'D';
  
  // Make request to SNS API.
  uint32_t ip = 0;
  while (ip == 0) {
    if (!cc3000.getHostByName(AWS_HOST, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }
  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, 80);
  if (www.connected()) {    
    www.fastrprint(F("POST /?"));
    www.fastrprint(F("AWSAccessKeyId="));
    www.fastrprint(AWS_ACCESS_KEY);
    www.fastrprint(F("&Action=Publish"));
    www.fastrprint(F("&Message="));
    www.fastrprint(message);
    www.fastrprint(F("&SignatureMethod=HmacSHA1"));
    www.fastrprint(F("&SignatureVersion=2"));
    www.fastrprint(F("&Timestamp="));
    www.fastrprint(dateTime);
    www.fastrprint(F("&TopicArn="));
    www.fastrprint(topic);
    www.fastrprint(F("&Version=2010-03-31"));
    www.fastrprint(F("&Signature="));
    www.fastrprint(b64Encoded);  
    www.fastrprint(F(" HTTP/1.1\r\nHost: "));
    www.fastrprint(AWS_HOST);
    www.fastrprint(F("\r\nContent-Length: 0\r\n\r\n"));
  } 
  else {
    Serial.println(F("Connection failed"));    
    www.close();
    return;
  }
  
  // Read data until either the connection is closed, or the idle timeout is reached.
  Serial.println(F("AWS response:"));
  unsigned long lastRead = millis();
  while (www.connected() && (millis() - lastRead < TIMEOUT_MS)) {
    while (www.available()) {
      char c = www.read();
      Serial.print(c);
      lastRead = millis();
    }
  }
  www.close();
}

// Fill a 24 character buffer with the date in URL-encoded ISO8601 format, like '2013-01-01T01%3A01%3A01Z'.  
// Buffer MUST be at least 24 characters long!
void dateTime8601UrlEncoded(int year, byte month, byte day, byte hour, byte minute, byte seconds, char* buffer) {
  ultoa(year, buffer, 10);
  buffer[4] = '-';
  btoa2Padded(month, buffer+5, 10);
  buffer[7] = '-';
  btoa2Padded(day, buffer+8, 10);
  buffer[10] = 'T';
  btoa2Padded(hour, buffer+11, 10);
  buffer[13] = '%';
  buffer[14] = '3';
  buffer[15] = 'A';
  btoa2Padded(minute, buffer+16, 10);
  buffer[18] = '%';
  buffer[19] = '3';
  buffer[20] = 'A';
  btoa2Padded(seconds, buffer+21, 10);
  buffer[23] = 'Z';
}

// Print a value from 0-99 to a 2 character 0 padded character buffer.
// Buffer MUST be at least 2 characters long!
void btoa2Padded(uint8_t value, char* buffer, int base) {
  if (value < base) {
    *buffer = '0';
    ultoa(value, buffer+1, base);
  }
  else {
    ultoa(value, buffer, base); 
  }
}

// getTime function adapted from CC3000 ntpTest sketch.
// Minimalist time server query; adapted from Adafruit Gutenbird sketch,
// which in turn has roots in Arduino UdpNTPClient tutorial.
unsigned long getTime(void) {
  Adafruit_CC3000_Client client;
  uint8_t       buf[48];
  unsigned long ip, startTime, t = 0L;

  // Hostname to IP lookup; use NTP pool (rotates through servers)
  if(cc3000.getHostByName("pool.ntp.org", &ip)) {
    static const char PROGMEM
      timeReqA[] = { 227,  0,  6, 236 },
      timeReqB[] = {  49, 78, 49,  52 };

    startTime = millis();
    do {
      client = cc3000.connectUDP(ip, 123);
    } while((!client.connected()) &&
            ((millis() - startTime) < TIMEOUT_MS));

    if(client.connected()) {
      // Assemble and issue request packet
      memset(buf, 0, sizeof(buf));
      memcpy_P( buf    , timeReqA, sizeof(timeReqA));
      memcpy_P(&buf[12], timeReqB, sizeof(timeReqB));
      client.write(buf, sizeof(buf));

      memset(buf, 0, sizeof(buf));
      startTime = millis();
      while((!client.available()) &&
            ((millis() - startTime) < TIMEOUT_MS));
      if(client.available()) {
        client.read(buf, sizeof(buf));
        t = (((unsigned long)buf[40] << 24) |
             ((unsigned long)buf[41] << 16) |
             ((unsigned long)buf[42] <<  8) |
              (unsigned long)buf[43]) - 2208988800UL;
      }
      client.close();
    }
  }
  return t;
}

