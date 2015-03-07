/*
To Compile (IDE 1.6.x).  Board = panStamp AVR

To Do:
Create new xivel feed and change channel IDs from numbers to text
Change from ERxPachube.h to Xively.h - maybe  http://github.com/xively/xively_arduino


Overview:
panStamp receives data from outside rain collector.  The data it receives is rain pulses (0.01" rain per pulse), heater on/off status and several temperatures. 
This data is then uploaded to Xively

Since both panstamp and Ethernet libary use pin D10 for slave select, some of the Ethernet libraries need to be modified to use a different pin.
SurferTim modified library files so Ethernet.select() would work:
Got new library from SurferTim here:  http://forum.arduino.cc/index.php?topic=217423.msg1962182#msg1962182

Difference between stock Arduino v1.5.8 original and SurferTim's are:
w5100.h      - https://www.diffchecker.com/2zzxtslg
w5100.cpp    - https://www.diffchecker.com/hl0s1zmd
Ethernet.h   - https://www.diffchecker.com/hwg0wq0p
Ethernet.cpp - https://www.diffchecker.com/6jdpzevk

Main Hardware:
 WIZ811MJ Ethernet module https://www.sparkfun.com/products/9473
 panStamp AVR  http://www.panstamp.com/product/panstamp-avr/


Xively Streams ( http://xively.com/feeds/103470 )
If you add a stream, you need to increase XIVELY_STREAMS
 0  Inches of rain.  Take pulses and divide by 10
 1  Heater On/Off
 2  Heating Pad 1 temp
 3  Heating Pad 2 temp
 4  Heating Pad 3 temp - future use
 5  Temp inside rain collector
 6  Outside temp
 7  Spare
 8  Xively upload successes
 9  Xively upload failures 
10  panStamp TxRx successes
11  Rx Status


Change Log:
09/13/14 v2.00 - Combined panStamp Rx and Base code together so panStamp can control Ethernet shield
10/04/14 v2.01 - Renamed a few things, some minor cleanup
10/17/14 v2.02 - Changed networks address (syncword)to two byte array
12/02/14 v2.03 - Compiled with IDE 1.5.8 and SurferTim modified libraries - doesn't work, hangs on Serial.println(Ethernet.localIP());
12/09/14 v2.04 - Added debugging code to find lockup problem.  Commented out Ethernet code and found it's hanging on radio.init().  Using stock Ethernet libraries
12/10/14 v2.05 - Updated for IDE 1.5.8 and panStamp API v2.0.  Finally got it working
01/09/15 v2.06 - Tx and Rx LED pins were reversed
02/02/15 v2.06 - Added new Xively channel for Rx status. 0 if not receiving data from outside, 1 if ok
02/28/15 v2.07 - changed some comments
03/03/15 v2.08 - Added RSSI 

*/

#define VERSION "v2.07"
// #define PRINT_DEBUG  // comment this out to turn off serial printing

#include <Arduino.h>
#include <SPI.h>             // http://arduino.cc/en/Reference/SPI
#include "HardwareSerial.h"  // Required by IDE 1.6.x
#include <ERxPachube.h>      // Library to upload to Xively  http://code.google.com/p/pachubelibrary/
#include <Ethernet.h>        // LIbrary for Arduino ethernet shield http://arduino.cc/en/Reference/Ethernet
#include <avr/wdt.h>         // Watchdog timer, Example code: http://code.google.com/p/arduwind/source/browse/trunk/ArduWind.ino
#include <Tokens.h>          //  Xively API Key


#define UPDATE_INTERVAL  20000   // Xively upload interval (mS) 
#define UPDATE_TIMEOUT  600000   // 10 minute timeout - if there are no successful updates in 10 minutes, reboot
uint32_t g_uploadTimout_timer = UPDATE_TIMEOUT; // Timer to reboot if no successful uploads in 10 minutes

// Xively Feed IDs
#define FEED_ID_TEST   4663  // Test feed  http://xively.com/feeds/4663
#define FEED_ID_RAIN 103470  // Rain Collector  http://xively.com/feeds/103470

ERxPachubeDataOut dataout_Rain(XIVELY_API_KEY, FEED_ID_RAIN);

const byte XIVELY_STREAMS = 12;

byte g_ip[] =  { 192, 168, 46, 83 };   // Suntec
// byte g_ip[] =  { 192, 168, 216, 40 };  // Crestview
byte g_mac[] = { 0xCC, 0xAC, 0xBE, 0x46, 0xFE, 0x99 }; 

// I/O
const byte ETH_SS_PIN =      9;  // Ethernet slave Select pin
const byte TX_OK_LED_PIN =   5;  // LED to flash when Xively upload succeeds
const byte RX_OK_LED_PIN =   4;  // LED to flash when panStamp packet is received

const byte g_RF_Channel =               0;  // panStamp channel
      byte g_psNetworkAddress[] = {10, 0};  // panStamp network address, aka SyncWord
const byte g_psReceiverAddress =       40;   // panStamp inside Rx address
      enum rx_status_t {RX_BAD, RX_GOOD};   // Status of packets received.
  uint32_t g_gotpsPacketTime =          0;   // Keeps track of time since last successful panStamp packet from rain collector
   uint8_t g_panStampSuccesses =        0;   // panStamp RxTx successes  
    int8_t g_rssi =                     0;   // panStamp signal strength
   
volatile boolean g_psPacketAvail = false;   // flag indicates a wireless panStamp packet has been received
 

// Variables to hold data coming from rain collector
int  g_tempIn;          // Temp inside rain collector
int  g_tempOut;         // Outdoor teamp
int  g_tempHeatPad1;    // Heating pad temp
int  g_tempHeatPad2;    // Heating pad temp
int  g_rainPulseCount;  // Rain gauges pulses, 1 pulse = 0.01" rain
bool g_isHeaterOn;      // True if heater on
bool g_TxOK;            // True if rain collector is transmitting


uint8_t g_xively_successes =  0;  // Successful Xively uploads
uint8_t g_xively_failures =   0;  // Xively Network failures


// Function Prototypes
void radioSignalInterrupt(void);
int  uploadRainCollector();
void checkXivelyStatus(int statusXively);
void BlinkLed(byte ledPin);
void PrintDataStream(const ERxPachube& pachube);
void printpanStampDeviceInfo();
void software_Reset();


// Routine to process interrupt from CC1101 (INT 0)
void radioSignalInterrupt(CCPACKET *psPacket)
{
  if( psPacket->crc_ok && psPacket->length > 1 )
  { 
    g_psPacketAvail = true; 
    g_gotpsPacketTime = millis(); // keep track of time since last good packet
    // Put packet data into global vairables
    //                  psPacket->data[0];    // Inside receiver address
    //                  psPacket->data[1];    // Outside transmitter address
    g_isHeaterOn =      psPacket->data[2];
    g_rainPulseCount =  psPacket->data[3] << 8;
    g_rainPulseCount |= psPacket->data[4];
    g_tempOut =         psPacket->data[5] << 8;
    g_tempOut |=        psPacket->data[6];
    g_tempIn =          psPacket->data[7] << 8;
    g_tempIn |=         psPacket->data[8];
    g_tempHeatPad1 =    psPacket->data[9] << 8;
    g_tempHeatPad1 |=   psPacket->data[10];         
    g_tempHeatPad2 =    psPacket->data[11] << 8;
    g_tempHeatPad2 |=   psPacket->data[12];
    g_rssi =            psPacket->rssi;
  } 

}  // end radioSignalInterrupt()


void setup()
{
  Serial.begin(9600);
  
  pinMode(RX_OK_LED_PIN, OUTPUT);
  pinMode(TX_OK_LED_PIN, OUTPUT);
  
  delay(500);
  Serial.print(F("Rain Collector\nVerion "));
  Serial.println(VERSION);

  Ethernet.select(ETH_SS_PIN);  // Set slave select pin - requires modified Ethernet.h library
  Ethernet.begin(g_mac, g_ip);  // Initialize Ethernet 
  Serial.println(Ethernet.localIP());

  // setup upload feeds 
  for (int i=0; i < XIVELY_STREAMS; i++)
  { dataout_Rain.addData(i); }
  
  // Setup the panStamp radio
  panstamp.radio.setChannel(g_RF_Channel);
  panstamp.radio.setSyncWord(g_psNetworkAddress);  // Set network address, pointer to address
  panstamp.radio.setDevAddress(g_psReceiverAddress);
  panstamp.setPacketRxCallback(radioSignalInterrupt);  // Declare RF callback function
  
  printpanStampDeviceInfo();  // print panStamp config
  
  // Flash LEDs to indicate startup   
  BlinkLed(RX_OK_LED_PIN);
  BlinkLed(RX_OK_LED_PIN);
  BlinkLed(TX_OK_LED_PIN);
  BlinkLed(TX_OK_LED_PIN);
  
} // end setup()


void loop()
{
  static uint32_t upload_timer =  millis() + UPDATE_INTERVAL;       // Timer for uploading to Xively
  
  if(g_psPacketAvail)
  {
    panstamp.rxOff();  // turn panStamp off while processing data
    g_psPacketAvail = false; // Reset panStamp paccket avail flag
    BlinkLed(RX_OK_LED_PIN);
    panstamp.rxOn(); // Finished prcessing, turn panStamp back on 
  }  

  // If panStamp packets haven't been received in 15 seconds, then set TxOK flag to false 
  // and clear variables
  if (long(millis() - g_gotpsPacketTime) > 15000L)
  { 
    g_TxOK = false; 
    // clear data
    g_isHeaterOn = false;
    g_rainPulseCount = 0;
    g_tempOut =        0;
    g_tempIn =         0;
    g_tempHeatPad1 =   0;
    g_tempHeatPad2 =   0;
  }
  else
  { g_TxOK = true; }

  // Upload to Xively
  if ((long)(millis() - upload_timer) > 0 )
  {
    Serial.println(F("Upload to Xively"));
    panstamp.rxOff();
    uploadRainCollector(g_TxOK);
    upload_timer = millis() + UPDATE_INTERVAL;
    panstamp.rxOn();
  }

} // end loop()


void BlinkLed(byte ledPin)
{
  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);
}  // end BlinkLed()


// Upload rain collector data to Xively
int uploadRainCollector(bool gotRainData)
{
  static rx_status_t Rx_Status;                // Status of whether or not data is being received from outside Tx
  static uint32_t Rx_Status_timer = millis();  // Timer resets every time new panStamp packet is received
  
  // If transmitter is sending data, increment g_panStampSuccesses
  if( gotRainData )
  { 
    g_panStampSuccesses++; 
    Rx_Status_timer = millis() + 1800000UL;  // Got data from Tx, so add 30 minutes to panStamp packet Rx timer
  }
  else
  {
    #ifdef PRINT_DEBUG
      Serial.println(F("Not receiving data from ouside"));
    #endif
  }
  
  // Set Rx status indicating if panStamp is receiving packets from outside
  // If no packets after 30 minutes, change status to RX_BAD (zero)
  if ((long)(millis()- Rx_Status_timer) > 0)
  { Rx_Status = RX_BAD; }
  else
  { Rx_Status = RX_GOOD; }
  

  dataout_Rain.updateData(0, (float) (g_rainPulseCount / 100.0));   // Rain - convert to inches
  dataout_Rain.updateData(1, g_isHeaterOn);                         // Heater on/off status
  dataout_Rain.updateData(2, g_tempHeatPad1);                       // Heating pad 1 temp
  dataout_Rain.updateData(3, g_tempHeatPad2);                       // Heating pad 2 temp
//dataout_Rain.updateData(4, tempH3);                               // Heating pad 3 temp - future
  dataout_Rain.updateData(5, g_tempIn);                             // Temp inside rain collector
  dataout_Rain.updateData(6, (int) g_tempOut);                      // Outside temp 
//dataout_Rain.updateData(7, rainData[]);                           // Spare
  dataout_Rain.updateData(8, g_xively_successes);                   // Xively upload successes
  dataout_Rain.updateData(9, g_xively_failures);                    // Xively upload failures
  dataout_Rain.updateData(10, g_panStampSuccesses);                 // panStamp Rx successes
  dataout_Rain.updateData(11, Rx_Status);                           // panStamp Rx status: 0 = not getting data, 1 = OK
  
  int xively_upload_status = dataout_Rain.updatePachube();  // send data to Xively
  
  #ifdef PRINT_DEBUG
    if (xively_upload_status == 200)
    { PrintDataStream(dataout_Rain); }
  #endif
  
  checkXivelyStatus(xively_upload_status);  // prints errors, updates success/failure counters

  return xively_upload_status;
  
}  // end uploadRainCollector()


void checkXivelyStatus(int statusXively)
{
  switch (statusXively)
  {
    case 0:
      Serial.println(F("Unknown Error"));
      g_xively_failures++;
      Serial.print(F("Failures = "));
      Serial.println(g_xively_failures);
      break;
    case 1:
      Serial.println(F("Can't connect to server"));
      g_xively_failures++;
      Serial.print(F("Failures = "));
      Serial.println(g_xively_failures);
      break;
    case 3:
      Serial.println(F("Data stream is empty"));
      g_xively_failures++;
      Serial.print(F("Failures = "));
      Serial.println(g_xively_failures);
      break;
    case 200: // success!
      Serial.println(F("Upload succeeded"));
      g_uploadTimout_timer = millis() + UPDATE_TIMEOUT; // Reset upload timout timer
      g_xively_successes++;
      g_xively_failures = 0;
      BlinkLed(TX_OK_LED_PIN);
      #ifdef WDT  
        wdt_reset();
      #endif
      break;
    default:
      Serial.print(F("Unknown status: "));
      Serial.println(statusXively);
      g_xively_failures++;
      Serial.print(F("Failures = "));
      Serial.println(g_xively_failures);
      break;
  }  
      
} // end checkXivelyStatus


// Print data uploaded to Xively
void PrintDataStream(const ERxPachube& pachube)
{
  unsigned int count = pachube.countDatastreams();

  Serial.println();
  Serial.print(F("ID:\t"));
  for(unsigned int i = 0; i < count; i++)
  {
    Serial.print(pachube.getIdByIndex(i));
    Serial.print(F("\t"));
    if (i == 0 )
    { Serial.print(F("\t")); }
  }
  Serial.println();
  
  Serial.print(F("Val:\t"));
  for(unsigned int i = 0; i < count; i++)
  {
    Serial.print(pachube.getValueByIndex(i));
    Serial.print(F("\t"));
  }
  Serial.println();
  
} // end PrintDataStream()


void printpanStampDeviceInfo()
{
  // Print device setup info
  Serial.print(F("Radio Frequency = "));
  if(panstamp.radio.carrierFreq == CFREQ_868)
  { Serial.println(F("868 Mhz")); }
  else
  { Serial.println(F("915 Mhz")); }
  
  Serial.print(F("Channel = "));
  Serial.println(panstamp.radio.channel);
  Serial.print(F("Network address = "));
  Serial.println(panstamp.radio.syncWord[0]);
  Serial.print(F("Device address = "));
  Serial.println(panstamp.radio.devAddress);
  Serial.print(F("RSSI = "));
  Serial.println(g_rssi);

}  // end printpanStampDeviceInfo()


// Restarts program from beginning but does not reset the peripherals and registers
// Reference: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1241733710
void software_Reset() 
{
  asm volatile ("  jmp 0");  
} // end software_Reset()




