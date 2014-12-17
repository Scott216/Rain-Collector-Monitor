/*
Rain collector heater 

Function:
Measure 4 temperatures: 2 heating pads, inside rain collector, outside rain collector
Turn heater on/off
Receive and retransmit pulse from rain collector
Sends data to a panStamp inside the house which uploads to Xively

      
I/O:
A0 - Thermistor under heater
A1 - Thermistor under heater
A2 - Thermistor on PCB
A3 - Thermistor outside 
D3 - Pulse input (interrupt pin) 
D4 - Pulse Output 
D5 - Heater on output 

Panstamp Packet Structure
Byte   Function
----   ------------------------------------
 0      Rx address
 1      Tx Address
 2      Heater On/Off
 3,4    Pulse count
 5,6    Outside Temp
 7,8    Inside rain collector temp
 9,10   Heater 1 Temp
 11,12  Heater 2 temp

Notes:
Heat: 
If outside temp is less then 40 degrees, turn on every hour for several minutes to see if you get any pulses
If so, keep heat on until pulses stop for 1 your. 
If heaters get too hot, then cycle them until you can figure out the best voltage.
use a low pass filter on thermistors

Change Log:
v0.10 09/12/14 - Added version number
v0.11 10/04/14 - Use interrupt for pulse in. Moved global variables to loop() and made static.  Renamed Thermistor() to thermistorTempF().  
                 Only send data every 1/2 second. 
v0.12 10/17/14 - Changed networks address (syncword)to two byte array
v0.13 10/18/14 - Compiled with new version of panStamp libraries v2

*/

#define VERSION "v0.13"
#define PRINT_DEBUG     // comment this out to turn of serial printint
#include <HardwareSerial.h> // Needed by Arduino IDE 1.5.x
#include <avr/wdt.h>  // http://github.com/simonz05/arduino-headers/blob/master/wdt.h
#include "EEPROM.h"   // http://www.arduino.cc/en/Reference/EEPROM
#include "cc1101.h"   // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/cc1101.h
#include "panstamp.h" // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/panstamp.h
                      // http://code.google.com/p/panstamp/wiki/PANSTAMPclass

// Analog pins
const int8_t   TEMP_HOT1 =  0;  // Temp on heating pad 1
const int8_t   TEMP_HOT2 =  1;  // Temp on heating pad 2
const int8_t   TEMP_IN =    2;  // Temp on PCB
const int8_t   TEMP_OUT =   3;  // Temp outside under rain collector
// Digital pin
const int8_t   PULSE_IN =          3;  // Rain pulse in
const int8_t   PULSE_OUT =         4;  // Rain pulse out to ISS on roof
const int8_t   HEAT_OUT =          5;  // Relay to turn heaters on
const uint32_t ONEMIN =      60000UL;  
const uint8_t  HIGH_TEMP_LIMIT = 175;  // high temperature limit (F)
const uint32_t HEATER_ON_TIME =  30UL * ONEMIN; // Time heater stays on after last pulse detected (if it's cold outside)

volatile bool g_gotRainPulse = false;  // set by rainPulse() interrupt when rain bucket tips


// The networkAdress of sender and receiver must be the same
// in the cc1101 documentation this byte is called syncword
// in the SWAP world of the panStamp it is called networkAddress
byte g_networkAddress[] =   {10,0};
byte g_receiverAddress =  40;
byte g_senderAddress =    41;  
CC1101  radio;   // http://code.google.com/p/panstamp/wiki/CC1101class

// Function Prototypes
float thermistorTempF(int RawADC);
void printpanStampConfig();
void rainPulse();  // Interrupt

void setup()
{
  #ifdef PRINT_DEBUG
    Serial.begin(9600);
    delay(3000);
    Serial.print("Rain Collector Tx ");
    Serial.println(VERSION);
  #endif
  
  pinMode(PULSE_IN,  INPUT );
  pinMode(PULSE_OUT, OUTPUT);
  pinMode(HEAT_OUT,  OUTPUT);
  digitalWrite(PULSE_OUT, LOW);
  digitalWrite(HEAT_OUT, LOW);
  
  attachInterrupt(1, rainPulse, FALLING);  // Pin D3 interrupt
  
  // Initialize the CC1101 RF Chip
  radio.init();
  radio.setSyncWord(g_networkAddress, false);   // true saves address to EEPROM
  radio.setDevAddress(g_senderAddress, false);   // true saves address to EEPROM

  #ifdef PRINT_DEGBUG
    printpanStampConfig();
  #endif
  wdt_enable (WDTO_8S);  // 8 second timeout for WDT
} // setup()


void loop()
{

  wdt_reset();  
  
  static int16_t tempIn;
  static int16_t tempOut;
  static int16_t tempHot1;
  static int16_t tempHot2; 
  double filterVal = 0.01;
  static uint32_t heatOnDelay;         // Delay so heater doesn't come back on too soon
  static bool heatOnDelayTmrOneshot;   // Flag used with heater on delay timer
  static bool hourlyCheckFlag;         // True if heater should come on for an hourly check to see if there is snow to melt.  
                                       // Heater shouls stay on for HEATER_ON_TIME 
 
  // Read temps, convert to F and smooth with low pass filter
  tempIn =    (thermistorTempF(analogRead(TEMP_IN))   * (1-filterVal)) + (filterVal * tempIn );
  tempOut =   (thermistorTempF(analogRead(TEMP_OUT))  * (1-filterVal)) + (filterVal * tempOut );
  tempHot1 =  (thermistorTempF(analogRead(TEMP_HOT1)) * (1-filterVal)) + (filterVal * tempHot1 );
  tempHot2 =  (thermistorTempF(analogRead(TEMP_HOT2)) * (1-filterVal)) + (filterVal * tempHot2 );
 
  // Turn on heater
  static uint32_t lastHeatOnTime = 0;   // Used to turn the heater on everyhour to see if any pulses start from melted snow
  static uint32_t lastPulseTime =  0;   // Used to see how long since the last pulse.  If heater is on an there are no pulses, then turn it off
  bool is_cold_outside = tempOut < 40;
  bool inside_temp_not_too_hot = tempIn   <  80;
  bool heat_pads_not_too_hot = (tempHot1 < HIGH_TEMP_LIMIT) && (tempHot2 < HIGH_TEMP_LIMIT);
  bool heat_timer_not_expired =  (((long)(millis() - lastPulseTime) < HEATER_ON_TIME) || hourlyCheckFlag == true);  
  bool heat_delay_has_passed = (long)(millis() - heatOnDelay) > 0;
  if ( is_cold_outside && heat_pads_not_too_hot && heat_timer_not_expired && heat_delay_has_passed )
  {
    // Turn heaters on
    digitalWrite(HEAT_OUT, HIGH);
    lastHeatOnTime = millis();
    heatOnDelayTmrOneshot = false;
  }
  else // turn heater off
  {
    digitalWrite(HEAT_OUT, LOW); 
    // Only want heatOnDelay timer to be set once
    if (heatOnDelayTmrOneshot == false)
    { 
      heatOnDelay = millis() + 30000L;  // 30 second delay before heater can come back on again
      heatOnDelayTmrOneshot = true;
     }
  }

  // Turn on heater every hour to see if there is snow to melt
  static uint32_t hourlyCheckTimer = 0;   // used to keep heater on for a few minuts every hour
  if((long)(millis() -  lastHeatOnTime ) >= (ONEMIN * 60L) && hourlyCheckFlag == false )
  {
    hourlyCheckFlag = true;
    hourlyCheckTimer = millis();
  }

  // After 15 minutes, reset hourly check flag
  if((long)(millis() - hourlyCheckTimer ) >= (ONEMIN * 15L) && hourlyCheckFlag == true )
  { hourlyCheckFlag = false; }   

  // Reset rain pulse counter after 10 hours of no pulses
  static uint16_t pulseCount =  0;   // Pulse count, reset after 10 hours of no pulses
  if( (long)( millis() - lastPulseTime ) >= (ONEMIN * 60UL * 10UL)) 
  { pulseCount = 0; }

  if ( g_gotRainPulse )
  {
    pulseCount++; 
    lastPulseTime = millis();

    // Turn on pulse out relay for 1/2 second
    digitalWrite(PULSE_OUT, HIGH);
    delay(500);
    digitalWrite(PULSE_OUT, LOW);
    
    g_gotRainPulse = false;  // reset 
  }

  static uint32_t sendDataTimer = millis();
  if( (long)(millis() - sendDataTimer) > 0 ) 
  {  
    CCPACKET xively;
    uint8_t k = 0;
    
    xively.length = 13;  // # bytes that make up packet to transmit
    xively.data[k++] = g_receiverAddress;  // Address of panStamp Receiver we are sending too. THIS IS REQUIRED BY THE CC1101 LIBRARY
    xively.data[k++] = g_senderAddress;    // Address of this panStamp Tx
    
    xively.data[k++] = digitalRead(HEAT_OUT);  // Heater on state - boolean
    xively.data[k++] = pulseCount >> 8 & 0xff;        
    xively.data[k++] = pulseCount & 0xff;             
  
    // Put temperatures in data array
    xively.data[k++] = tempOut >> 8 & 0xff;        
    xively.data[k++] = tempOut & 0xff;             
    xively.data[k++] = tempIn >> 8 & 0xff;         
    xively.data[k++] = tempIn & 0xff;              
    xively.data[k++] = tempHot1 >> 8 & 0xff;        
    xively.data[k++] = tempHot1 & 0xff;             
    xively.data[k++] = tempHot2 >> 8 & 0xff;        
    xively.data[k++] = tempHot2 & 0xff;             
   
    bool sentStatus = radio.sendData(xively);
    sendDataTimer = millis() + 500;  // timer set to send data again in 1/2 second
    
    #ifdef PRINT_DEBUG
      if(sentStatus)
      { Serial.println("sent data success"); } // This does not mean the receiver successfully received the data
      else
      { Serial.println("sent data failed "); }  
    #endif
  } // end send data
  
}  // end loop()


// return temp (F) for 10k thermistor 
float thermistorTempF(int RawADC) 
{
  float Temp;
  Temp = log(((10240000/RawADC) - 10000));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
  Temp = Temp - 273.15;            // Convert Kelvin to Celsius
  Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celsius to Fahrenheit
  return Temp;
} // thermistorTempF()


void printpanStampConfig()
{
  // Print device setup info
  Serial.print("Radio Frequency = ");
  if(radio.carrierFreq == CFREQ_868)
  {Serial.println("868 Mhz");}
  else
  {Serial.println("915 Mhz");}
  Serial.print("Channel = ");
  Serial.println(radio.channel);
  Serial.print("Network address = ");
  Serial.println(radio.syncWord[0]);
  Serial.print("Device address = ");
  Serial.println(radio.devAddress);
  
}  // printpanStampConfig()


// Interrupt routine, executed when pin D3 goes from HIGH to LOW
void rainPulse()
{
  g_gotRainPulse = true; 
}
