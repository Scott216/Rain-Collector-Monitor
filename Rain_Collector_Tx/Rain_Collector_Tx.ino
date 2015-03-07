/*
Rain collector heater 
Board: panStamp AVR  http://www.panstamp.com/product/panstamp-avr/
IDE Settings 1.6.x: 
Board: panStamp AVR

Xively Feed: http://xively.com/feeds/103470/

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
09/12/14 v0.10 - Added version number
10/04/14 v0.11 - Use interrupt for pulse in. Moved global variables to loop() and made static.  Renamed Thermistor() to thermistorTempF().  
                 Only send data every 1/2 second. 
10/17/14 v0.12 - Changed networks address (syncword)to two byte array
10/18/14 v0.13 - Compiled with new version of panStamp libraries v2
12/25/14 v0.14 - Added setTxPowerAmp(). Uploaded to rain collector on 1/12/15
02/10/15 v0.15 - When outside temp is < 20, heater now stays on for 40 minutes, if > 20F, it's on for 15 minuts.
                 Also, renamed some variables and constants.
02/28/15 v2.00 - Upgraded code to use latest panStamp API.  Now both TX and Rx are up to date.  
03/03/15 v2.01 - Added RSSI Moved CCPACKET wirelessPacket to global area.  
*/

#define VERSION "v2.01"
#define PRINT_DEBUG     // comment this out to turn off serial printing

#include <Arduino.h>
#include <HardwareSerial.h> // Needed by Arduino IDE 1.6.x
#include <avr/wdt.h>        // http://github.com/simonz05/arduino-headers/blob/master/wdt.h

// Analog pins
const int8_t   PIN_TEMP_HOT1 =     0;  // Temp on heating pad 1
const int8_t   PIN_TEMP_HOT2 =     1;  // Temp on heating pad 2
const int8_t   PIN_TEMP_PCB =      2;  // Temp on PCB
const int8_t   TEMP_OUT =          3;  // Temp outside under rain collector
// Digital pins
const int8_t   PIN_PULSE_IN =      3;  // Rain pulse in
const int8_t   PIN_PULSE_OUT =     4;  // Rain pulse out to ISS on roof
const int8_t   PIN_HEATER_OUTPUT = 5;  // Relay to turn heaters on

const uint32_t ONEMIN =      60000UL;  // 1 minute in mS

volatile bool g_gotRainPulse = false;  // set by rainPulse() interrupt when rain bucket tips


// panStamp config
const byte g_RF_Channel =               0;  // panStamp channel
      byte g_psNetworkAddress[] = {10, 0};  // panStamp network address, aka SyncWord.  Tx and Rx must have same network address
const byte g_psSenderAddress =         41;  // panStamp outside Tx address
const byte g_psReceiverAddress =       40;  // panStamp inside Rx address

CCPACKET wirelessPacket;  // packet array of data to send to other indoor panStamp

// Function Prototypes
float thermistorTempF(int RawADC);
void rainPulse();  // Interrupt
void printPanstampDeviceInfo();

void setup()
{
  #ifdef PRINT_DEBUG
    Serial.begin(9600);
    delay(3000);
    Serial.print("Rain Collector Tx\nversion ");
    Serial.println(VERSION);
  #endif
  
  pinMode(PIN_PULSE_IN,         INPUT);
  pinMode(PIN_PULSE_OUT,       OUTPUT);
  pinMode(PIN_HEATER_OUTPUT,   OUTPUT);
  digitalWrite(PIN_PULSE_OUT,     LOW);
  digitalWrite(PIN_HEATER_OUTPUT, LOW);
  
  attachInterrupt(1, rainPulse, FALLING);  // Pin D3 interrupt for rain pulse

  // initialize panStamp radio
  panstamp.radio.setChannel(g_RF_Channel);
  panstamp.radio.setSyncWord(g_psNetworkAddress);  // Set network address, pointer to address
  panstamp.radio.setDevAddress(g_psSenderAddress);
  panstamp.radio.setTxPowerAmp(PA_LongDistance);  // Turns on high power mode. PA_LowPower is the default 
  
  #ifdef PRINT_DEBUG
    printPanstampDeviceInfo();  // print panStamp config
  #endif
  wdt_enable (WDTO_8S);  // 8 second timeout for WDT
} // end setup()


void loop()
{
  wdt_reset();  
  
  const uint32_t HEATER_ON_TIME =  30UL * ONEMIN; // Time heater stays on after last pulse detected (if it's cold outside)
  const uint8_t  HIGH_TEMP_LIMIT = 175;  // high temperature limit (F) for heaters

  static int16_t temp_PCB;
  static int16_t temp_Outside;
  static int16_t temp_Heater1;
  static int16_t temp_Heater2; 
  const double FILTERVAL = 0.01;  // used to smooth (average) temperature readings
  // Read therister temps, convert to F and smooth with low pass filter
  temp_PCB =      (thermistorTempF(analogRead(PIN_TEMP_PCB))  * (1-FILTERVAL)) + (FILTERVAL *     temp_PCB );
  temp_Outside =  (thermistorTempF(analogRead(TEMP_OUT))      * (1-FILTERVAL)) + (FILTERVAL * temp_Outside );
  temp_Heater1 =  (thermistorTempF(analogRead(PIN_TEMP_HOT1)) * (1-FILTERVAL)) + (FILTERVAL * temp_Heater1 );
  temp_Heater2 =  (thermistorTempF(analogRead(PIN_TEMP_HOT2)) * (1-FILTERVAL)) + (FILTERVAL * temp_Heater2 );
 
  static uint32_t heatOnDelay =    0;     // Delay so heater doesn't come back on too soon
  static uint32_t lastHeatOnTime = 0;     // Used to turn the heater on everyhour to see if any pulses start from melted snow
  static uint32_t lastPulseTime =  0;     // Used to see how long since the last pulse.  If heater is on an there are no pulses, then turn it off
  static bool     heatOnDelayTmrOneshot;  // Flag used with heater on delay timer
  static bool     hourlyCheckFlag;        // True if heater should come on for an hourly check to see if there is snow to melt.  
                                          // Heater should stay on for HEATER_ON_TIME 
  // See if heater should turn on
  bool is_cold_outside = temp_Outside < 40;
  bool inside_temp_not_too_hot = temp_PCB < 80;
  bool heat_pads_not_too_hot = (temp_Heater1 < HIGH_TEMP_LIMIT) && (temp_Heater2 < HIGH_TEMP_LIMIT);
  bool heat_timer_not_expired =  (((long)(millis() - lastPulseTime) < HEATER_ON_TIME) || hourlyCheckFlag == true);  
  bool heat_delay_has_passed = (long)(millis() - heatOnDelay) > 0;
  if ( is_cold_outside && heat_pads_not_too_hot && inside_temp_not_too_hot && heat_timer_not_expired && heat_delay_has_passed )
  {
    // Turn heaters on
    digitalWrite(PIN_HEATER_OUTPUT, HIGH);
    lastHeatOnTime = millis();
    heatOnDelayTmrOneshot = false;
  }
  else // turn heater off
  {
    digitalWrite(PIN_HEATER_OUTPUT, LOW); 
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

  // Reset hourly check flag.  This is where heater turns on once an hour to see if any snow melts.
  // If outside temp > 20F then reset after 15 minutes. If temp is < 20F, reset flag after 40 minutes
  uint32_t heater_on_time;
  if ( temp_Outside > 20 )
  { heater_on_time = ONEMIN * 15L; }
  else
  { heater_on_time = ONEMIN * 40L; }
  
  if( ((long)(millis() - hourlyCheckTimer ) >= heater_on_time ) && (hourlyCheckFlag == true) )
  { hourlyCheckFlag = false; }   

  // Reset rain pulse counter after 10 hours of no pulses
  static uint16_t pulseCount =  0;   // Pulse count, reset after 10 hours of no pulses
  const uint32_t TEN_HOURS = ONEMIN * 60UL * 10UL;
  if( (long)( millis() - lastPulseTime ) >= TEN_HOURS ) 
  { pulseCount = 0; }

  if ( g_gotRainPulse )
  {
    pulseCount++; 
    lastPulseTime = millis();

    // Turn on pulse out relay for 1/2 second
    digitalWrite(PIN_PULSE_OUT, HIGH);
    delay(500);
    digitalWrite(PIN_PULSE_OUT, LOW);
    
    g_gotRainPulse = false;  // reset 
  }

  // send data to inside panstamp
  static uint32_t sendDataTimer = millis();
  if( (long)(millis() - sendDataTimer) > 0 ) 
  {  
    
    wirelessPacket.length = 13;  // # bytes that make up packet to transmit
    uint8_t k = 0;
    wirelessPacket.data[k++] = g_psReceiverAddress;  // Address of panStamp Receiver that data is sent too (inside). First data byte has to be the destination address
    wirelessPacket.data[k++] = g_psSenderAddress;    // Address of this panStamp tranmsitter (outside)
    wirelessPacket.data[k++] = digitalRead(PIN_HEATER_OUTPUT);  // Heater on/off state 
    wirelessPacket.data[k++] = pulseCount >> 8 & 0xff;        
    wirelessPacket.data[k++] = pulseCount & 0xff;             
    wirelessPacket.data[k++] = temp_Outside >> 8 & 0xff;        
    wirelessPacket.data[k++] = temp_Outside & 0xff;             
    wirelessPacket.data[k++] = temp_PCB >> 8 & 0xff;         
    wirelessPacket.data[k++] = temp_PCB & 0xff;              
    wirelessPacket.data[k++] = temp_Heater1 >> 8 & 0xff;        
    wirelessPacket.data[k++] = temp_Heater1 & 0xff;             
    wirelessPacket.data[k++] = temp_Heater2 >> 8 & 0xff;        
    wirelessPacket.data[k++] = temp_Heater2 & 0xff;             
   
    bool sentStatus = panstamp.radio.sendData(wirelessPacket);  // transmit the data
    sendDataTimer = millis() + 500;  // add 500 mS to timer
    
    #ifdef PRINT_DEBUG
      if(sentStatus)
      {  // This does not mean the receiver successfully received the data
        Serial.print("Sent data success"); 
      } 
      else
      { Serial.println("Sent data failed"); }  
    #endif
  } // end send data
  
}  // end loop()


// return temp (F) for 10k thermistor 
float thermistorTempF(int RawADC) 
{
  float thermister;
  thermister = log(((10240000/RawADC) - 10000));
  thermister = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * thermister * thermister )) * thermister );
  thermister = thermister - 273.15;            // Convert Kelvin to Celsius
  thermister = (thermister * 9.0)/ 5.0 + 32.0; // Convert Celsius to Fahrenheit
  return thermister;
}  // end thermistorTempF()


void printPanstampDeviceInfo()
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
  
}  // end printPanstampDeviceInfo()


// Interrupt routine, executed when pin D3 goes from HIGH to LOW
void rainPulse()
{
  g_gotRainPulse = true; 
}  // end rainPulse()
