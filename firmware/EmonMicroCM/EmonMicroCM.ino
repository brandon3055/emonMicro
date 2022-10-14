/*
  emonMicro Continuous Sampling
  using EmonLibCM https://github.com/openenergymonitor/EmonLibCM
  Author: brandon3055
  
  -----------------------------------------
  This firmware is based on EmonTxV4CM
  Compatible with openenergymonitor.org project
  Licence: GNU GPL V3

  -----------------------------------------
  Note on uploading.
  Most of the instructions for the EmonTxV4 apply here.
  But due to the use of opto-isolation the communication baud rate is limited to 9600.
  Programming with this baud rate requires a modified version of optiboot that is included in the firmware folder.
  
  This also means uploading directly from the arduino IDE does not work. There is probably a solution to this issue.
  But the simple hack i use is as follows. (This is for linux but it will most likely also apply to windows)
  
  First go to arduino preferences and ensure "Show verbose output" is enabled for upload.
  Next try to uoload the sketch as you would normally. 
  This will fail but the sketch will be compiled and the command used to upload the compiled sketch will be listed in the console output.
  It will look something like this: 
  /home/brandon3055/.arduino15/packages/DxCore/tools/avrdude/6.3.0-arduino17or18/bin/avrdude -C/home/brandon3055/.arduino15/packages/DxCore/hardware/megaavr/1.4.10/avrdude.conf -v -pavr128db48 -carduino -D -P/dev/ttyUSB0 -b115200 -Uflash:w:/tmp/arduino_build_334173/EmonMicroCM.ino.hex:i 
  Now just copy that command, change -b115200 to -b9600 and run the command yourself in a terminal.
  The upload should complete successfully. 
*/

/*
Change Log:
v1.0: First release for EmonMicro.

TODO: Get serial config working

emonhub.conf
See: https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md
copy the following in to emonhub.conf:

Defauld node id is 24, 
The three switches are read as a 3 bit binary number (0 to 7)
This value is addes to the node id.
0,0,0 = 0      0,1,1 = 3      1,1,0 = 6
0,0,1 = 1      1,0,0 = 4      1,1,1 = 7
0,1,0 = 2      1,0,1 = 5

[[24]]
    nodename = emonMicro24
    [[[rx]]]
        names = MSG, Vrms, P1, P2, P3, P4, E1, E2, E3, E4
        datacodes = L,h,h,h,h,h,l,l,l,l
        scales = 1,0.01,1,1,1,1,1,1,1,1
        units = n,V,W,W,W,W,Wh,Wh,Wh,Wh
        whitening = 1

*/

#define Serial Serial3
#include <Arduino.h>

const byte version = 11; 

// Comment/Uncomment as applicable
#define DEBUG                                           // Debug level print out
#define SHOW_CAL                                        // Uncomment to show current for calibration

#define RFM69CW
#define RFMSELPIN PIN_PB5                               // RFM pins
//#define RFPWR 0x99                                    // RFM Power setting - see rfm.ino for more
#define RFPWR 0x9F
#define FACTORYTESTGROUP 1                              // R.F. group for factory test only
#include "emonLibCM.h"

#include <Wire.h>                                       // Required for RFM & temperature measurement
#include <SPI.h>
#include <util/crc16.h>
#include <OneWire.h>

enum rfband {RF12_433MHZ = 1, RF12_868MHZ, RF12_915MHZ }; // frequency band.

byte RF_freq = RF12_433MHZ;                             // Frequency of radio module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. 
byte nodeID = 24;                                       // node ID for this emonMicro.
int networkGroup = 210;                                 // wireless network group, needs to be same as emonBase / emonPi and emonGLCD. OEM default is 210
const int busyThreshold = -97;                          // Signal level below which the radio channel is clear to transmit
const byte busyTimeout = 15;                            // Time in ms to wait for the channel to become clear, before transmitting anyway
int rf_whitening = 2;                                   // RF & data whitening - 0 = no RF, 1 = RF on, no whitening, default = 2: RF is ON with whitening.

typedef struct {
    unsigned long Msg;
    int Vrms,P1,P2,P3,P4; 
    long E1,E2,E3,E4;
} PayloadTX;
PayloadTX emontx;                                       // create an instance


//----------------------------emonMicro V1 Settings - Shared with config.ino------------------------
#define PHASECAL 1.5

//5mR shunt, 6 x gain
float i1Cal = 33.333333;                 // 1 / (0.005 * 6) = 33.3333
float i1Lead = PHASECAL;
float i2Cal = 33.333333;         
float i2Lead = PHASECAL;
float i3Cal = 33.333333;         
float i3Lead = PHASECAL; 
float i4Cal = 33.333333;         
float i4Lead = PHASECAL;

float vCal  = 1023;          // (4 x 511) = 2044, (2044 + 2) / 2 = 1023
const float vCal_USA = 1023; // Will be the same

bool  USA=false;
float assumedVrms2 = 240.0;  // voltage to use for calculating assumed apparent power if a.c input is absent.
float period = 9.94;        // datalogging period

//----------------------------emonMicro V1 hard-wired connections-----------------------------------
const byte LEDpin      = PIN_PB2;  // emonMicro V1 LED
const byte DIP_switch1 = PIN_PA5;  
const byte DIP_switch2 = PIN_PA6;  
const byte DIP_switch3 = PIN_PA7;  
const byte RFM_reset   = PIN_PB4; 

//---------------------------------CS availability status----------------------------------------

byte CS_count = 0;
bool CS1, CS2, CS3, CS4; // Record if shunt present during startup

//----------------------------------------Setup--------------------------------------------------


void setup() {
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin,HIGH);
  
  pinMode(DIP_switch1, INPUT_PULLUP);
  pinMode(DIP_switch2, INPUT_PULLUP);
  pinMode(DIP_switch3, INPUT_PULLUP);

  int channelOffset = (!digitalRead(DIP_switch1) << 2) | (!digitalRead(DIP_switch2) << 1) | !digitalRead(DIP_switch3);
  nodeID += channelOffset;
  //Add a slight additional offset based on channel offset.
  //The down side here is the transmitters will occasionally sync up and step on each other.
  //But it ensures senders will never be able to end up perminently in sync.
//  period -= (channelOffset * 0.02); //(Moved to after config load)

  // Enabled Channel Detection---------------------------------------------------------------------------------

//  pinMode(PIN_PD1, INPUT_PULLUP);
//  pinMode(PIN_PD2, INPUT_PULLUP);
//  pinMode(PIN_PD3, INPUT_PULLUP);
//  pinMode(PIN_PD4, INPUT_PULLUP);
//  CS1 = !digitalRead(PIN_PD1);
//  CS2 = !digitalRead(PIN_PD2);
//  CS3 = !digitalRead(PIN_PD3);
//  CS4 = !digitalRead(PIN_PD4);
//  pinMode(PIN_PD1, INPUT);
//  pinMode(PIN_PD2, INPUT);
//  pinMode(PIN_PD3, INPUT);
//  pinMode(PIN_PD4, INPUT);
//
//  CS_count = CS1 + CS2 + CS3 + CS4;
  
  // Serial---------------------------------------------------------------------------------
  Serial.begin(9600);
  
  #ifdef DEBUG
    Serial.print(F("emonMicro V1 EmonLibCM Continuous Monitoring V")); Serial.println(version*0.1);
    Serial.println(F("OpenEnergyMonitor.org"));
    Serial.print(F("NodeID Offset: ")); Serial.println(channelOffset);
//    Serial.print(F("Detected Channels: ")); Serial.println(CS_count);
  #else
    Serial.println(F("describe:EmonMicroCM"));
  #endif

  load_config(true);
  period -= (channelOffset * 0.02);  

  // Reset RFM module ---------------------------------------------------------------------------------
  
  // The 3.3v rail rises a little slow on powerup. This does not seem to cause any issues for the AVR
  // but it does occasionally cause the RFM module to lock up on the first rfm_send call.
  // Resetting the RFM module after powerup is a simple way to resolve this issue. 
  pinMode(RFM_reset, OUTPUT);
  digitalWrite(RFM_reset, HIGH);
  delay(50);
  digitalWrite(RFM_reset, LOW);
  pinMode(RFM_reset, INPUT);
  delay(50);

  //On my v1.1 boards i added a bodge wire from one of the unused crystal pins to reset the RFM module
  //This is not needed with the latest board revision.
//  pinMode(PIN_PA0, OUTPUT);
//  digitalWrite(PIN_PA0, HIGH);
//  delay(50);
//  digitalWrite(PIN_PA0, LOW);
//  delay(50);
  
  
  pinMode(RFMSELPIN, INPUT_PULLUP);
    if (digitalRead(RFMSELPIN)==LOW) {
    rf_whitening = 0;
    Serial.println(F("RFM Select pin LOW, ESP32 Taking control of RFM69"));
  } else {
    // Serial.println(F("RFM Select pin HIGH"));
  }
  
  if (rf_whitening)
  {
    #ifdef DEBUG
      Serial.print(F("RFM69CW only"));
      Serial.print(F(" Node: ")); Serial.print(nodeID);
      Serial.print(" Freq: ");
      if (RF_freq == RF12_433MHZ) Serial.print(F("433MHz"));
      if (RF_freq == RF12_868MHZ) Serial.print(F("868MHz"));
      if (RF_freq == RF12_915MHZ) Serial.print(F("915MHz"));
      Serial.print(F(" Group: ")); Serial.println(networkGroup);
      Serial.println(" ");
    #endif
  }

  digitalWrite(LEDpin,LOW);

  readConfigInput();

  if (rf_whitening)
  {
    #ifdef DEBUG
      Serial.println(F("Pre Init RF"));
    #endif
    rfm_init(RF_freq);     
    #ifdef DEBUG
      Serial.println(F("RF Init Finished"));
    #endif
    // initialize RFM
    for (int i=10; i>=0; i--)                                             // Send RF test sequence (for factory testing)
    {
      emontx.P1=i;
      PayloadTX tmp = emontx;
      if (rf_whitening == 2)
      {
          byte WHITENING = 0x55;
          for (byte i = 0, *p = (byte *)&tmp; i < sizeof tmp; i++, p++)
              *p ^= (byte)WHITENING;
      }
//      #ifdef DEBUG
//        Serial.println(F("rfs"));
//      #endif
      rfm_send((byte *)&tmp, sizeof(tmp), FACTORYTESTGROUP, nodeID, busyThreshold, busyTimeout);
      delay(100);
    }
    emontx.P1=0;
  }

//  Serial.println(F("Ini send complete"));

  // ---------------------------------------------------------------------------------------
  

  // ----------------------------------------------------------------------------
  // EmonLibCM config
  // ----------------------------------------------------------------------------
  // 12 bit ADC = 4096 divisions
  // Time in microseconds for one ADC conversion: 40 us 
  EmonLibCM_setADC(12,29.5);

  // Using AVR-DB 1.024V internal voltage reference
  EmonLibCM_ADCCal(1.024);
  
  EmonLibCM_SetADC_VChannel(0, vCal);                      // ADC Input channel, voltage calibration
  if (USA) EmonLibCM_SetADC_VChannel(0, vCal_USA);
  EmonLibCM_SetADC_IChannel(1, i1Cal, i1Lead);             // ADC Input channel, current calibration, phase calibration
  EmonLibCM_SetADC_IChannel(2, i2Cal, i2Lead);
  EmonLibCM_SetADC_IChannel(3, i3Cal, i3Lead);
  EmonLibCM_SetADC_IChannel(4, i4Cal, i4Lead);

  // mains frequency 50Hz
  if (USA) EmonLibCM_cycles_per_second(60);                // mains frequency 60Hz
  EmonLibCM_datalog_period(period);                        // period of readings in seconds - normal value for emoncms.org  
  
  EmonLibCM_Init();                                        // Start continuous monitoring.
  emontx.Msg = 0;

  #ifdef DEBUG
    Serial.println(F("Setup Complete"));
  #endif
}

void loop() {
  static double E1 = 0.0, E2 = 0.0, E3 = 0.0, E4 = 0.0;    // Sketch's own value to use when a.c. fails.
  getCalibration();
  
  if (EmonLibCM_Ready())   
  {
    #ifdef DEBUG
    if (emontx.Msg==0) {
      Serial.println(EmonLibCM_acPresent()?F("AC present "):F("AC missing "));
      delay(5);
    }
    #endif

    emontx.Msg++;

    // Other options calculated by EmonLibCM
    // RMS Current:    EmonLibCM_getIrms(ch)
    // Apparent Power: EmonLibCM_getApparentPower(ch)
    // Power Factor:   EmonLibCM_getPF(ch)

    if (EmonLibCM_acPresent())
    {
      emontx.P1 = EmonLibCM_getRealPower(0); 
      emontx.E1 = EmonLibCM_getWattHour(0); 
      E1        = EmonLibCM_getWattHour(0);

      emontx.P2 = EmonLibCM_getRealPower(1); 
      emontx.E2 = EmonLibCM_getWattHour(1); 
      E2        = EmonLibCM_getWattHour(1);
      
      emontx.P3 = EmonLibCM_getRealPower(2); 
      emontx.E3 = EmonLibCM_getWattHour(2); 
      E3        = EmonLibCM_getWattHour(2);
    
      emontx.P4 = EmonLibCM_getRealPower(3); 
      emontx.E4 = EmonLibCM_getWattHour(3); 
      E4        = EmonLibCM_getWattHour(3);
    }
    else
    {
      emontx.P1 = assumedVrms2 * EmonLibCM_getIrms(0);       // Alternative calculations for estimated power & energy  
      emontx.P2 = assumedVrms2 * EmonLibCM_getIrms(1);       //   when no a.c. voltage is available
      emontx.P3 = assumedVrms2 * EmonLibCM_getIrms(2);       // Alternative calculations for estimated power & energy  
      emontx.P4 = assumedVrms2 * EmonLibCM_getIrms(3);       //   when no a.c. voltage is available
      
      E1       += assumedVrms2 * EmonLibCM_getIrms(0) * EmonLibCM_getDatalog_period()/3600.0;
      emontx.E1 = E1 + 0.5;                                        // rounded value
      E2       += assumedVrms2 * EmonLibCM_getIrms(1) * EmonLibCM_getDatalog_period()/3600.0;
      emontx.E2 = E2 + 0.5;                                        // rounded value
      E3       += assumedVrms2 * EmonLibCM_getIrms(2) * EmonLibCM_getDatalog_period()/3600.0;
      emontx.E3 = E3 + 0.5;                                        // rounded value
      E4       += assumedVrms2 * EmonLibCM_getIrms(3) * EmonLibCM_getDatalog_period()/3600.0;
      emontx.E4 = E4 + 0.5;                                        // rounded value
    }
    emontx.Vrms = EmonLibCM_getVrms() * 100;
  
    if (rf_whitening)
    {
//      Serial.println(F("About to Send"));
      PayloadTX tmp = emontx;
      if (rf_whitening == 2)
      {
          byte WHITENING = 0x55;
          for (byte i = 0, *p = (byte *)&tmp; i < sizeof tmp; i++, p++)
              *p ^= (byte)WHITENING;
      }
      rfm_send((byte *)&tmp, sizeof(tmp), networkGroup, nodeID, busyThreshold, busyTimeout);     //send data
      delay(50);
    }

    // ---------------------------------------------------------------------
    // Key:Value format, used by EmonESP & emonhub EmonHubTx3eInterfacer
    // ---------------------------------------------------------------------
    Serial.print(F("MSG:")); Serial.print(emontx.Msg);
    Serial.print(F(",Vrms:")); Serial.print(emontx.Vrms*0.01);
    
    Serial.print(F(",P1:")); Serial.print(emontx.P1);
    Serial.print(F(",P2:")); Serial.print(emontx.P2);
    Serial.print(F(",P3:")); Serial.print(emontx.P3);
    Serial.print(F(",P4:")); Serial.print(emontx.P4);
  
    Serial.print(F(",E1:")); Serial.print(emontx.E1);
    Serial.print(F(",E2:")); Serial.print(emontx.E2);
    Serial.print(F(",E3:")); Serial.print(emontx.E3);
    Serial.print(F(",E4:")); Serial.print(emontx.E4);
    
    delay(20);

    digitalWrite(LEDpin,HIGH); delay(50);digitalWrite(LEDpin,LOW);

//// ###################### Testing
//        Serial.println();
//      Serial.print(F("I1:")); Serial.print(EmonLibCM_getIrms(0),3);
//      Serial.print(F(",I2:")); Serial.print(EmonLibCM_getIrms(1),3);
//      Serial.print(F(",I3:")); Serial.print(EmonLibCM_getIrms(2),3);
//      Serial.print(F(",I4:")); Serial.print(EmonLibCM_getIrms(3),3);
//
//  Serial.println();
//      Serial.print(F("A1:")); Serial.print(analogRead(PIN_PD3));
//      Serial.print(F(",A2:")); Serial.print(analogRead(PIN_PD4));
//      Serial.print(F(",A3:")); Serial.print(analogRead(PIN_PD5));
//      Serial.print(F(",A4:")); Serial.print(analogRead(PIN_PD6));
//// ######################


    #ifdef SHOW_CAL
      // to show current & power factor for calibration:
      Serial.println();
      Serial.print(F(",I1:")); Serial.print(EmonLibCM_getIrms(0),3);
      Serial.print(F(",I2:")); Serial.print(EmonLibCM_getIrms(1),3);
      Serial.print(F(",I3:")); Serial.print(EmonLibCM_getIrms(2),3);
      Serial.print(F(",I4:")); Serial.print(EmonLibCM_getIrms(3),3);

      Serial.print(F(",pf1:")); Serial.print(EmonLibCM_getPF(0),4);
      Serial.print(F(",pf2:")); Serial.print(EmonLibCM_getPF(1),4);
      Serial.print(F(",pf3:")); Serial.print(EmonLibCM_getPF(2),4);
      Serial.print(F(",pf4:")); Serial.print(EmonLibCM_getPF(3),4);

    #endif
    Serial.println();
    
    // End of print out ----------------------------------------------------
  }
  //wdt_reset();
  delay(20);
}
