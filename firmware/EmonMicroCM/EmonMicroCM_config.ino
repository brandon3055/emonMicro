/*
 
Configuration file for EmonMicro.ino - V2.3 

EEPROM layout

Byte
0     NodeID
1     RF Freq
2     Network Group
3-6   vCal

7-10  i1Cal
11-14 i1Lead
15-18 i2Cal
19-22 i2Lead
23-26 i3Cal
27-30 i3Lead
31-34 i4Cal
35-38 i4Lead

51-54  Datalogging period
55     Pulses enabled
56-57  Pulse min period
58     Temperatures enabled
59-106 Temperature Addresses (6×8)
107    Data whitening
108-111 assumedVrms

*/
#define Serial Serial3
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>

 // Available Serial Commands
const PROGMEM char helpText1[] =                                
"\n"
"Available commands for config during start-up:\n"
"  b<n>      - set r.f. band n = a single numeral: 4 = 433MHz, 8 = 868MHz, 9 = 915MHz (may require hardware change)\n"
"  g<nnn>    - set Network Group  nnn - an integer (OEM default = 210)\n"
"  i<nn>     - set node ID i= an integer (standard node ids are 1..31)\n"
"  r         - restore sketch defaults\n"
"  s         - save config to EEPROM\n"
"  v         - show firmware version\n"
"  w<x>      - turn RFM Wireless data on or off:\n"
"            - x = 0 for OFF, x = 1 for ON, x = 2 for ON with whitening\n"
"  x         - exit and continue\n"
"  ?         - show this text again\n"
"\n"
"Available commands only when running:\n"
"  k<x> <yy.y> <zz.z>\n"
"            - Calibrate an analogue input channel:\n"
"            - x = a single numeral: 0 = voltage calibration, 1 = ct1 calibration, 2 = ct2 calibration, etc\n"
"            - yy.y = a floating point number for the voltage/current calibration constant\n"
"            - zz.z = a floating point number for the phase calibration for this c.t. (z is not needed, or ignored if supplied, when x = 0)\n"
"            -  e.g. k0 256.8\n"
"            -       k1 90.9 2.00\n"
"  a<xx.x>   - xx.x = a floating point number for the assumed voltage if no a.c. is detected\n"
"  l         - list the config values\n"
"  p<xx.x>   - xx.x = a floating point number for the datalogging period\n" 
"  s         - save config to EEPROM\n"
"  ?         - show this text again\n"
;

struct eeprom {byte nodeID, RF_freq, networkGroup; 
      float vCal, i1Cal, i1Lead, i2Cal, i2Lead, i3Cal, i3Lead, i4Cal, i4Lead, period; 
      int rf_whitening; float assumedVrms;
      } data;

extern DeviceAddress *temperatureSensors;

static void load_config(bool verbose)
{
  byte* src = (byte *)&data;
  
  byte flag=0;
  bool dataPresent = false;
  for (byte j=0; j<sizeof(data); j++) {
    // Check each EEPROM memory location for data, 255 = no data present
    // If data present then increment flag
    if ((EEPROM.read(j) != 255)) flag ++;
    // Serial.print(EEPROM.read(j));
    // Serial.print(" ");
    // Serial.println(flag);

    // Check if EEPROM data matches the expected sizeof data, if not ignore config since this could be for emonTx V3
    if (flag == sizeof(data)) dataPresent = true;
  }
 
  if (dataPresent==true)
  {
    for (byte j=0; j<sizeof(data); j++, src++)
          *src = EEPROM.read(j); 

    // check validity of EEPROM config
    byte invalid_config = 0;
    // rfm node id should be 1..30
    if (data.nodeID>=1 && data.nodeID<=32) nodeID = data.nodeID; else {
      invalid_config++;
//      Serial.println(F("nodeID Invalid"));
    }
    // rfm frequency should be either 433, 868 or 915
    if (data.RF_freq==RF12_433MHZ || data.RF_freq==RF12_868MHZ || data.RF_freq==RF12_915MHZ) RF_freq = data.RF_freq; else {
      invalid_config++;
//      Serial.println(F("RF_freq Invalid"));
    }
    // rfm network group should be 1..212
    if (data.networkGroup>=1 && data.networkGroup<=212) networkGroup = data.networkGroup; else {
      invalid_config++;
//      Serial.println(F("networkGroup Invalid"));
    }
    
    // Calibration values should not be 0 but are less critical
    // and so are copied here without validation
    vCal         = data.vCal;
    i1Cal        = data.i1Cal;
    i1Lead       = data.i1Lead;
    i2Cal        = data.i2Cal;
    i2Lead       = data.i2Lead;
    i3Cal        = data.i3Cal;
    i3Lead       = data.i3Lead; 
    i4Cal        = data.i4Cal; 
    i4Lead       = data.i4Lead;
    
    // data period must be greater than 0
    if (data.period>0) period = data.period; else {
      invalid_config++; 
//      Serial.println(F("period Invalid"));
    }
   
    rf_whitening = data.rf_whitening;
    assumedVrms2  = data.assumedVrms;
      
    if (invalid_config) {
        Serial.print(F("ERROR EEPROM config contains ")); Serial.print(invalid_config); Serial.println(F(" invalid values"));
        Serial.println(F("Consider restoring defaults and re-applying config."));
    }
  }   
  
  if (verbose)
  {
    if (dataPresent) {
      Serial.println(F("Loaded EEPROM config"));
    } else { 
      Serial.println(F("No EEPROM config"));
    }
    list_calibration();
  }    
}

static void showString (PGM_P s) {
  for (;;) {
    char c = pgm_read_byte(s++);
    if (c == 0)
      break;
    if (c == '\n')
      Serial.print('\r');
    Serial.print(c);
  }
}

static void list_calibration(void)
{
  
  Serial.println(F("Settings:"));
  Serial.print(F("Group ")); Serial.print(networkGroup);
  Serial.print(F(", Node ")); Serial.print(nodeID & 0x1F);
  Serial.print(F(", Band ")); 
  Serial.print(RF_freq == RF12_433MHZ ? 433 : 
               RF_freq == RF12_868MHZ ? 868 :
               RF_freq == RF12_915MHZ ? 915 : 0);
  Serial.print(F(" MHz\n\n"));
 
  Serial.println(F("Calibration:"));
  Serial.print(F("vCal = ")); Serial.println(vCal, 6);
  Serial.print(F("assumedV = ")); Serial.println(assumedVrms2);
  Serial.print(F("i1Cal = ")); Serial.println(i1Cal, 6);
  Serial.print(F("i1Lead = ")); Serial.println(i1Lead);
  Serial.print(F("i2Cal = ")); Serial.println(i2Cal, 6);
  Serial.print(F("i2Lead = ")); Serial.println(i2Lead);
  Serial.print(F("i3Cal = ")); Serial.println(i3Cal, 6);
  Serial.print(F("i3Lead = ")); Serial.println(i3Lead);
  Serial.print(F("i4Cal = ")); Serial.println(i4Cal, 6);
  Serial.print(F("i4Lead = ")); Serial.println(i4Lead);
  Serial.print(rf_whitening ? (rf_whitening ==1 ? "RF on":"RF whitened"):"RF off"); Serial.print("\n");
}

static void save_config()
{
  Serial.println("Saving...");

  //Save new settings
  byte* src = (byte*) &data;
  data.nodeID       = nodeID;  
  data.RF_freq      = RF_freq;
  data.networkGroup = networkGroup;
  data.vCal         = vCal;
  data.i1Cal        = i1Cal;
  data.i1Lead       = i1Lead;
  data.i2Cal        = i2Cal;
  data.i2Lead       = i2Lead;
  data.i3Cal        = i3Cal;
  data.i3Lead       = i3Lead; 
  data.i4Cal        = i4Cal; 
  data.i4Lead       = i4Lead;     
  data.period       = period; 
  data.rf_whitening = rf_whitening;
  data.assumedVrms  = assumedVrms2;

  for (byte j=0; j<sizeof(data); j++, src++)
    EEPROM[j] = *src;    

  for (byte j=0; j<sizeof(data); j++)
  {
    if (EEPROM[j] < 0x10)
      Serial.print('0');
    Serial.print(EEPROM[j],16);Serial.print(" ");
  }
  
  Serial.println(F("\n\nDone. New config saved to EEPROM"));
}

static void wipe_eeprom(void)
{
  Serial.println(F("Erasing EEPROM..."));
  
  for (byte j=0; j<sizeof(data); j++)
    EEPROM[j] = 255;
    
  Serial.println(F("Done. Sketch will now restart using default config."));
  delay(200);
}

void softReset(void)
{
  asm volatile ("  jmp 0");
}

void readConfigInput(void)
{
  Serial.println(F("POST.....wait 10s"));
  Serial.println(F("'+++' then [Enter] for config mode"));
    
  unsigned long start = millis();
  while (millis() < (start + 10000))
  {
    // If serial input of keyword string '+++' is entered during 10s POST then enter config mode
    if (Serial.available()) 
    {
      if ( Serial.readString() == "+++\r\n")
      {
        Serial.println(F("Entering config mode..."));
        showString(helpText1);
        while(config()) {
          //wdt_reset();
          delay(100);
        }
      }
    }
    //wdt_reset();
    delay(100);
  }
}



static bool config(void) 
{
  if (Serial.available())
  {
    char c = Serial.peek();
    switch (c) {
        case 'a':
          assumedVrms2 = Serial.parseFloat();
          Serial.print(F("Assumed V: "));Serial.println(assumedVrms2);
          break;
          
        case 'b':  // set band: 4 = 433, 8 = 868, 9 = 915
          RF_freq = bandToFreq(Serial.parseFloat());
          while (Serial.available())
            Serial.read(); 
          Serial.print(RF_freq == RF12_433MHZ ? 433 : 
                       RF_freq == RF12_868MHZ ? 868 :
                       RF_freq == RF12_915MHZ ? 915 : 0);
          Serial.println(F(" MHz"));
          break;

        case 'g':  // set network group
          networkGroup = Serial.parseFloat();
          while (Serial.available())
            Serial.read(); 
          Serial.print(F("Group ")); Serial.println(networkGroup);
          break;

        case 'i':  //  Set NodeID )range expected: 1 - 30
          nodeID = Serial.parseFloat();
          Serial.print(F("Node ")); Serial.println(nodeID & 0x1F);
          break;

        case 'l': // print the calibration values
          list_calibration();
          break;

        case 'r': // restore sketch defaults
          wipe_eeprom();
          softReset();
          break;

        case 's': // Save to EEPROM. ATMega328p has 1kB  EEPROM
          save_config();
          break;

        case 'v': // print firmware version
          Serial.print(F("emonMicro V1.0 + RFM69CW EmonLibCM Continuous Monitoring V")); Serial.println(version*0.1);
          break;
        
        case 'w' :  // RF Off / On / On & whitened
          /* Format expected: w[x]
           */
          rf_whitening = Serial.parseFloat(); 
          Serial.print(rf_whitening ? (rf_whitening ==1 ? "RF on":"RF whitened"):"RF off"); Serial.print("\n");
          break;
          
        case 'x':  // exit and continue
          Serial.print(F("\nContinuing...\n"));
          while (Serial.available())
            Serial.read(); 
          return false;

        case '?':  // show Help text        
          showString(helpText1);
          Serial.println(F(" "));
          break;
        
        default:
          break;
    } //end switch
    while (Serial.available())
      Serial.read(); 
  }   
  return true;

}



void getCalibration(void)
{
/*
 * Reads calibration information (if available) from the serial port. Data is expected in the format
 * 
 *  k[x] [y] [z]
 * 
 * where:
 *  [x] = a single numeral: 0 = voltage calibration, 1 = ct1 calibration, 2 = ct2 calibration, etc
 *  [y] = a floating point number for the voltage/current calibration constant
 *  [z] = a floating point number for the phase calibration for this c.t. (z is not needed, or ignored if supplied, when x = 0)
 * 
 * e.g. k0 256.8
 *      k1 90.9 1.7 
 * 
 * If power factor is not displayed, it is impossible to calibrate for phase errors,
 *  and the standard value of phase calibration MUST BE SENT when a current calibration is changed.
 * 
 */

  if (Serial.available())
  {   
    int k1; 
    double k2, k3; 
    char c = Serial.peek();
    switch (c) {
      case 'f':
        /*
        *  Format expected: f50 | f60
        */
        k1 = Serial.parseFloat();
        while (Serial.available())
          Serial.read(); 
        EmonLibCM_cycles_per_second(k1);
        Serial.print(F("Freq: "));Serial.println(k1);
        break;
      
      case 'k':
        /*  Format expected: k[x] [y] [z]
        * 
        * where:
        *  [x] = a single numeral: 0 = voltage calibration, 1 = ct1 calibration, 2 = ct2 calibration, etc
        *  [y] = a floating point number for the voltage/current calibration constant
        *  [z] = a floating point number for the phase calibration for this c.t. (z is not needed, or ignored if supplied, when x = 0)
        * 
        * e.g. k0 256.8
        *      k1 90.9 1.7 
        * 
        * If power factor is not displayed, it is impossible to calibrate for phase errors,
        *  and the standard value of phase calibration MUST BE SENT when a current calibration is changed.
        */
        k1 = Serial.parseFloat(); 
        k2 = Serial.parseFloat(); 
        k3 = Serial.parseFloat(); 
        while (Serial.available())
          Serial.read(); 
              
        // Write the values back as Globals, re-calculate intermediate values.
        switch (k1) {
          case 0 : EmonLibCM_ReCalibrate_VChannel(k2);
            vCal = k2;
            break;
              
          case 1 : EmonLibCM_ReCalibrate_IChannel(1, k2, k3);
            i1Cal = k2;
            i1Lead = k3;
            break;

          case 2 : EmonLibCM_ReCalibrate_IChannel(2, k2, k3);
            i2Cal = k2;
            i2Lead = k3;
            break;

          case 3 : EmonLibCM_ReCalibrate_IChannel(3, k2, k3);
            i3Cal = k2;
            i3Lead = k3;
            break;

          case 4 : EmonLibCM_ReCalibrate_IChannel(4, k2, k3);
            i4Cal = k2;
            i4Lead = k3;
            break;
                                 
          default : ;
        }
        Serial.print(F("Cal: k"));Serial.print(k1);Serial.print(F(" "));Serial.print(k2);Serial.print(F(" "));Serial.println(k3);        
        break;
          
      case 'l':
        list_calibration(); // print the calibration values
        printTemperatureSensorAddresses();        
      break;
          
      case 'p':
        /*  Format expected: p[x]
         * 
         * where:
         *  [x] = a floating point number for the datalogging period in s
         */
        k2 = Serial.parseFloat(); 
        EmonLibCM_datalog_period(k2); 
        period = k2;
        Serial.print(F("logging period: ")); Serial.print(k2);Serial.println(F(" s"));
        break;

      case 's' :
        save_config(); // Save to EEPROM. ATMega328p has 1kB  EEPROM
        break;

      case '?':  // show Help text        
        showString(helpText1);
        Serial.println(F(" "));
        break;

    }
    // flush the input buffer
    while (Serial.available())
      Serial.read();
  }
}


static byte bandToFreq (byte band) {
  return band == 4 ? RF12_433MHZ : band == 8 ? RF12_868MHZ : band == 9 ? RF12_915MHZ : 0;
}




byte c2h(byte b)
{
  if (b > 47 && b < 58) 
    return b - 48;
  else if (b > 64 && b < 71) 
    return b - 55;
  else if (b > 96 && b < 103) 
    return b - 87;
  return 0;
}
