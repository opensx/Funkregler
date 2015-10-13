/*****************************************************************
XBee_Throttle.ino

version for Teensy3.1, TeensyLC or ATmega328

for teensy's:
  XBee is connected to RX1 (pin2) and TX1 (pin3)

for avr/atmega328
  XBee is connected to RX (pin0) and TX (pin1)

SSD1306 display 128x64 - weil sie weniger RAM braucht als die 
ADAfruit lib, wird die U8glib benutzt.
genutzte Fonts: 
   u8g_font_fur42n
   u8g_font_fur20
   u8g_font_helvB12  

Rotary Encoder für Speed und Adress Selection
Buttons für Adress-Selection ("A"), Licht (F0="L") und Function (F1="F")

13.10.2015 "waiting_for_response" modus hinzugefügt
    (nach: neue Lok selektiert, vor: noch keine Antwort von der Zentrale)

12.10.2015 simplified, removed settings, protocol rev.1.1.

5.10.2015 using SXLoco instead of "general" Loco
   larger encoder rotation => increase/decr address by several steps
2.10.2015 distinction between 0-forward and 0-backward
   larger encoder rotation => increase/decr speed by several steps

31.8.2015 - SELECTRIX MODE
4.9.2015 - bidirectional communication of trackPower
(press RotEnc Button for longer than 1.3 secs to toggle power)
5.9.2015 - "Batt" is displayed when battery below 3300 mV
   (and battery state [10mV] is sent every 10mins to coord: "B 352" )

TODO Sleep einbauen

*****************************************************************/

#define HW_REV  "HW1.1" // or higher
#define SW_REV  "SW1.1.6 - 13 Oct 2015"

#define SELECTRIX_MODE    // use simple selectrix mode
//#define DCC_MODE       // use DCC mode - NOT YET IMPLEMENTED !
//#define DISP_ROTATED    // if defined, rotate display by 180' and
                          // exchange "A" and "F" buttons



//#define _TEENSY_31     // default is now ATmega328 !
//#define _TEENSY_LC
#define _DEBUG_AVR // if defined, the debut output is sent to the 
                   // AT-configured XBEE
// #define _DEBUG   // if debug => output to Serial Port ONLY FOR TEENSY !!

// Setup a RotaryEncoder
//#define ENCODER_DO_NOT_USE_INTERRUPTS
#define ENCODER_USE_INTERRUPTS

// operating modes, change by hitting the "A" button
#define MODE_NORMAL               0
#define MODE_ADDRESS_SELECTION    1 // short press of "A" button
#define MODE_WAITING_FOR_RESPONSE 2 // waiting for loco info from central


#include <EEPROM.h>
#include <XBee.h>
#include <U8glib.h>
#include <Bounce.h>
#include <Encoder.h>
#include "sxutils.h"  // includes debug settings
#include "SXLoco.h"
#include "AddrSelection.h"

// Setup a buttons
#if defined(_TEENSY_31) || defined(_TEENSY_LC)
#define ENC_PIN1   5
#define ENC_PIN2   6
#define ENC_BTN    7   // sets speed to zero
#define ADDR_BTN  19   // toogle between address selection and speed selection
#define F0_BTN    18
#define F1_BTN    17
#define PWR_DOWN  21   // to generate a power down interrupt
#define MEAS_VIN  A6
#define SLEEP_RQ 22
#else   // arduino pro mini
#define ENC_PIN1   3
#define ENC_PIN2   2  // =5 for HW 1.1
#define ENC_BTN    5  // =2 for HW 1.1 + switch on  // sets speed to zero
#ifndef DISP_ROTATED
#define ADDR_BTN  6    // toogle between address selection and speed selection
#define F0_BTN    7
#define F1_BTN    8
#else   // also rename 'A' 'L' 'F' buttons
#define ADDR_BTN  9    // toogle between address selection and speed selection
#define F0_BTN    7
#define F1_BTN    6
#endif  // DISP_ROTATED
#define PWR_DISP  A2   // power out for oled display
#define SLEEP_RQ  A1   // let xbee sleep
#define MEAS_VIN  A0   // measure battery voltage
#endif

Bounce encButton(ENC_BTN,100);
Bounce addrButton(ADDR_BTN, 100);
long addrButtonTimer = 0;
uint8_t addrButtonPressed = 0;
Bounce f0Button(F0_BTN, 100);
Bounce f1Button(F1_BTN, 100);


// using software SPI (the default case and default pins):
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
U8GLIB_SSD1306_128X64_2X u8g(OLED_CLK, OLED_MOSI, OLED_CS, OLED_DC ,OLED_RESET);

Encoder encoder(ENC_PIN1,ENC_PIN2);

// uint8_t payload[PAYLOAD_LEN] ;   // 12 byte string for sending data to coordinator
//   NOW IN SXUTILS.H

// Create an XBee object
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();

// create reusable response objects for responses we expect to handle
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

long timerAnalogIn = 0;
long stopTime = 0;  // for blinking "STOP" on display

int mode = MODE_NORMAL;
long oldPosition = 0; // store encoder position
SXLoco loco;   // construct an SXloco with address etc
AddrSelection addrSelection;  // functions for address selection
long requestTimer = 0;

uint16_t batteryLevel = 0;
long lastBatteryTimer = 0;

long lastBlink = 0 ;

uint8_t trackPower = 0;  // 0=no power, 1=power on
long redrawTime = 0;    // timer for redrawing graphics
long encButtonTimer = 0;   // to check for LONG PRESS of encoder button
uint8_t encButtonPressed = 0;

// Address of receiving XBee
XBeeAddress64 coord64;
ZBTxRequest zbTx;
ZBTxStatusResponse txStatus;


void setup()
{
   
#ifdef DISP_ROTATED
   u8g.setRot180();   // rotate display by 180 degrees
#endif
   
   // Set up XBee port at 9600 baud. This value is most important
   // for the XBee. Make sure the baud rate matches the config
   // setting of your XBee.
#if defined(_TEENSY_LC) || defined(_TEENSY_31)
   Serial.begin(57600); // USB is always 12 Mbit/sec
   Serial.println(HW_REV);
   Serial.println(SW_REV);
   Serial.println("Teensy");
   
   Serial1.begin(9600); // XBee
   Serial.println("Teensy Processor");
   delay(200);
   xbee.setSerial(Serial1);
#else
#ifdef _DEBUG_AVR
   Serial.begin(57600); //XBee
   Serial.println(HW_REV);
   Serial.println(SW_REV);
#ifdef __AVR__
   Serial.println("AVR Processor");
#endif
   
#else
   Serial.begin(9600); //XBee

   delay(200);
   xbee.setSerial(Serial);
#endif   // _DEBUG_AVR
#endif   // defined(_TEENSY_LC) || defined(_TEENSY_31)
   
    // read 4 last used addresses and last used address from EEPROM and init loco
   addrSelection.initFromEEPROM();
   loco.setFromSXData(0);  // use 0 if we don't get data from central sx bus
   sendRequestLoco(loco.getAddress());  // read state from central sx bus   
      
   coord64 = XBeeAddress64(0, 0);  //coordinator address
   zbTx = ZBTxRequest(coord64, (uint8_t *)payload, sizeof(payload));  // 64bit address
   zbTx.setAddress16(0); // addr16 = 0;
   txStatus = ZBTxStatusResponse();

   

   #if defined(_DEBUG) || defined(_DEBUG_AVR)
   Serial.print("loco adr=");
   Serial.println(loco.getAddress());
   #endif

   // init buttons
   pinMode(ENC_BTN,INPUT_PULLUP);
   pinMode(ADDR_BTN,INPUT_PULLUP);
   pinMode(F0_BTN,INPUT_PULLUP);
   pinMode(F1_BTN,INPUT_PULLUP);

   pinMode(SLEEP_RQ,OUTPUT);
   digitalWrite(SLEEP_RQ,LOW);   // wake up xbee
#if defined(_TEENSY_31) || defined(_TEENSY_LC)
   attachInterrupt(PWR_DOWN, powerDown, FALLING);
#endif 
   delay(100);

   #ifdef _TEENSY_LC
   // compare bandgap channel 39 (hack analog.c AD27) to Vcc   LC
   //  datasheet says  1.00  min/max 0.97/1.03 v
   //see https://forum.pjrc.com/threads/26117-Teensy-3-1-Voltage-sensing-and-low-battery-alert
   analogReference(DEFAULT);
   analogReadResolution(12);
   analogReadAveraging(32);
   PMC_REGSC |= PMC_REGSC_BGBE;
  
   #elif _TEENSY_31
   analogReference(EXTERNAL);
   analogReadResolution(12);
   analogReadAveraging(32); // this one is optional.
   #elif __AVR__   // or atmega328p
   // TODO 
   #endif
   
   delay(10);
   sendBatteryVoltage();
}

void powerDown () {
  // save loco address when powering down
 /* if (EEPROM.read(EEPROM_ADDR_CHANNEL) != loco.getAddress()) {
            EEPROM.write(EEPROM_ADDR_CHANNEL,(0xff)&loco.getAddress());
  } */
}

void updateLocoSpeed() {
   long newPosition = encoder.read() >> 2;
   if (newPosition != oldPosition) {
      int8_t delta = (int8_t)(newPosition-oldPosition);
      oldPosition = newPosition;
      loco.updateSpeed(delta);
   }  // newPosition != oldPosition
}

void updateAddrSel() {
   long newPosition = encoder.read() >> 2;
   if (newPosition != oldPosition) {
      int8_t delta = (int8_t)(newPosition-oldPosition);
      oldPosition = newPosition;
      addrSelection.update(delta);
   }
}

void drawWaiting() {
    // redraw the complete screen in "waiting for response" mode
   u8g.setFont(u8g_font_fur20);
   u8g.setPrintPos(0,30);  // for loco address string
   u8g.print(loco.getAddress());
   u8g.setPrintPos(60,43);  // for speed string
   u8g.print("???");

}

void drawAddressSelection() {
    // redraw the complete screen in "address selection" mode
   u8g.setFont(u8g_font_fur42n);
   u8g.setPrintPos(60,50);
   u8g.print(addrSelection.getCurrentAddress());
   u8g.setFont(u8g_font_fur20);
   u8g.drawStr( 0, 26, "A");
   u8g.drawStr( 0, 56, "?");
   uint8_t sIndex = addrSelection.getCurrentStoredIndex();
   if (sIndex != INVALID_UI8) {
      u8g.setFont(u8g_font_helvB12);
      u8g.drawStr( 74, 64, "SP");
      u8g.setPrintPos(104,64);
      u8g.print(sIndex);
   }
   // zu gross u8g.setFont(u8g_font_fub14);
}

void drawAddressAndSpeed(void) {
   // redraw the complete screen in "standard" mode
   u8g.setFont(u8g_font_fur42n);
   u8g.setPrintPos(60,43);  // for speed string
   u8g.print(loco.getSpeed());  

   u8g.setFont(u8g_font_fur20);
   if (loco.getBackward()) {
      u8g.drawStr( 0, 60, "<");
   } else {
      u8g.drawStr( 0, 60, ">");
   }
   u8g.setFont(u8g_font_fur20);
   u8g.setPrintPos(0,30);  // for loco address string
   u8g.print(loco.getAddress());

   // zu gross u8g.setFont(u8g_font_fub14);
   u8g.setFont(u8g_font_helvB12);
   if (loco.getF0()) {
      u8g.drawStr( 34, 64, "L");
   }
   if (loco.getF1()) {
      u8g.drawStr( 48, 64, "F");
   }

   u8g_uint_t stopPositionX = 84;
   if (batteryLevel < 330) {
      u8g.drawStr( 62, 64, "Batt");
      stopPositionX = 100;  // move to right if "Batt" is shown
   }
   if (trackPower == 0) {  // show blinking "STOP" 
      if ( (millis()-stopTime) < 700) {
         u8g.drawStr(stopPositionX, 64, "STOP");
      } else if ((millis()-stopTime) >= 1400) {
         stopTime = millis();
      }
   }

}

/** start address selection mode when address button is clicked
 *                                           (and released)
 */
void addrButtonClick() {

   if (addrButton.risingEdge()) {
      addrButtonPressed = 0;
      
   } else if (addrButton.fallingEdge()) {
      addrButtonTimer = millis(); // store for later use: determine 
          // duration of pressing
      addrButtonPressed = 1;
    
      // toggle address/normal mode
      switch (mode) {
         case MODE_NORMAL:  
            // start with address selection
            oldPosition = encoder.read() >> 2;  // reset encoder position
            // init address selection
            addrSelection.init(indexLastAddr);      
            mode = MODE_ADDRESS_SELECTION;
            break;
         case MODE_ADDRESS_SELECTION:
            // finish address selection
             oldPosition = encoder.read() >> 2;  // reset encoder position
             if (addrSelection.finish()) {
               // loco address has changed
               loco.setIndex(indexLastAddr);
               sendRequestLoco(loco.getAddress());  // read state from central sx bus
               mode = MODE_WAITING_FOR_RESPONSE;
               requestTimer=millis();
             } else {
               mode = MODE_NORMAL;
             }
            break;
      
         default:
            mode = MODE_NORMAL;
            break;
      }
      
   }
}

void encButtonClick() {

   #ifdef _DEBUG
   Serial.println("encButton clicked.");
   #endif
   if (encButton.risingEdge()) {
      encButtonPressed = 0;
   } else {
      encButtonTimer = millis();
      encButtonPressed = 1;
      loco.stop();  // does not change direction!
      drawAddressAndSpeed();
   }
}

void f0ButtonClick() {
   if (f0Button.risingEdge()) return;
   loco.toggleF0();
   drawAddressAndSpeed();
}

void f1ButtonClick() {
   if (f1Button.risingEdge()) return;
   loco.toggleF1();
   drawAddressAndSpeed();
}

void sendSpeed() {
   // address first 
   uint8_t addr = loco.getAddress();
   uint8_t sx= loco.getSXData();
   makePayloadCommandAnd2Numbers('S',addr, sx);
#if defined(_DEBUG) || defined(_DEBUG_AVR)
   sendPayloadToSerial();
#endif   // defined(_DEBUG) || defined(_DEBUG_AVR)

#ifndef _DEBUG_AVR
   xbee.send(zbTx);
#endif

}

/** request loco info from SX interface 
 * after this request, we are waiting for a second
 * to get a response from central - then setting 
 * loco to "zero" if no response
 */
void sendRequestLoco(uint8_t addr) {
   makePayloadCommandAndNumber('R',addr);

#if defined(_DEBUG) || defined(_DEBUG_AVR)
   sendPayloadToSerial();
#endif   // defined(_DEBUG) || defined(_DEBUG_AVR)

#ifndef _DEBUG_AVR
   xbee.send(zbTx);
#endif
}

/** read incoming commands from xbee:
*  "P 0" (= track power off) - or "P 1"
*  "44 134" (=loco addr 44 has sx data=134)
*/
void interpretCommand(String s) {
   if ( ( s.charAt(0) != 'f') && (s.charAt(0) != 'F')) return; // only feedback cmd read
   
   // loco or trackpower feedback command
   // example:  "F44 31" = loco #44 has SX data= 31,
   //       (standard selectrix data format, 8 bit)
   s = s.substring(1);  // remove "F "
   int n= s.indexOf(' '); // command split by space,i.e. 2 numbers 
   if (n != -1) {
      int i_ch = s.toInt();
      String s2 = s.substring(n+1,s.length());
      int i_value = s2.toInt();
      if (i_ch == 127) {
         // trackpower command on channel 127
         if (i_value == 0) {
            trackPower = 0;
         } else {
            trackPower = 1;
         }
      }  else if ((loco.getAddress() == i_ch) && (i_value >= 0) && (i_value <= 255) ) {
         // received sx data for the currently controlled loco
         loco.setFromSXData((uint8_t)i_value) ;
         if (mode == MODE_WAITING_FOR_RESPONSE) {
            // reset to normal if response received
            mode = MODE_NORMAL;
         }
      }
   }
}

void sendTrackPower() {
   // send value of trackpower on channel 127
   makePayloadCommandAnd2Numbers('S',127,trackPower);

#ifndef _DEBUG_AVR
   xbee.send(zbTx);
#endif

#if defined(_DEBUG) || defined(_DEBUG_AVR)
   sendPayloadToSerial();
#endif

}

// send payload as characters to Serial
void sendPayloadToSerial() {
  for (int i=0; i<12; i++) {
      if (payload[i] == 0) break;
      Serial.print((char)payload[i]);     
  }
  Serial.println();
}

void sendBatteryVoltage() {
 //TODO implement with simple analogRead()  
   #ifdef _TEENSY_LC
   uint32_t mv=0;
   // compare bandgap channel 39 (hack analog.c AD27) to Vcc   LC
   //  datasheet says  1.00  min/max 0.97/1.03 v
   //  see https://forum.pjrc.com/threads/26117-Teensy-3-1-Voltage-sensing-and-low-battery-alert

   mv = 1000 * 4096 /analogRead(39);
   #elif _TEENSY_31
   uint32_t mv=0;
   // for Teensy 3.1, only valid between 2.0V and 3.5V. Returns in millivolts.
   uint32_t x = analogRead(39);
   mv = (178*x*x + 2688743864 - 1182047 * x) / 371794;
   #else
   // avr - not yet implemented
   uint16_t mv = 3500;
   // mv = AnalogRead(A2) * 3 /2;
   #endif
   

   mv = mv/10; // reduce to 3 digits
   makePayloadCommandAndNumber('B',(uint16_t)mv);
   
#if defined(_DEBUG) || defined(_DEBUG_AVR)
   sendPayloadToSerial();
#endif // defined(_DEBUG) || defined(_DEBUG_AVR)

#ifndef _DEBUG_AVR
   xbee.send(zbTx);
#endif
   batteryLevel = mv;
}

void loop()
{  if (encButton.update()) encButtonClick();
   if (addrButton.update()) addrButtonClick();
   if (f0Button.update()) f0ButtonClick();
   if (f1Button.update()) f1ButtonClick();

#ifndef _DEBUG_AVR
   xbee.readPacket();

   if (xbee.getResponse().isAvailable()) {
      // got something
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
         // got a zb rx packet
         // now fill our zb rx class
         xbee.getResponse().getZBRxResponse(rx);
         String s="";
         for (int i=0; i<rx.getDataLength(); i++) {
            char c = rx.getData(i);
            s += c;
         }
         interpretCommand(s);
      }
   } else if (xbee.getResponse().isError()) {
      //nss.print("Error reading packet.  Error code: ");
   }
#else  // read commands from serial port 
 // the following 9 lines add 2.5k PROGMEM use
   static String s="";
   while (Serial.available() >0) {
      char c = Serial.read();
      if (c == '\n') { //end of string       
         interpretCommand(s);  
         s="";
      } else {
         s +=c;
      }
   } 
#endif


   if (mode == MODE_ADDRESS_SELECTION) {
      // selecting a new address for the loco
      // DO NOT SEND xbee Loco messages during the selection
      if ((millis() - timerAnalogIn) >200) {
         timerAnalogIn = millis();
         updateAddrSel();
         // picture loop
         u8g.firstPage();
         do {
            drawAddressSelection();
         } while( u8g.nextPage() );
      }
   } else if (mode == MODE_NORMAL) {
      // no address selection running - get speed value
      if ((millis() - timerAnalogIn) >200) {
         timerAnalogIn = millis();
         updateLocoSpeed();
         if ( loco.hasChanged() ) {
            loco.resetChanged();
            sendSpeed();
         }
         // picture loop
         u8g.firstPage();
         do {
            drawAddressAndSpeed();
         } while( u8g.nextPage() );
      } //endif timerAnalogIn
      if ( encButtonPressed && 
           ((millis() - encButtonTimer) > 1300 ) ) {
         // toggle trackPower
         trackPower = !trackPower;
         sendTrackPower();
         encButtonTimer = millis();  // toggle again after 2 secs
         #ifdef _DEBUG
         Serial.println("toggled trackpower.");
         #endif
      }  // endif encButtonTimer

      if ( (millis() - lastBatteryTimer) > 60000 ) {   // every 1 minute
         lastBatteryTimer=millis();
         sendBatteryVoltage();
      }  // endif lastBatteryTime     

   } else if (mode == MODE_WAITING_FOR_RESPONSE) {
      // check for timeout for waiting for response
      if ((millis() - requestTimer) > 1000 )  {
         loco.setFromSXData(0); // reset loco
         mode = MODE_NORMAL;      
      }
      // picture loop
         u8g.firstPage();
         do {
            drawWaiting();
         } while( u8g.nextPage() );
      
   } // endif "mode" == ..

   delay(1);
}
